#!/usr/bin/env python3
"""
RTT (Round-Trip Time) Benchmark Tool for MAVLink Commands
Measures delay between sending commands and receiving acknowledgments.
Generates statistics and distribution plots.
"""

import socket
import time
import struct
import json
import argparse
import sys
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from collections import defaultdict
import statistics

# Try to import plotting libraries
try:
    import matplotlib.pyplot as plt
    import numpy as np
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Warning: matplotlib/numpy not installed. Plotting disabled.")
    print("Install with: pip install matplotlib numpy")

# MAVLink constants
MAVLINK_STX_V2 = 0xFD
MAVLINK_STX_V1 = 0xFE

# Message IDs
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70
MAVLINK_MSG_ID_PARAM_REQUEST_READ = 20
MAVLINK_MSG_ID_PARAM_VALUE = 22
MAVLINK_MSG_ID_PING = 4
MAVLINK_MSG_ID_SYSTEM_TIME = 2

# MAV_CMD values
MAV_CMD_REQUEST_MESSAGE = 512
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
MAV_CMD_GET_HOME_POSITION = 410

# MAV_TYPE and MAV_AUTOPILOT
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8

# System IDs
SOURCE_SYSTEM = 255
SOURCE_COMPONENT = 190

# CRC extras for message types (needed for MAVLink v1/v2 CRC)
CRC_EXTRA = {
    MAVLINK_MSG_ID_HEARTBEAT: 50,
    MAVLINK_MSG_ID_COMMAND_LONG: 152,
    MAVLINK_MSG_ID_COMMAND_ACK: 143,
    MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: 124,
    MAVLINK_MSG_ID_PARAM_REQUEST_READ: 214,
    MAVLINK_MSG_ID_PARAM_VALUE: 220,
    MAVLINK_MSG_ID_PING: 237,
    MAVLINK_MSG_ID_SYSTEM_TIME: 137,
}


def crc16_mcrf4xx(data: bytes, crc: int = 0xFFFF) -> int:
    """Calculate MAVLink CRC (X.25 CRC)"""
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


@dataclass
class RTTMeasurement:
    """Single RTT measurement result"""
    command_type: str
    send_time: float
    recv_time: float
    rtt_ms: float
    success: bool
    seq: int


@dataclass 
class RTTStats:
    """Statistics for a set of RTT measurements"""
    command_type: str
    count: int
    success_count: int
    success_rate: float
    min_ms: float
    max_ms: float
    mean_ms: float
    median_ms: float
    std_ms: float
    percentile_95: float
    percentile_99: float
    measurements: List[float] = field(default_factory=list)


class MAVLinkConnection:
    """Simple MAVLink connection handler"""
    
    def __init__(self, host: str, port: int, debug: bool = False):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.seq = 0
        self.target_system = 1
        self.target_component = 1
        self.recv_buffer = b''
        self.debug = debug

    def connect(self, timeout: float = 10.0) -> bool:
        """Connect and wait for heartbeat"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(timeout)
            self.sock.connect((self.host, self.port))
            self.sock.setblocking(False)
            print(f"Connected to {self.host}:{self.port}")
            
            # Wait for heartbeat
            if not self._wait_for_heartbeat(timeout):
                print("Timeout waiting for heartbeat")
                return False
                
            print(f"Vehicle detected: System {self.target_system}, Component {self.target_component}")
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def _wait_for_heartbeat(self, timeout: float) -> bool:
        """Wait for vehicle heartbeat to get target IDs"""
        start = time.time()
        while time.time() - start < timeout:
            msg = self._receive_message(timeout=0.1)
            if msg and msg['msgid'] == MAVLINK_MSG_ID_HEARTBEAT:
                self.target_system = msg['sysid']
                self.target_component = msg['compid']
                return True
        return False
    
    def disconnect(self):
        """Close connection"""
        if self.sock:
            self.sock.close()
            self.sock = None
    
    def _get_seq(self) -> int:
        """Get next sequence number"""
        seq = self.seq
        self.seq = (self.seq + 1) % 256
        return seq
    
    def _build_mavlink_v1_message(self, msgid: int, payload: bytes) -> bytes:
        """Build MAVLink v1 message"""
        seq = self._get_seq()
        header = struct.pack('<BBBBBB', 
            MAVLINK_STX_V1,
            len(payload),
            seq,
            SOURCE_SYSTEM,
            SOURCE_COMPONENT,
            msgid
        )
        
        # CRC calculation
        crc_data = header[1:] + payload
        crc = crc16_mcrf4xx(crc_data)
        if msgid in CRC_EXTRA:
            crc = crc16_mcrf4xx(bytes([CRC_EXTRA[msgid]]), crc)
        
        return header + payload + struct.pack('<H', crc)
    
    def send_heartbeat(self) -> bool:
        """Send GCS heartbeat"""
        payload = struct.pack('<IBBBBB',
            0,  # custom_mode
            MAV_TYPE_GCS,
            MAV_AUTOPILOT_INVALID,
            0,  # base_mode
            0,  # system_status
            3   # mavlink_version
        )
        msg = self._build_mavlink_v1_message(MAVLINK_MSG_ID_HEARTBEAT, payload)
        return self._send(msg)
    
    def send_command_long(self, command: int, param1: float = 0, param2: float = 0,
                          param3: float = 0, param4: float = 0, param5: float = 0,
                          param6: float = 0, param7: float = 0, confirmation: int = 0) -> int:
        """Send COMMAND_LONG and return sequence number"""
        seq = self.seq  # Capture before increment
        payload = struct.pack('<BBHBfffffff',
            self.target_system,
            self.target_component,
            command,
            confirmation,
            param1, param2, param3, param4, param5, param6, param7
        )
        msg = self._build_mavlink_v1_message(MAVLINK_MSG_ID_COMMAND_LONG, payload)
        if self._send(msg):
            return seq
        return -1
    
    def send_param_request(self, param_id: str) -> int:
        """Send PARAM_REQUEST_READ and return sequence number"""
        seq = self.seq
        param_bytes = param_id.encode('ascii')[:16].ljust(16, b'\x00')
        payload = struct.pack('<BB16sh',
            self.target_system,
            self.target_component,
            param_bytes,
            -1  # param_index (-1 means use param_id)
        )
        msg = self._build_mavlink_v1_message(MAVLINK_MSG_ID_PARAM_REQUEST_READ, payload)
        if self._send(msg):
            return seq
        return -1
    
    def send_ping(self, ping_seq: int) -> int:
        """Send PING message"""
        seq = self.seq
        payload = struct.pack('<QBBi',
            int(time.time() * 1000000),  # time_usec
            self.target_system,
            self.target_component,
            ping_seq
        )
        msg = self._build_mavlink_v1_message(MAVLINK_MSG_ID_PING, payload)
        if self._send(msg):
            return seq
        return -1
    
    def send_request_message(self, message_id: int) -> int:
        """Send MAV_CMD_REQUEST_MESSAGE command"""
        return self.send_command_long(MAV_CMD_REQUEST_MESSAGE, param1=float(message_id))
    
    def _send(self, data: bytes) -> bool:
        """Send raw data"""
        if not self.sock:
            return False
        try:
            self.sock.sendall(data)
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def _receive_message(self, timeout: float = 1.0) -> Optional[Dict]:
        """Receive and parse a MAVLink message"""
        start = time.time()
        
        while time.time() - start < timeout:
            # Try to receive more data
            try:
                data = self.sock.recv(1024)
                if data:
                    self.recv_buffer += data
            except BlockingIOError:
                pass
            except Exception as e:
                print(f"Receive error: {e}")
                return None
            
            # Try to parse a message from buffer
            msg = self._parse_buffer()
            if msg:
                return msg
            
            time.sleep(0.001)  # Small sleep to prevent busy loop
        
        return None
    
    def _parse_buffer(self) -> Optional[Dict]:
        """Try to parse a MAVLink message from buffer"""
        while len(self.recv_buffer) > 0:
            # Find start byte
            if self.recv_buffer[0] == MAVLINK_STX_V1:
                if len(self.recv_buffer) < 8:
                    return None  # Need more data
                
                payload_len = self.recv_buffer[1]
                msg_len = 8 + payload_len  # header(6) + payload + crc(2)
                
                if len(self.recv_buffer) < msg_len:
                    return None
                
                msg_data = self.recv_buffer[:msg_len]
                self.recv_buffer = self.recv_buffer[msg_len:]
                
                parsed = {
                    'sysid': msg_data[3],
                    'compid': msg_data[4],
                    'msgid': msg_data[5],
                    'payload': msg_data[6:-2]
                }
                if self.debug:
                    print(f"    [DEBUG] Received MAVLink v1 msg_id={parsed['msgid']}")
                return parsed

            elif self.recv_buffer[0] == MAVLINK_STX_V2:
                if len(self.recv_buffer) < 12:
                    return None
                
                payload_len = self.recv_buffer[1]
                msg_len = 12 + payload_len  # header(10) + payload + crc(2)
                
                if len(self.recv_buffer) < msg_len:
                    return None
                
                msg_data = self.recv_buffer[:msg_len]
                self.recv_buffer = self.recv_buffer[msg_len:]
                
                msgid = msg_data[7] | (msg_data[8] << 8) | (msg_data[9] << 16)
                
                parsed = {
                    'sysid': msg_data[5],
                    'compid': msg_data[6],
                    'msgid': msgid,
                    'payload': msg_data[10:-2]
                }
                if self.debug:
                    print(f"    [DEBUG] Received MAVLink v2 msg_id={parsed['msgid']}")
                return parsed
            else:
                # Skip invalid byte
                self.recv_buffer = self.recv_buffer[1:]
        
        return None
    
    def wait_for_ack(self, expected_command: int, timeout: float = 2.0, any_ack: bool = False) -> Optional[Dict]:
        """Wait for COMMAND_ACK for specific command

        If any_ack=True, accept any COMMAND_ACK (useful for autopilots that return command=0)
        """
        start = time.time()
        while time.time() - start < timeout:
            msg = self._receive_message(timeout=0.05)
            if msg and msg['msgid'] == MAVLINK_MSG_ID_COMMAND_ACK:
                if len(msg['payload']) >= 3:
                    cmd = struct.unpack('<H', msg['payload'][:2])[0]
                    result = msg['payload'][2]
                    if any_ack or cmd == expected_command or cmd == 0:
                        return {'command': cmd, 'result': result}
        return None
    
    def wait_for_param_value(self, timeout: float = 2.0) -> Optional[Dict]:
        """Wait for PARAM_VALUE response"""
        start = time.time()
        while time.time() - start < timeout:
            msg = self._receive_message(timeout=0.05)
            if msg and msg['msgid'] == MAVLINK_MSG_ID_PARAM_VALUE:
                return msg
        return None
    
    def wait_for_message(self, msgid: int, timeout: float = 2.0) -> Optional[Dict]:
        """Wait for specific message ID"""
        start = time.time()
        while time.time() - start < timeout:
            msg = self._receive_message(timeout=0.05)
            if msg and msg['msgid'] == msgid:
                return msg
        return None


class RTTBenchmark:
    """RTT Benchmark runner"""
    
    def __init__(self, conn: MAVLinkConnection, num_measurements: int = 10000):
        self.conn = conn
        self.num_measurements = num_measurements
        self.measurements: Dict[str, List[RTTMeasurement]] = defaultdict(list)

    def run_heartbeat_benchmark(self) -> List[RTTMeasurement]:
        """Measure RTT using HEARTBEAT -> any response timing

        Sends heartbeat and measures time to receive any message back.
        This works with any autopilot as a basic connectivity/latency test.
        """
        command_name = "HEARTBEAT_RTT"
        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        results = []

        # First, drain the receive buffer
        while self.conn._receive_message(timeout=0.1):
            pass

        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")

            # Drain buffer before each measurement
            self.conn.recv_buffer = b''
            while True:
                try:
                    self.conn.sock.recv(1024)
                except:
                    break

            send_time = time.perf_counter()
            success = self.conn.send_heartbeat()

            if not success:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue

            # Wait for any response
            response = self.conn._receive_message(timeout=2.0)
            recv_time = time.perf_counter()

            if response:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=i
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms (got msg_id={response['msgid']})")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            time.sleep(0.05)  # Slightly longer delay to avoid flooding

        self.measurements[command_name] = results
        return results

    def run_ping_benchmark(self) -> List[RTTMeasurement]:
        """Measure RTT using PING message

        Sends PING and waits for PING response (echo).
        This is the most accurate RTT measurement if supported by autopilot.
        """
        command_name = "PING_RTT"
        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        results = []

        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")

            # Send heartbeat periodically to keep connection alive
            if i % 100 == 0:
                self.conn.send_heartbeat()

            # Drain buffer before measurement
            self.conn.recv_buffer = b''
            while True:
                try:
                    self.conn.sock.recv(1024)
                except:
                    break

            send_time = time.perf_counter()
            seq = self.conn.send_ping(i)

            if seq < 0:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue

            # Wait for PING response
            response = self.conn.wait_for_message(MAVLINK_MSG_ID_PING, timeout=2.0)
            recv_time = time.perf_counter()

            if response:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=i
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            time.sleep(0.01)

        self.measurements[command_name] = results
        return results

    def run_timesync_benchmark(self) -> List[RTTMeasurement]:
        """Measure RTT using TIMESYNC message

        Sends TIMESYNC with tc1=0 and waits for response with tc1 set.
        This is used by PX4 and ArduPilot for time synchronization.
        """
        command_name = "TIMESYNC_RTT"
        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        results = []

        MAVLINK_MSG_ID_TIMESYNC = 111

        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")

            if i % 100 == 0:
                self.conn.send_heartbeat()

            # Drain buffer
            self.conn.recv_buffer = b''
            while True:
                try:
                    self.conn.sock.recv(1024)
                except:
                    break

            # Build TIMESYNC message: tc1=0, ts1=current_time_ns
            ts1 = int(time.time() * 1e9)
            payload = struct.pack('<qq', 0, ts1)  # tc1=0, ts1=timestamp

            send_time = time.perf_counter()
            msg = self.conn._build_mavlink_v1_message(MAVLINK_MSG_ID_TIMESYNC, payload)
            success = self.conn._send(msg)

            if not success:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue

            # Wait for TIMESYNC response
            response = self.conn.wait_for_message(MAVLINK_MSG_ID_TIMESYNC, timeout=2.0)
            recv_time = time.perf_counter()

            if response:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=i
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            time.sleep(0.01)

        self.measurements[command_name] = results
        return results

    def run_combined_benchmark(self) -> tuple:
        """Most accurate test: measures Network and Command RTT simultaneously

        For each measurement:
        1. Send ping and measure network RTT
        2. Immediately send command and measure command RTT
        3. Calculate MAVLink latency = Command RTT - Network RTT

        This ensures both measurements are taken under the same network conditions.
        Stores paired measurements for accurate statistical analysis.
        """
        import subprocess
        import re

        cmd_name = "ACCURATE_CMD_RTT"
        net_name = "NETWORK_RTT"
        mavlink_name = "MAVLINK_LATENCY"
        MAV_CMD_DO_SET_SERVO = 183

        print(f"\nRunning COMBINED benchmark ({self.num_measurements} measurements)...")
        print("  Measuring Network and Command RTT simultaneously for accurate comparison")

        cmd_results = []
        net_results = []
        mavlink_results = []  # Store per-measurement MAVLink latency

        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")

            if i % 100 == 0:
                self.conn.send_heartbeat()

            # === 1. Measure Network RTT (ping) ===
            net_send_time = time.perf_counter()
            try:
                result = subprocess.run(
                    ['ping', '-c', '1', '-W', '2', self.conn.host],
                    capture_output=True,
                    text=True,
                    timeout=3
                )
                net_recv_time = time.perf_counter()

                if result.returncode == 0:
                    match = re.search(r'time[=<](\d+\.?\d*)\s*ms', result.stdout)
                    if match:
                        net_rtt_ms = float(match.group(1))
                    else:
                        net_rtt_ms = (net_recv_time - net_send_time) * 1000
                    net_success = True
                else:
                    net_rtt_ms = 0
                    net_success = False
            except:
                net_recv_time = time.perf_counter()
                net_rtt_ms = 0
                net_success = False

            net_results.append(RTTMeasurement(
                command_type=net_name,
                send_time=net_send_time,
                recv_time=net_recv_time,
                rtt_ms=net_rtt_ms,
                success=net_success,
                seq=i
            ))

            # === 2. Measure Command RTT (immediately after) ===
            # Drain buffer
            self.conn.recv_buffer = b''
            while True:
                try:
                    self.conn.sock.recv(1024)
                except:
                    break

            cmd_send_time = time.perf_counter()
            seq = self.conn.send_command_long(MAV_CMD_DO_SET_SERVO, param1=0, param2=1500)

            if seq < 0:
                cmd_results.append(RTTMeasurement(
                    command_type=cmd_name,
                    send_time=cmd_send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                cmd_success = False
                cmd_rtt_ms = 0
                cmd_recv_time = 0
            else:
                ack = self.conn.wait_for_ack(MAV_CMD_DO_SET_SERVO, timeout=2.0, any_ack=True)
                cmd_recv_time = time.perf_counter()

                if ack:
                    cmd_rtt_ms = (cmd_recv_time - cmd_send_time) * 1000
                    cmd_success = True
                    cmd_results.append(RTTMeasurement(
                        command_type=cmd_name,
                        send_time=cmd_send_time,
                        recv_time=cmd_recv_time,
                        rtt_ms=cmd_rtt_ms,
                        success=True,
                        seq=i
                    ))
                else:
                    cmd_rtt_ms = 0
                    cmd_success = False
                    cmd_results.append(RTTMeasurement(
                        command_type=cmd_name,
                        send_time=cmd_send_time,
                        recv_time=cmd_recv_time,
                        rtt_ms=0,
                        success=False,
                        seq=i
                    ))

            # === 3. Calculate and store MAVLink latency for this pair ===
            if net_success and cmd_success:
                # Only count if command RTT > network RTT (valid measurement)
                mavlink_latency = cmd_rtt_ms - net_rtt_ms
                # If MAVLink is negative, it means network spike during ping - skip this pair
                if mavlink_latency >= 0:
                    mavlink_results.append(RTTMeasurement(
                        command_type=mavlink_name,
                        send_time=cmd_send_time,
                        recv_time=cmd_recv_time,
                        rtt_ms=mavlink_latency,
                        success=True,
                        seq=i
                    ))
                    print(f"  Test {i+1} - Network: {net_rtt_ms:.3f} ms | Command: {cmd_rtt_ms:.3f} ms | MAVLink: {mavlink_latency:.3f} ms")
                else:
                    # Network spike - mark as unreliable
                    mavlink_results.append(RTTMeasurement(
                        command_type=mavlink_name,
                        send_time=cmd_send_time,
                        recv_time=cmd_recv_time,
                        rtt_ms=0,
                        success=False,
                        seq=i
                    ))
                    print(f"  Test {i+1} - Network: {net_rtt_ms:.3f} ms | Command: {cmd_rtt_ms:.3f} ms | SKIPPED (network spike)")
            elif cmd_success:
                print(f"  Test {i+1} - Network: FAILED | Command: {cmd_rtt_ms:.3f} ms")
            elif net_success:
                print(f"  Test {i+1} - Network: {net_rtt_ms:.3f} ms | Command: FAILED")
            else:
                print(f"  Test {i+1} - Network: FAILED | Command: FAILED")

            time.sleep(0.01)

        self.measurements[cmd_name] = cmd_results
        self.measurements[net_name] = net_results
        self.measurements[mavlink_name] = mavlink_results
        return cmd_results, net_results, mavlink_results

    def run_accurate_rtt_benchmark(self) -> List[RTTMeasurement]:
        """Most accurate RTT measurement using DO_SET_SERVO command

        This is the most accurate test because:
        1. COMMAND_LONG always gets COMMAND_ACK response
        2. It simulates real control command (like joystick would trigger)
        3. Measures actual round-trip: send command -> autopilot processes -> ACK received

        Uses MAV_CMD_DO_SET_SERVO with channel 0 (no-op) to avoid affecting anything.
        """
        command_name = "ACCURATE_CMD_RTT"
        MAV_CMD_DO_SET_SERVO = 183

        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        print("  This is the most accurate RTT measurement (COMMAND_LONG -> COMMAND_ACK)")
        results = []

        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")

            if i % 100 == 0:
                self.conn.send_heartbeat()

            # Drain buffer completely
            self.conn.recv_buffer = b''
            while True:
                try:
                    self.conn.sock.recv(1024)
                except:
                    break

            send_time = time.perf_counter()
            # Send DO_SET_SERVO with channel=0, PWM=1500 (neutral, no-op)
            seq = self.conn.send_command_long(MAV_CMD_DO_SET_SERVO, param1=0, param2=1500)

            if seq < 0:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue

            # Wait for COMMAND_ACK
            ack = self.conn.wait_for_ack(MAV_CMD_DO_SET_SERVO, timeout=2.0, any_ack=True)
            recv_time = time.perf_counter()

            if ack:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=i
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            time.sleep(0.01)

        self.measurements[command_name] = results
        return results

    def run_capabilities_benchmark(self) -> List[RTTMeasurement]:
        """Measure RTT using MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES

        This command is supported by most autopilots and returns COMMAND_ACK.
        """
        command_name = "CAPABILITIES_RTT"
        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        results = []

        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")

            if i % 100 == 0:
                self.conn.send_heartbeat()

            # Drain buffer
            self.conn.recv_buffer = b''
            while True:
                try:
                    self.conn.sock.recv(1024)
                except:
                    break

            send_time = time.perf_counter()
            seq = self.conn.send_command_long(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, param1=1.0)

            if seq < 0:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue

            # Wait for any COMMAND_ACK
            ack = self.conn.wait_for_ack(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, timeout=2.0, any_ack=True)
            recv_time = time.perf_counter()

            if ack:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=i
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            time.sleep(0.01)

        self.measurements[command_name] = results
        return results

    def run_rc_override_benchmark(self) -> List[RTTMeasurement]:
        """Measure RTT for RC_CHANNELS_OVERRIDE (simulates joystick control)

        This sends RC channel values like a joystick would.
        Since RC_CHANNELS_OVERRIDE doesn't have ACK, we measure time to receive
        any response from the autopilot (indicating it processed our command).

        Note: This is one-way latency estimation, not true RTT.
        """
        command_name = "RC_OVERRIDE_RTT"
        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        print("  Note: RC_OVERRIDE has no ACK, measuring time to next autopilot response")
        results = []

        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")

            if i % 100 == 0:
                self.conn.send_heartbeat()

            # Drain buffer
            self.conn.recv_buffer = b''
            while True:
                try:
                    self.conn.sock.recv(1024)
                except:
                    break

            # Build RC_CHANNELS_OVERRIDE message
            # Channels: 1500 = neutral, range 1000-2000
            # Set all 8 channels to neutral (1500)
            channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
            payload = struct.pack('<BB8H',
                self.conn.target_system,
                self.conn.target_component,
                *channels
            )

            send_time = time.perf_counter()
            msg = self.conn._build_mavlink_v1_message(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, payload)
            success = self.conn._send(msg)

            if not success:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue

            # Wait for any response (RC_OVERRIDE doesn't have ACK)
            response = self.conn._receive_message(timeout=2.0)
            recv_time = time.perf_counter()

            if response:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=i
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms (response msg_id={response['msgid']})")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            time.sleep(0.01)

        self.measurements[command_name] = results
        return results

    def run_manual_control_benchmark(self) -> List[RTTMeasurement]:
        """Measure RTT for MANUAL_CONTROL message (simulates joystick control)

        MANUAL_CONTROL is the preferred way to send joystick inputs.
        Values: x, y, z, r are -1000 to 1000 (0 = neutral)

        Note: This is one-way latency estimation, not true RTT.
        """
        command_name = "MANUAL_CONTROL_RTT"
        MAVLINK_MSG_ID_MANUAL_CONTROL = 69

        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        print("  Note: MANUAL_CONTROL has no ACK, measuring time to next autopilot response")
        results = []

        # Add CRC extra for MANUAL_CONTROL
        CRC_EXTRA_MANUAL_CONTROL = 243

        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")

            if i % 100 == 0:
                self.conn.send_heartbeat()

            # Drain buffer
            self.conn.recv_buffer = b''
            while True:
                try:
                    self.conn.sock.recv(1024)
                except:
                    break

            # Build MANUAL_CONTROL message
            # x, y, z, r: -1000 to 1000 (0 = neutral)
            # buttons: bitmask of pressed buttons
            payload = struct.pack('<BhhhhH',
                self.conn.target_system,
                0,      # x (pitch) - neutral
                0,      # y (roll) - neutral
                500,    # z (throttle) - 50%
                0,      # r (yaw) - neutral
                0       # buttons - none pressed
            )

            send_time = time.perf_counter()

            # Build message with custom CRC extra
            seq = self.conn._get_seq()
            header = struct.pack('<BBBBBB',
                MAVLINK_STX_V1,
                len(payload),
                seq,
                SOURCE_SYSTEM,
                SOURCE_COMPONENT,
                MAVLINK_MSG_ID_MANUAL_CONTROL
            )
            crc_data = header[1:] + payload
            crc = crc16_mcrf4xx(crc_data)
            crc = crc16_mcrf4xx(bytes([CRC_EXTRA_MANUAL_CONTROL]), crc)
            msg = header + payload + struct.pack('<H', crc)

            success = self.conn._send(msg)

            if not success:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue

            # Wait for any response
            response = self.conn._receive_message(timeout=2.0)
            recv_time = time.perf_counter()

            if response:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=i
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms (response msg_id={response['msgid']})")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            time.sleep(0.01)

        self.measurements[command_name] = results
        return results

    def run_network_rtt_benchmark(self) -> List[RTTMeasurement]:
        """Measure pure network RTT using ICMP ping

        This uses system ping command to measure network latency
        without any MAVLink processing delays.
        """
        import subprocess
        import re

        command_name = "NETWORK_RTT"
        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        print(f"  Measuring ICMP ping RTT to {self.conn.host}")
        results = []

        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")

            send_time = time.perf_counter()

            try:
                # Run ping command (1 packet, 2 second timeout)
                result = subprocess.run(
                    ['ping', '-c', '1', '-W', '2', self.conn.host],
                    capture_output=True,
                    text=True,
                    timeout=3
                )
                recv_time = time.perf_counter()

                if result.returncode == 0:
                    # Parse RTT from ping output
                    # Example: "time=5.23 ms"
                    match = re.search(r'time[=<](\d+\.?\d*)\s*ms', result.stdout)
                    if match:
                        rtt_ms = float(match.group(1))
                    else:
                        rtt_ms = (recv_time - send_time) * 1000

                    results.append(RTTMeasurement(
                        command_type=command_name,
                        send_time=send_time,
                        recv_time=recv_time,
                        rtt_ms=rtt_ms,
                        success=True,
                        seq=i
                    ))
                    print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms")
                else:
                    results.append(RTTMeasurement(
                        command_type=command_name,
                        send_time=send_time,
                        recv_time=recv_time,
                        rtt_ms=0,
                        success=False,
                        seq=i
                    ))
                    print(f"  Test {i+1} - FAILED (no response)")

            except subprocess.TimeoutExpired:
                recv_time = time.perf_counter()
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED (timeout)")
            except Exception as e:
                recv_time = time.perf_counter()
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=i
                ))
                print(f"  Test {i+1} - FAILED ({e})")

            time.sleep(0.01)

        self.measurements[command_name] = results
        return results

    def run_command_ack_benchmark(self, command_name: str, command_id: int,
                                   param1: float = 0) -> List[RTTMeasurement]:
        """Measure RTT for COMMAND_LONG -> COMMAND_ACK"""
        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        results = []
        
        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")
            
            # Send heartbeat periodically
            if i % 100 == 0:
                self.conn.send_heartbeat()
            
            send_time = time.perf_counter()
            seq = self.conn.send_command_long(command_id, param1=param1)
            
            if seq < 0:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=seq
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue
            
            ack = self.conn.wait_for_ack(command_id, timeout=2.0)
            recv_time = time.perf_counter()
            
            if ack:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=seq
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=seq
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            # Small delay between measurements
            time.sleep(0.01)
        
        self.measurements[command_name] = results
        return results
    
    def run_param_request_benchmark(self, param_name: str = "SYSID_THISMAV") -> List[RTTMeasurement]:
        """Measure RTT for PARAM_REQUEST_READ -> PARAM_VALUE"""
        command_name = "PARAM_REQUEST"
        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        results = []
        
        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")
            
            if i % 100 == 0:
                self.conn.send_heartbeat()
            
            send_time = time.perf_counter()
            seq = self.conn.send_param_request(param_name)
            
            if seq < 0:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=seq
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue
            
            response = self.conn.wait_for_param_value(timeout=2.0)
            recv_time = time.perf_counter()
            
            if response:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=seq
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=seq
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            time.sleep(0.01)
        
        self.measurements[command_name] = results
        return results
    
    def run_request_message_benchmark(self, message_id: int = MAVLINK_MSG_ID_SYSTEM_TIME,
                                       message_name: str = "SYSTEM_TIME") -> List[RTTMeasurement]:
        """Measure RTT for REQUEST_MESSAGE command"""
        command_name = f"REQUEST_{message_name}"
        print(f"\nRunning {command_name} benchmark ({self.num_measurements} measurements)...")
        results = []
        
        for i in range(self.num_measurements):
            if i % 1000 == 0 and i > 0:
                print(f"  Progress: {i}/{self.num_measurements}")
            
            if i % 100 == 0:
                self.conn.send_heartbeat()
            
            send_time = time.perf_counter()
            seq = self.conn.send_request_message(message_id)
            
            if seq < 0:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=0,
                    rtt_ms=0,
                    success=False,
                    seq=seq
                ))
                print(f"  Test {i+1} - FAILED (send error)")
                continue
            
            # Wait for either ACK or the requested message
            response = self.conn.wait_for_ack(MAV_CMD_REQUEST_MESSAGE, timeout=2.0)
            recv_time = time.perf_counter()
            
            if response:
                rtt_ms = (recv_time - send_time) * 1000
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=rtt_ms,
                    success=True,
                    seq=seq
                ))
                print(f"  Test {i+1} - RTT: {rtt_ms:.3f} ms")
            else:
                results.append(RTTMeasurement(
                    command_type=command_name,
                    send_time=send_time,
                    recv_time=recv_time,
                    rtt_ms=0,
                    success=False,
                    seq=seq
                ))
                print(f"  Test {i+1} - FAILED (timeout)")

            time.sleep(0.01)
        
        self.measurements[command_name] = results
        return results
    
    def calculate_stats(self, command_type: str) -> Optional[RTTStats]:
        """Calculate statistics for a command type"""
        if command_type not in self.measurements:
            return None
        
        results = self.measurements[command_type]
        successful = [m.rtt_ms for m in results if m.success]
        
        if not successful:
            return RTTStats(
                command_type=command_type,
                count=len(results),
                success_count=0,
                success_rate=0.0,
                min_ms=0, max_ms=0, mean_ms=0, median_ms=0, std_ms=0,
                percentile_95=0, percentile_99=0,
                measurements=[]
            )
        
        return RTTStats(
            command_type=command_type,
            count=len(results),
            success_count=len(successful),
            success_rate=len(successful) / len(results) * 100,
            min_ms=min(successful),
            max_ms=max(successful),
            mean_ms=statistics.mean(successful),
            median_ms=statistics.median(successful),
            std_ms=statistics.stdev(successful) if len(successful) > 1 else 0,
            percentile_95=sorted(successful)[int(len(successful) * 0.95)] if successful else 0,
            percentile_99=sorted(successful)[int(len(successful) * 0.99)] if successful else 0,
            measurements=successful
        )
    
    def print_stats(self, stats: RTTStats):
        """Print statistics to console"""
        print(f"\n{'='*60}")
        print(f"RTT Statistics: {stats.command_type}")
        print(f"{'='*60}")
        print(f"Total measurements:  {stats.count}")
        print(f"Successful:          {stats.success_count} ({stats.success_rate:.1f}%)")
        print(f"{'─'*60}")
        print(f"Min RTT:             {stats.min_ms:.3f} ms")
        print(f"Max RTT:             {stats.max_ms:.3f} ms")
        print(f"Mean RTT:            {stats.mean_ms:.3f} ms")
        print(f"Median RTT:          {stats.median_ms:.3f} ms")
        print(f"Std Dev:             {stats.std_ms:.3f} ms")
        print(f"95th percentile:     {stats.percentile_95:.3f} ms")
        print(f"99th percentile:     {stats.percentile_99:.3f} ms")
        print(f"{'='*60}")

    def print_mavlink_latency_analysis(self):
        """Calculate and print pure MAVLink processing latency

        Uses paired measurements from combined benchmark for accurate analysis.
        """
        cmd_stats = self.calculate_stats("ACCURATE_CMD_RTT")
        net_stats = self.calculate_stats("NETWORK_RTT")
        mavlink_stats = self.calculate_stats("MAVLINK_LATENCY")

        # If we have direct MAVLINK_LATENCY measurements (from combined benchmark)
        if mavlink_stats and mavlink_stats.success_count > 0:
            print(f"\n{'='*60}")
            print(f"MAVLINK LATENCY ANALYSIS (Direct Measurement)")
            print(f"{'='*60}")
            print(f"Valid paired measurements: {mavlink_stats.success_count}")
            print(f"(Measurements with network spikes are excluded)")
            print(f"{'─'*60}")
            print(f"Min MAVLink Latency:       {mavlink_stats.min_ms:.3f} ms")
            print(f"Max MAVLink Latency:       {mavlink_stats.max_ms:.3f} ms")
            print(f"Mean MAVLink Latency:      {mavlink_stats.mean_ms:.3f} ms")
            print(f"Median MAVLink Latency:    {mavlink_stats.median_ms:.3f} ms")
            print(f"Std Dev:                   {mavlink_stats.std_ms:.3f} ms")
            print(f"95th percentile:           {mavlink_stats.percentile_95:.3f} ms")
            print(f"99th percentile:           {mavlink_stats.percentile_99:.3f} ms")
            print(f"{'─'*60}")

            if cmd_stats and net_stats and cmd_stats.success_count > 0 and net_stats.success_count > 0:
                net_pct = (net_stats.median_ms / cmd_stats.median_ms) * 100 if cmd_stats.median_ms > 0 else 0
                mavlink_pct = (mavlink_stats.median_ms / cmd_stats.median_ms) * 100 if cmd_stats.median_ms > 0 else 0

                print(f"LATENCY BREAKDOWN (based on median):")
                print(f"  Network RTT:        {net_stats.median_ms:8.3f} ms  ({net_pct:5.1f}%)")
                print(f"  MAVLink processing: {mavlink_stats.median_ms:8.3f} ms  ({mavlink_pct:5.1f}%)")
                print(f"  Command RTT:        {cmd_stats.median_ms:8.3f} ms  (total)")
            print(f"{'='*60}")

            return {
                'mavlink_latency_ms': {
                    'min': mavlink_stats.min_ms,
                    'median': mavlink_stats.median_ms,
                    'mean': mavlink_stats.mean_ms,
                    'std': mavlink_stats.std_ms,
                    'p95': mavlink_stats.percentile_95,
                    'p99': mavlink_stats.percentile_99,
                    'max': mavlink_stats.max_ms,
                    'valid_count': mavlink_stats.success_count,
                },
                'network_median_ms': net_stats.median_ms if net_stats else 0,
                'command_median_ms': cmd_stats.median_ms if cmd_stats else 0,
            }

        # Fallback: calculate from separate tests (less accurate)
        if not cmd_stats or not net_stats:
            print("\n⚠ Cannot calculate MAVLink latency: need both 'accurate' and 'network' tests")
            return None

        if cmd_stats.success_count == 0 or net_stats.success_count == 0:
            print("\n⚠ Cannot calculate MAVLink latency: no successful measurements")
            return None

        # Calculate MAVLink overhead (fallback method)
        mavlink_min = max(0, cmd_stats.min_ms - net_stats.min_ms)
        mavlink_max = cmd_stats.max_ms - net_stats.max_ms
        mavlink_mean = cmd_stats.mean_ms - net_stats.mean_ms
        mavlink_median = cmd_stats.median_ms - net_stats.median_ms
        mavlink_p95 = cmd_stats.percentile_95 - net_stats.percentile_95
        mavlink_p99 = cmd_stats.percentile_99 - net_stats.percentile_99

        # Calculate percentages
        net_pct = (net_stats.median_ms / cmd_stats.median_ms) * 100 if cmd_stats.median_ms > 0 else 0
        mavlink_pct = 100 - net_pct

        print(f"\n{'='*60}")
        print(f"MAVLINK LATENCY ANALYSIS (Estimated)")
        print(f"{'='*60}")
        print(f"⚠ Note: Tests run separately, less accurate than 'combined'")
        print(f"Formula: MAVLink Latency = Command RTT - Network RTT")
        print(f"{'─'*60}")
        print(f"                    Network    Command    MAVLink")
        print(f"{'─'*60}")
        print(f"Min:              {net_stats.min_ms:8.3f} ms  {cmd_stats.min_ms:8.3f} ms  {mavlink_min:8.3f} ms")
        print(f"Median:           {net_stats.median_ms:8.3f} ms  {cmd_stats.median_ms:8.3f} ms  {mavlink_median:8.3f} ms")
        print(f"Mean:             {net_stats.mean_ms:8.3f} ms  {cmd_stats.mean_ms:8.3f} ms  {mavlink_mean:8.3f} ms")
        print(f"95th percentile:  {net_stats.percentile_95:8.3f} ms  {cmd_stats.percentile_95:8.3f} ms  {mavlink_p95:8.3f} ms")
        print(f"99th percentile:  {net_stats.percentile_99:8.3f} ms  {cmd_stats.percentile_99:8.3f} ms  {mavlink_p99:8.3f} ms")
        print(f"Max:              {net_stats.max_ms:8.3f} ms  {cmd_stats.max_ms:8.3f} ms  {mavlink_max:8.3f} ms")
        print(f"{'─'*60}")
        print(f"LATENCY BREAKDOWN (based on median):")
        print(f"  Network:        {net_stats.median_ms:8.3f} ms  ({net_pct:5.1f}%)")
        print(f"  MAVLink:        {mavlink_median:8.3f} ms  ({mavlink_pct:5.1f}%)")
        print(f"  TOTAL:          {cmd_stats.median_ms:8.3f} ms  (100.0%)")
        print(f"{'='*60}")

        return {
            'network_ms': {
                'min': net_stats.min_ms,
                'median': net_stats.median_ms,
                'mean': net_stats.mean_ms,
                'p95': net_stats.percentile_95,
                'p99': net_stats.percentile_99,
                'max': net_stats.max_ms,
            },
            'command_ms': {
                'min': cmd_stats.min_ms,
                'median': cmd_stats.median_ms,
                'mean': cmd_stats.mean_ms,
                'p95': cmd_stats.percentile_95,
                'p99': cmd_stats.percentile_99,
                'max': cmd_stats.max_ms,
            },
            'mavlink_ms': {
                'min': mavlink_min,
                'median': mavlink_median,
                'mean': mavlink_mean,
                'p95': mavlink_p95,
                'p99': mavlink_p99,
                'max': mavlink_max,
            },
            'breakdown_percent': {
                'network': net_pct,
                'mavlink': mavlink_pct,
            }
        }

    def save_results(self, filename: str = "rtt_results.json", mavlink_analysis: dict = None):
        """Save results to JSON file"""
        data = {}
        for cmd_type, measurements in self.measurements.items():
            stats = self.calculate_stats(cmd_type)
            data[cmd_type] = {
                'stats': {
                    'count': stats.count,
                    'success_count': stats.success_count,
                    'success_rate': stats.success_rate,
                    'min_ms': stats.min_ms,
                    'max_ms': stats.max_ms,
                    'mean_ms': stats.mean_ms,
                    'median_ms': stats.median_ms,
                    'std_ms': stats.std_ms,
                    'percentile_95': stats.percentile_95,
                    'percentile_99': stats.percentile_99,
                },
                'measurements': [
                    {
                        'rtt_ms': m.rtt_ms,
                        'success': m.success,
                        'seq': m.seq
                    }
                    for m in measurements
                ]
            }
        
        # Add MAVLink latency analysis if available
        if mavlink_analysis:
            data['mavlink_latency_analysis'] = mavlink_analysis

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"\nResults saved to {filename}")
    
    def generate_plots(self, output_prefix: str = "rtt"):
        """Generate plots for all measurements"""
        if not PLOTTING_AVAILABLE:
            print("Plotting not available. Install matplotlib and numpy.")
            return
        
        all_stats = []
        for cmd_type in self.measurements:
            stats = self.calculate_stats(cmd_type)
            if stats and stats.success_count > 0:
                all_stats.append(stats)
        
        if not all_stats:
            print("No successful measurements to plot.")
            return
        
        # Create figure with subplots
        n_types = len(all_stats)
        fig = plt.figure(figsize=(16, 5 * n_types + 4))
        
        # Plot for each command type
        for idx, stats in enumerate(all_stats):
            measurements = np.array(stats.measurements)
            
            # Histogram
            ax1 = fig.add_subplot(n_types + 1, 3, idx * 3 + 1)
            ax1.hist(measurements, bins=50, edgecolor='black', alpha=0.7, color='steelblue')
            ax1.axvline(stats.mean_ms, color='red', linestyle='--', label=f'Mean: {stats.mean_ms:.2f}ms')
            ax1.axvline(stats.median_ms, color='green', linestyle='--', label=f'Median: {stats.median_ms:.2f}ms')
            ax1.set_xlabel('RTT (ms)')
            ax1.set_ylabel('Frequency')
            ax1.set_title(f'{stats.command_type} - Distribution')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Time series
            ax2 = fig.add_subplot(n_types + 1, 3, idx * 3 + 2)
            ax2.plot(measurements, alpha=0.5, linewidth=0.5, color='steelblue')
            ax2.axhline(stats.mean_ms, color='red', linestyle='--', label=f'Mean: {stats.mean_ms:.2f}ms')
            ax2.set_xlabel('Measurement #')
            ax2.set_ylabel('RTT (ms)')
            ax2.set_title(f'{stats.command_type} - Time Series')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            
            # Box plot with percentiles
            ax3 = fig.add_subplot(n_types + 1, 3, idx * 3 + 3)
            bp = ax3.boxplot(measurements, vert=True, patch_artist=True)
            bp['boxes'][0].set_facecolor('steelblue')
            bp['boxes'][0].set_alpha(0.7)
            ax3.set_ylabel('RTT (ms)')
            ax3.set_title(f'{stats.command_type} - Box Plot')
            ax3.set_xticklabels([stats.command_type])
            ax3.grid(True, alpha=0.3)
            
            # Add text with stats
            textstr = f'n={stats.success_count}\nμ={stats.mean_ms:.2f}ms\nσ={stats.std_ms:.2f}ms'
            ax3.text(1.3, 0.5, textstr, transform=ax3.transAxes, fontsize=10,
                    verticalalignment='center', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Comparison bar chart (if multiple command types)
        if n_types > 1:
            ax_compare = fig.add_subplot(n_types + 1, 1, n_types + 1)
            x = np.arange(n_types)
            width = 0.25
            
            means = [s.mean_ms for s in all_stats]
            medians = [s.median_ms for s in all_stats]
            p95 = [s.percentile_95 for s in all_stats]
            
            ax_compare.bar(x - width, means, width, label='Mean', color='steelblue')
            ax_compare.bar(x, medians, width, label='Median', color='forestgreen')
            ax_compare.bar(x + width, p95, width, label='95th %ile', color='coral')
            
            ax_compare.set_ylabel('RTT (ms)')
            ax_compare.set_title('RTT Comparison Across Command Types')
            ax_compare.set_xticks(x)
            ax_compare.set_xticklabels([s.command_type for s in all_stats])
            ax_compare.legend()
            ax_compare.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        
        # Save plot
        plot_filename = f"{output_prefix}_plots.png"
        plt.savefig(plot_filename, dpi=150, bbox_inches='tight')
        print(f"Plots saved to {plot_filename}")
        
        # Also create CDF plot
        self._generate_cdf_plot(all_stats, output_prefix)
        
        plt.show()
    
    def _generate_cdf_plot(self, all_stats: List[RTTStats], output_prefix: str):
        """Generate CDF (Cumulative Distribution Function) plot"""
        plt.figure(figsize=(10, 6))
        
        colors = plt.cm.tab10(np.linspace(0, 1, len(all_stats)))
        
        for stats, color in zip(all_stats, colors):
            sorted_data = np.sort(stats.measurements)
            cdf = np.arange(1, len(sorted_data) + 1) / len(sorted_data)
            plt.plot(sorted_data, cdf, label=stats.command_type, color=color, linewidth=2)
            
            # Mark 50th, 95th, 99th percentiles
            for pct, style in [(0.5, ':'), (0.95, '--'), (0.99, '-.')]:
                idx = int(len(sorted_data) * pct)
                if idx < len(sorted_data):
                    plt.axhline(y=pct, color='gray', linestyle=style, alpha=0.3)
        
        plt.xlabel('RTT (ms)')
        plt.ylabel('CDF')
        plt.title('RTT Cumulative Distribution Function')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        cdf_filename = f"{output_prefix}_cdf.png"
        plt.savefig(cdf_filename, dpi=150, bbox_inches='tight')
        print(f"CDF plot saved to {cdf_filename}")


def main():
    parser = argparse.ArgumentParser(description='MAVLink RTT Benchmark Tool')
    parser.add_argument('--host', default='localhost', help='Host to connect to')
    parser.add_argument('--port', type=int, default=14550, help='Port to connect to')
    parser.add_argument('-n', '--num', type=int, default=10000, help='Number of measurements per command')
    parser.add_argument('-o', '--output', default='rtt', help='Output filename prefix')
    parser.add_argument('--no-plots', action='store_true', help='Skip generating plots')
    parser.add_argument('--commands', nargs='+', 
                       choices=['request_msg', 'param', 'heartbeat', 'ping', 'timesync', 'capabilities', 'rc_override', 'manual_control', 'joystick', 'network', 'accurate', 'combined', 'all'],
                       default=['all'],
                       help='Which commands to benchmark (combined = most accurate, measures both simultaneously)')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')

    args = parser.parse_args()
    
    print("="*60)
    print("MAVLink RTT Benchmark Tool")
    print("="*60)
    print(f"Host: {args.host}:{args.port}")
    print(f"Measurements per command: {args.num}")
    print(f"Output prefix: {args.output}")
    print("="*60)
    
    # Connect
    conn = MAVLinkConnection(args.host, args.port, debug=args.debug)
    if not conn.connect():
        sys.exit(1)
    
    try:
        benchmark = RTTBenchmark(conn, num_measurements=args.num)
        
        commands = args.commands
        if 'all' in commands:
            commands = ['combined']
        if 'joystick' in commands:
            commands = [c for c in commands if c != 'joystick'] + ['rc_override', 'manual_control']

        # Run benchmarks
        for cmd in commands:
            if cmd == 'request_msg':
                benchmark.run_request_message_benchmark()
            elif cmd == 'param':
                benchmark.run_param_request_benchmark()
            elif cmd == 'heartbeat':
                benchmark.run_heartbeat_benchmark()
            elif cmd == 'ping':
                benchmark.run_ping_benchmark()
            elif cmd == 'timesync':
                benchmark.run_timesync_benchmark()
            elif cmd == 'capabilities':
                benchmark.run_capabilities_benchmark()
            elif cmd == 'rc_override':
                benchmark.run_rc_override_benchmark()
            elif cmd == 'manual_control':
                benchmark.run_manual_control_benchmark()
            elif cmd == 'network':
                benchmark.run_network_rtt_benchmark()
            elif cmd == 'accurate':
                benchmark.run_accurate_rtt_benchmark()
            elif cmd == 'combined':
                benchmark.run_combined_benchmark()

        # Print statistics
        for cmd_type in benchmark.measurements:
            stats = benchmark.calculate_stats(cmd_type)
            if stats:
                benchmark.print_stats(stats)
        
        # Print MAVLink latency analysis (if both accurate and network tests were run)
        mavlink_analysis = benchmark.print_mavlink_latency_analysis()

        # Save results
        benchmark.save_results(f"{args.output}_results.json", mavlink_analysis)

        # Generate plots
        if not args.no_plots:
            benchmark.generate_plots(args.output)
        
    finally:
        conn.disconnect()
    
    print("\nBenchmark complete!")


if __name__ == '__main__':
    main()

