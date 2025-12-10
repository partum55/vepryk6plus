#!/usr/bin/env python3
import socket
import serial
import time
import select
from pymavlink import mavutil

# Configuration
SERIAL_PORT = '/dev/ttyS0'
BAUD_RATE = 57600
TCP_PORT = 14550
HEARTBEAT_TIMEOUT = 1.0
FAILSAFE_INTERVAL = 0.1
CHANNEL6_LOW_DURATION = 2.0

def main():
    print(f"=== Python MAVLink Bridge ===")
    print(f"Serial: {SERIAL_PORT} @ {BAUD_RATE}")
    print(f"TCP: 0.0.0.0:{TCP_PORT}")
    
    # Open Serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
    except Exception as e:
        print(f"Error opening serial: {e}")
        return

    # Setup TCP Server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', TCP_PORT))
    server.listen(1)
    server.setblocking(False)
    
    client = None
    
    # MAVLink helper
    mav = mavutil.mavlink.MAVLink(None)
    mav.srcSystem = 255
    mav.srcComponent = 1
    
    # State
    last_heartbeat_time = 0  # 0 means waiting for first heartbeat
    failsafe_active = False
    failsafe_start_time = 0
    last_failsafe_send = 0
    last_heartbeat_send = 0
    
    print("Bridge Ready. Waiting for connections...")
    
    while True:
        inputs = [server, ser]
        if client:
            inputs.append(client)
            
        try:
            readable, _, _ = select.select(inputs, [], [], 0.05)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Select error: {e}")
            break
            
        now = time.time()
        
        for s in readable:
            # New TCP Connection
            if s is server:
                conn, addr = server.accept()
                conn.setblocking(False)
                if client:
                    client.close()
                client = conn
                print(f"Client connected: {addr}")
            
            # Data from TCP Client (GCS)
            elif s is client:
                try:
                    data = client.recv(4096)
                    if data:
                        # Forward to Serial
                        ser.write(data)
                        
                        # Parse for Heartbeat (GCS -> Vehicle)
                        msgs = mav.parse_buffer(data)
                        if msgs:
                            for msg in msgs:
                                if msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == 255:
                                    last_heartbeat_time = now
                                    if failsafe_active:
                                        print("Heartbeat restored. Failsafe OFF.")
                                        failsafe_active = False
                    else:
                        print("Client disconnected")
                        client.close()
                        client = None
                except Exception as e:
                    print(f"TCP Read Error: {e}")
                    if client:
                        client.close()
                    client = None
            
            # Data from Serial (Vehicle)
            elif s is ser:
                try:
                    data = ser.read(4096)
                    if data and client:
                        try:
                            client.send(data)
                        except:
                            client.close()
                            client = None
                except Exception as e:
                    print(f"Serial Read Error: {e}")
        
        # Failsafe Logic
        if last_heartbeat_time > 0 and now - last_heartbeat_time > HEARTBEAT_TIMEOUT:
            if not failsafe_active:
                print(f"FAILSAFE ACTIVATED (No heartbeat for {now - last_heartbeat_time:.1f}s)")
                failsafe_active = True
                failsafe_start_time = now
            
            # Send RC Override
            if now - last_failsafe_send > FAILSAFE_INTERVAL:
                # Channel 6 logic
                chan6 = 1000 if (now - failsafe_start_time < CHANNEL6_LOW_DURATION) else 1500
                
                msg = mav.rc_channels_override_encode(
                    1, 1, # target sys, comp
                    1500, 65535, 1500, 65535, 65535, chan6, 65535, 65535, # 1-8
                )
                ser.write(msg.pack(mav))
                
                last_failsafe_send = now
                
            # Send Heartbeat (keep link alive)
            if now - last_heartbeat_send > 1.0:
                msg = mav.heartbeat_encode(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                buf = msg.pack(mav)
                if client:
                    try:
                        client.send(buf)
                    except:
                        pass
                ser.write(buf)
                last_heartbeat_send = now

if __name__ == "__main__":
    main()
