# Vepryk Control

Joystick control system for Vepryk platform with MAVLink communication.

## RTT Benchmark Tool

A Python tool to measure Round-Trip Time (RTT) for MAVLink commands.

### Installation

```bash
pip install -r requirements.txt
```

### Usage

```bash
# Run full benchmark (10000 measurements each command type)
python3 rtt_benchmark.py --host localhost --port 14550

# Quick test with fewer measurements
python3 rtt_benchmark.py -n 100

# Custom host/port
python3 rtt_benchmark.py --host 192.168.1.100 --port 14550 -n 10000

# Only benchmark specific commands
python3 rtt_benchmark.py --commands param           # Parameter request only
python3 rtt_benchmark.py --commands request_msg     # Request message only

# Skip plot generation
python3 rtt_benchmark.py -n 10000 --no-plots

# Custom output filename prefix
python3 rtt_benchmark.py -o my_benchmark
```

### Output Files

- `rtt_results.json` - Raw measurement data and statistics
- `rtt_plots.png` - Histograms, time series, and box plots
- `rtt_cdf.png` - Cumulative distribution function plot

### Command Types Benchmarked

1. **PARAM_REQUEST** - Measures RTT for `PARAM_REQUEST_READ` → `PARAM_VALUE`
2. **REQUEST_SYSTEM_TIME** - Measures RTT for `MAV_CMD_REQUEST_MESSAGE` → `COMMAND_ACK`

### Statistics Provided

- Min, Max, Mean, Median RTT
- Standard Deviation
- 95th and 99th percentiles
- Success rate

## Building Main Control Application

```bash
mkdir build && cd build
cmake ..
make
./vepryk_control
```

