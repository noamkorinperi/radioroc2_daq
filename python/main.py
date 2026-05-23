"""
================================================================
main.py — RADIOROC2 DAQ PC Application

Entry point. Connects to STM32 via USB CDC (Virtual COM Port),
receives events, builds per-channel and combined histograms,
and displays them when the STM32 signals histogram-ready.

Usage:
    python main.py --port COM3          (Windows)
    python main.py --port /dev/ttyACM0 (Linux)
    python main.py --port /dev/tty.usbmodem* (macOS)

    Optional flags:
    --mode  count --param 1000   display histogram every 1000 events
    --mode  time  --param 30     display histogram every 30 seconds
    --calib 3.1                  set S_T_REF (ADC counts/keV)
================================================================
"""

import argparse
import sys
import signal

from comms   import USBComm
from daq     import DAQSession
from plotter import HistogramPlotter


def parse_args():
    parser = argparse.ArgumentParser(
        description="RADIOROC2 DAQ data acquisition and display")

    parser.add_argument("--port",  required=True,
                        help="Serial port (e.g. COM3 or /dev/ttyACM0)")
    parser.add_argument("--baud",  type=int, default=115200,
                        help="Baud rate (default 115200)")
    parser.add_argument("--mode",  choices=["count", "time"],
                        default="count",
                        help="Histogram trigger mode (default: count)")
    parser.add_argument("--param", type=int, default=1000,
                        help="N events (count mode) or T seconds (time mode)")
    parser.add_argument("--calib", type=float, default=3.1,
                        help="Calibration factor S_T_REF [ADC counts/keV]")
    parser.add_argument("--channels", type=int, nargs="+",
                        default=[0, 1, 2],
                        help="Active channel numbers (default: 0 1 2)")
    parser.add_argument("--bins", type=int, default=512,
                        help="Number of histogram bins (default: 512)")
    parser.add_argument("--energy-max", type=float, default=3000.0,
                        help="Maximum energy axis in keV (default: 3000)")

    return parser.parse_args()


def main():
    args = parse_args()

    print(f"[DAQ] Connecting to {args.port}...")

    # --- USB communication ---
    comm = USBComm(port=args.port, baudrate=args.baud)
    if not comm.connect():
        print(f"[ERROR] Could not open {args.port}")
        sys.exit(1)
    print(f"[DAQ] Connected.")

    # --- Send configuration to STM32 ---
    comm.send_mode(args.mode, args.param)
    comm.send_calibration(args.calib)
    print(f"[DAQ] Mode: {args.mode.upper()}, param: {args.param}, "
          f"calib: {args.calib:.3f} counts/keV")

    # --- DAQ session ---
    session = DAQSession(channels=args.channels,
                         bins=args.bins,
                         energy_max=args.energy_max)

    # --- Plotter ---
    plotter = HistogramPlotter(channels=args.channels,
                               bins=args.bins,
                               energy_max=args.energy_max)

    # --- Graceful exit on Ctrl+C ---
    def on_exit(sig, frame):
        print("\n[DAQ] Stopping...")
        comm.disconnect()
        plotter.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_exit)

    # --- Main receive loop ---
    print("[DAQ] Receiving events. Press Ctrl+C to stop.\n")

    while True:
        msg = comm.receive_packet()
        if msg is None:
            continue

        if msg["type"] == "event":
            channel    = msg["channel"]
            energy_kev = msg["energy_kev"]

            # Ignore invalid events (energy = -1.0, loop not converged)
            if energy_kev < 0.0:
                continue

            session.add_event(channel, energy_kev)

        elif msg["type"] == "histogram_ready":
            print(f"[DAQ] Histogram trigger — "
                  f"{session.total_events()} events total")
            plotter.update(session)
            plotter.show()


if __name__ == "__main__":
    main()
