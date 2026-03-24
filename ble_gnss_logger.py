#!/usr/bin/env python3
"""BLE GNSS logger for UM980 via Nordic UART Service.

Connects to UM980_C6_GPS over BLE, logs all NMEA sentences to file,
and displays live status (fix, satellites, coordinates) in terminal.

Usage:
    /tmp/ble_env/bin/python ble_gnss_logger.py [--output logs/test.nmea] [--addr 58:E6:C5:C5:18:22]
"""

import asyncio
import argparse
import os
import sys
from datetime import datetime

# Nordic UART Service UUIDs
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notifications FROM device
NUS_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write TO device

DEFAULT_ADDR = "58:E6:C5:C5:18:22"


def parse_gga(fields):
    """Parse $GNGGA sentence, return status dict."""
    info = {}
    if len(fields) < 15:
        return info
    # Time
    t = fields[1]
    if len(t) >= 6:
        info["utc"] = f"{t[0:2]}:{t[2:4]}:{t[4:]}"
    # Lat
    if fields[2] and fields[3]:
        deg = float(fields[2][:2])
        mins = float(fields[2][2:])
        lat = deg + mins / 60.0
        if fields[3] == "S":
            lat = -lat
        info["lat"] = lat
    # Lon
    if fields[4] and fields[5]:
        deg = float(fields[4][:3])
        mins = float(fields[4][3:])
        lon = deg + mins / 60.0
        if fields[5] == "W":
            lon = -lon
        info["lon"] = lon
    # Fix quality
    info["fix"] = int(fields[6]) if fields[6] else 0
    # Num satellites
    info["nsats"] = int(fields[7]) if fields[7] else 0
    # HDOP
    if fields[8]:
        info["hdop"] = float(fields[8])
    # Altitude
    if fields[9]:
        info["alt"] = float(fields[9])
    return info


def parse_gsv(fields):
    """Parse $GxGSV sentence, return (system, total_sats)."""
    if len(fields) < 4:
        return None, 0
    prefix = fields[0]  # $GPGSV, $GLGSV, $GAGSV, $GBGSV, $GQGSV
    sys_map = {
        "GP": "GPS", "GL": "GLO", "GA": "GAL",
        "GB": "BDS", "GQ": "QZSS", "GN": "GNSS",
    }
    sys_id = prefix[1:3] if len(prefix) >= 3 else "??"
    system = sys_map.get(sys_id, sys_id)
    total = int(fields[3]) if fields[3] else 0
    return system, total


def format_status(gga, gsv_counts, line_count, start_time):
    """Format one-line terminal status."""
    elapsed = datetime.now() - start_time
    mins = int(elapsed.total_seconds()) // 60
    secs = int(elapsed.total_seconds()) % 60

    fix_names = {0: "No fix", 1: "GPS", 2: "DGPS", 4: "RTK fix", 5: "RTK float", 6: "Dead reck"}
    fix_str = fix_names.get(gga.get("fix", 0), f"Fix={gga.get('fix', '?')}")

    parts = [f"{mins:02d}:{secs:02d}"]

    if gga.get("fix", 0) > 0:
        parts.append(f"\033[92m{fix_str}\033[0m")  # green
        if "lat" in gga and "lon" in gga:
            parts.append(f"{gga['lat']:.6f},{gga['lon']:.6f}")
        if "alt" in gga:
            parts.append(f"alt={gga['alt']:.1f}m")
        parts.append(f"sats={gga.get('nsats', 0)}")
        if "hdop" in gga:
            parts.append(f"hdop={gga['hdop']:.1f}")
    else:
        parts.append(f"\033[91mNo fix\033[0m")  # red
        parts.append(f"sats={gga.get('nsats', 0)}")

    # Per-system satellite counts from GSV
    if gsv_counts:
        sv_str = " ".join(f"{k}:{v}" for k, v in sorted(gsv_counts.items()))
        parts.append(f"[{sv_str}]")

    if gga.get("utc"):
        parts.append(f"UTC={gga['utc']}")

    parts.append(f"({line_count} lines)")
    return " | ".join(parts)


async def main():
    from bleak import BleakClient

    parser = argparse.ArgumentParser(description="BLE GNSS logger for UM980")
    parser.add_argument("--output", "-o", default=None,
                        help="Output log file (default: logs/YYYYMMDD_HHMMSS.nmea)")
    parser.add_argument("--addr", default=DEFAULT_ADDR,
                        help=f"BLE device address (default: {DEFAULT_ADDR})")
    parser.add_argument("--duration", "-d", type=int, default=0,
                        help="Stop after N seconds (0 = run until Ctrl+C)")
    args = parser.parse_args()

    # Create log file path
    if args.output:
        log_path = args.output
    else:
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"logs/{ts}.nmea"

    print(f"UM980 BLE GNSS Logger")
    print(f"Device:  {args.addr}")
    print(f"Logfile: {log_path}")
    print()

    # State
    buf = bytearray()
    gga_info = {}
    gsv_counts = {}
    line_count = 0
    start_time = datetime.now()
    log_file = open(log_path, "w", buffering=1)  # line-buffered

    def on_data(sender, data: bytearray):
        nonlocal buf, gga_info, gsv_counts, line_count

        buf.extend(data)
        while b'\n' in buf:
            raw_line, buf = buf.split(b'\n', 1)
            text = raw_line.decode('ascii', errors='replace').strip()
            if not text:
                continue

            # Log with timestamp
            ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            log_file.write(f"{ts} {text}\n")
            line_count += 1

            # Parse for display
            fields = text.split(',')
            sentence = fields[0] if fields else ""

            if sentence in ("$GNGGA", "$GPGGA"):
                try:
                    gga_info = parse_gga(fields)
                except (ValueError, IndexError):
                    pass
                # Print status line after each GGA
                status = format_status(gga_info, gsv_counts, line_count, start_time)
                print(f"\r\033[K{status}", end="", flush=True)

            elif "GSV" in sentence:
                try:
                    sys_name, total = parse_gsv(fields)
                    if sys_name:
                        gsv_counts[sys_name] = total
                except (ValueError, IndexError):
                    pass

    # Connect with retry — use address directly (no pre-scan needed)
    client = None
    for attempt in range(5):
        print(f"Connecting to {args.addr} (attempt {attempt+1}/5)...", end="", flush=True)
        try:
            client = BleakClient(args.addr, timeout=15.0)
            await client.connect()
            break
        except Exception as e:
            print(f" failed: {e}")
            client = None
            if attempt < 4:
                await asyncio.sleep(3)
            else:
                print("Giving up after 5 attempts")
                log_file.close()
                return

    try:
        print(f" connected!")
        await client.start_notify(NUS_TX_CHAR_UUID, on_data)
        print(f"Logging to {log_path} — Ctrl+C to stop\n")

        try:
            if args.duration > 0:
                await asyncio.sleep(args.duration)
            else:
                while True:
                    await asyncio.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            await client.stop_notify(NUS_TX_CHAR_UUID)
            await asyncio.sleep(0.2)
            log_file.close()
            print(f"\n\nStopped. {line_count} lines logged to {log_path}")
    finally:
        if client:
            await client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
