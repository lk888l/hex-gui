#!/usr/bin/env python3
"""Read-only HopeA3 SocketCAN audit; sends only expedited SDO upload requests.

Does not initialize, enable, clear faults, change limits, or command motion.
Run with the GUI's active SocketCAN interface; avoid concurrent SDO operations.
"""

import argparse
import datetime
import json
import socket
import struct
import time


OBJECTS = [
    ("vendor_id", 0x1018, 1, "I"),
    ("product_code", 0x1018, 2, "I"),
    ("revision", 0x1018, 3, "I"),
    ("mode_display", 0x4402, 0, "B"),
    ("detailed_error", 0x453F, 0, "H"),
    ("compressed_mit_enabled", 0x4103, 1, "B"),
    ("mit_velocity_rev_s", 0x4102, 2, "f"),
    ("mit_kp_raw", 0x4102, 4, "H"),
    ("mit_kd_raw", 0x4102, 5, "H"),
    ("mit_result_limit_permille", 0x4102, 6, "H"),
    ("mit_gain_factor", 0x4102, 7, "f"),
    ("max_torque_permille", 0x4572, 0, "H"),
    ("peak_torque_nm", 0x4576, 0, "f"),
    ("velocity_rev_s", 0x456C, 0, "f"),
    ("torque_permille", 0x4577, 0, "h"),
]


def upload(bus, nid, index, subindex, fmt, timeout):
    # Drain old replies; an outstanding transaction must not be attributed to us.
    bus.setblocking(False)
    while True:
        try:
            bus.recv(72)
        except BlockingIOError:
            break
    payload = struct.pack("<BHBI", 0x40, index, subindex, 0)
    bus.send(struct.pack("=IB3x8s", 0x600 + nid, 8, payload))
    deadline = time.monotonic() + timeout
    while (remaining := deadline - time.monotonic()) > 0:
        bus.settimeout(remaining)
        try:
            frame = bus.recv(72)
        except socket.timeout:
            break
        cob_id, length = struct.unpack_from("=IB", frame)
        data = frame[8:8 + length]
        if cob_id != 0x580 + nid or len(data) != 8:
            continue
        if data[1:4] != struct.pack("<HB", index, subindex):
            continue
        if data[0] == 0x80:
            return {"error": f"SDO abort 0x{int.from_bytes(data[4:], 'little'):08X}"}
        size = struct.calcsize(fmt)
        if data[0] != (0x43 | ((4 - size) << 2)):
            return {"error": f"Unexpected SDO response 0x{data[0]:02X}"}
        return struct.unpack("<" + fmt, data[4:4 + size])[0]
    return {"error": "SDO timeout"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--timeout", type=float, default=0.3)
    args = parser.parse_args()
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                       b"".join(struct.pack("=II", 0x580 + nid, 0xC00007FF) for nid in (21, 22, 23)))
        bus.bind((args.interface,))
        result = {"timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  "interface": args.interface, "read_only": True, "motors": []}
        for nid in (21, 22, 23):
            motor = {"node_id": nid}
            for name, index, subindex, fmt in OBJECTS:
                motor[name] = upload(bus, nid, index, subindex, fmt, args.timeout)
            if isinstance(motor["mit_kd_raw"], int) and isinstance(motor["mit_gain_factor"], float):
                motor["kd_si"] = motor["mit_kd_raw"] * motor["mit_gain_factor"] / (2 * 3.141592653589793)
            result["motors"].append(motor)
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
