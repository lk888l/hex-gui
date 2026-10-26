#!/usr/bin/env python3
"""Small SocketCAN CLI for the EL05-compatible standard-frame MIT protocol.

Uses only Python's standard library. Positions are rad, speeds are rad/s:
tau = kp * (p_target - p) + kd * (v_target - v) + tau_feedforward.
The 29-bit native RobStride protocol uses a different wire format.
"""

import argparse
import json
import math
import select
import signal
import socket
import struct
import sys
import time


POSITION_RANGE = (-12.57, 12.57)
VELOCITY_RANGE = (-50.0, 50.0)
TORQUE_RANGE = (-6.0, 6.0)
STOP = bytes.fromhex("fffffffffffffffd")
ENABLE = bytes.fromhex("fffffffffffffffc")
CAN_FRAME = struct.Struct("=IB3x8s")
CAN_FLAG_MASK = 0xE0000000  # EFF, RTR and ERR; only standard data is accepted.


def quantize(value, minimum, maximum, bits):
    """Encode a finite, in-range value, using the full 65535/4095 span."""
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"value {value!r} outside {minimum}..{maximum}")
    return int((value - minimum) * ((1 << bits) - 1) / (maximum - minimum))


def dequantize(value, minimum, maximum, bits):
    return value * (maximum - minimum) / ((1 << bits) - 1) + minimum


def encode_command(position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0):
    """Return the eight MIT bytes: p16, v12, kp12, kd12, torque12."""
    p = quantize(position, *POSITION_RANGE, 16)
    v = quantize(velocity, *VELOCITY_RANGE, 12)
    k_p = quantize(kp, 0.0, 500.0, 12)
    k_d = quantize(kd, 0.0, 5.0, 12)
    t = quantize(torque, *TORQUE_RANGE, 12)
    return bytes((p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (k_p >> 8),
                  k_p & 255, k_d >> 4, ((k_d & 15) << 4) | (t >> 8), t & 255))


def decode_feedback(can_id, data, motor_id=None, reply_id=None):
    """Decode a standard eight-byte reply, or return None for another frame.

    The motor address is byte 0. The CAN reply ID is configurable and may even
    equal the motor ID; it is not assumed to be the manual's default 0xFD.
    Raw mode bits are reported without treating them as proof of enable/stop:
    compatible firmware has been observed to leave them zero during motion.
    """
    if can_id & CAN_FLAG_MASK or not 0 <= can_id <= 0x7FF or len(data) != 8:
        return None
    if reply_id is not None and can_id != reply_id:
        return None
    if motor_id is not None and data[0] != motor_id:
        return None
    if data[:6] == b"\xff" * 6:
        return None  # Special host commands are not telemetry.
    status = data[6] >> 6
    temperature = (((data[6] & 15) << 8) | data[7]) / 10.0
    if status == 3 or temperature > 200.0:
        return None
    return {
        "motor_id": data[0], "reply_id": can_id,
        "position_rad": dequantize((data[1] << 8) | data[2], *POSITION_RANGE, 16),
        "velocity_rad_s": dequantize((data[3] << 4) | (data[4] >> 4), *VELOCITY_RANGE, 12),
        "torque_nm": dequantize(((data[4] & 15) << 8) | data[5], *TORQUE_RANGE, 12),
        "status_raw": status, "fault": bool(data[6] & 0x20),
        "warning": bool(data[6] & 0x10), "temperature_c": temperature,
        "data": data.hex(),
    }


class JsonLog:
    def __init__(self, filename=None):
        self.file = open(filename, "w", encoding="utf-8") if filename else None
        self.started = time.monotonic()

    def write(self, event, phase, **values):
        if self.file:
            row = {"time_s": time.monotonic() - self.started,
                   "event": event, "phase": phase, **values}
            self.file.write(json.dumps(row, ensure_ascii=False) + "\n")
            self.file.flush()

    def close(self):
        if self.file:
            self.file.close()


class SocketCan:
    """Classic CAN transport; never changes interface timing or configuration."""
    def __init__(self, interface, log=None):
        self.log = log
        self.phase = "connect"
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
            self.socket.bind((interface,))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise

    def send(self, motor_id, data):
        if not 0 <= motor_id <= 255 or len(data) != 8:
            raise ValueError("motor ID must be 0..255 and payload must be eight bytes")
        frame = CAN_FRAME.pack(motor_id, len(data), data)
        if self.socket.send(frame) != CAN_FRAME.size:
            raise OSError("short CAN write")
        if self.log:
            self.log.write("tx", self.phase, can_id=motor_id, data=data.hex())

    def receive(self, timeout):
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.socket], [], [], remaining)[0]:
                return None
            try:
                raw, _ancillary, flags, _address = self.socket.recvmsg(72)
            except BlockingIOError:
                continue
            # SocketCAN marks every locally generated frame MSG_DONTROUTE,
            # including commands from other sockets/processes on this host.
            if flags & socket.MSG_DONTROUTE or len(raw) != CAN_FRAME.size:
                continue
            can_id, length, data = CAN_FRAME.unpack(raw)
            if can_id & CAN_FLAG_MASK or can_id > 0x7FF or length != 8:
                continue
            if self.log:
                self.log.write("rx", self.phase, can_id=can_id, data=data.hex())
            return can_id, data

    def close(self):
        self.socket.close()


def get_feedback(bus, motor_id, timeout, reply_id=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = bus.receive(deadline - time.monotonic())
        if frame is None:
            break
        feedback = decode_feedback(*frame, motor_id=motor_id, reply_id=reply_id)
        if feedback is not None:
            return feedback
    raise TimeoutError(f"no MIT feedback from motor {motor_id} within {timeout:.3f}s")


def request_feedback(bus, motor_id, payload=STOP, timeout=0.4, reply_id=None):
    bus.send(motor_id, payload)
    return get_feedback(bus, motor_id, timeout, reply_id)


def repeated_stop(bus, motor_id, reply_id=None):
    """Attempt every stop even when an earlier write or acknowledgement fails."""
    bus.phase = "cleanup_stop"
    last = None
    errors = []
    for _ in range(3):
        try:
            last = request_feedback(bus, motor_id, STOP, 0.15, reply_id)
        except (OSError, TimeoutError) as error:
            errors.append(str(error))
        time.sleep(0.02)
    return last, errors


def check_feedback(feedback, max_speed, max_temperature):
    if feedback["fault"]:
        raise RuntimeError("motor reported a fault")
    if feedback["warning"]:
        raise RuntimeError("motor reported a warning")
    if feedback["temperature_c"] > max_temperature:
        raise RuntimeError(f"temperature exceeds {max_temperature} C")
    if abs(feedback["velocity_rad_s"]) > max_speed:
        raise RuntimeError(f"feedback speed exceeds {max_speed} rad/s")


def scan(bus, args):
    bus.phase = "scan_stop"
    found = {}
    for motor_id in range(args.start, args.end + 1):
        try:
            feedback = request_feedback(bus, motor_id, STOP, args.scan_timeout, args.reply_id)
        except TimeoutError:
            continue
        found[motor_id] = feedback
        print(json.dumps({"event": "found", **feedback}), flush=True)
    print(json.dumps({"event": "scan_complete", "motor_ids": sorted(found),
                      "range": [args.start, args.end]}), flush=True)


def run_motion(bus, args):
    motor_id = args.motor_id
    bus.phase = "initial_stop"
    feedback = request_feedback(bus, motor_id, STOP, args.feedback_timeout, args.reply_id)
    reply_id = feedback["reply_id"]
    check_feedback(feedback, args.max_speed, args.max_temperature)
    initial_position = feedback["position_rad"]
    goal = initial_position + args.offset if args.command == "hold" else 0.0
    if args.command == "hold" and not POSITION_RANGE[0] <= goal <= POSITION_RANGE[1]:
        raise ValueError("relative hold target crosses the documented position range")
    bus.phase = "neutral_before_enable"
    request_feedback(bus, motor_id, encode_command(), args.feedback_timeout, reply_id)
    bus.phase = "enable"
    feedback = request_feedback(bus, motor_id, ENABLE, args.feedback_timeout, reply_id)
    check_feedback(feedback, args.max_speed, args.max_temperature)
    print(json.dumps({"event": "enable_reply", "note": "raw status is not proof of enable",
                      **feedback}), flush=True)
    if args.command == "demo":
        phases = [("neutral", 0.3, 0.0), ("positive_velocity", args.duration, abs(args.velocity)),
                  ("neutral", 0.4, 0.0), ("negative_velocity", args.duration, -abs(args.velocity)),
                  ("neutral", 0.4, 0.0)]
    else:
        phases = [(args.command, args.duration, args.velocity if args.command == "velocity" else 0.0)]
    last_position = feedback["position_rad"]
    displacement = 0.0
    last_rx = time.monotonic()
    last_print = 0.0
    period = 1.0 / args.rate
    for name, duration, velocity in phases:
        bus.phase = name
        print(json.dumps({"event": "phase", "phase": name, "duration_s": duration,
                          "target_velocity_rad_s": velocity, "target_position_rad": goal}), flush=True)
        end = time.monotonic() + duration
        next_tx = time.monotonic()
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_rx > args.feedback_timeout:
                raise TimeoutError("feedback expired during motion")
            if now >= next_tx:
                payload = encode_command(position=goal, velocity=velocity,
                                         kp=args.kp if args.command == "hold" else 0.0,
                                         kd=args.kd, torque=0.0)
                bus.send(motor_id, payload)
                next_tx = now + period
            frame = bus.receive(min(max(0.0, next_tx - time.monotonic()), max(0.0, end - time.monotonic())))
            if frame is None:
                continue
            feedback = decode_feedback(*frame, motor_id=motor_id, reply_id=reply_id)
            if feedback is None:
                continue
            last_rx = time.monotonic()
            check_feedback(feedback, args.max_speed, args.max_temperature)
            position = feedback["position_rad"]
            span = POSITION_RANGE[1] - POSITION_RANGE[0]
            displacement += (position - last_position + span / 2) % span - span / 2
            last_position = position
            if abs(displacement) > args.max_displacement:
                raise RuntimeError(f"displacement exceeds {args.max_displacement} rad")
            if last_rx - last_print >= 0.25:
                print(json.dumps({"event": "feedback", "phase": name,
                                  "displacement_rad": displacement, **feedback}), flush=True)
                last_print = last_rx


def integer(value):
    try:
        return int(value, 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected decimal or 0x-prefixed integer") from error


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
                                     epilog="Examples:\n  %(prog)s scan\n  %(prog)s status 1\n"
                                     "  %(prog)s demo 1 --motion --velocity 1 --kd 0.5\n"
                                     "  %(prog)s hold 1 --motion --offset 0.2\n"
                                     "status requests fresh feedback by sending STOP. Scan also sends STOP.\n"
                                     "Mode bits are shown raw; compatible firmware may leave them zero.\n"
                                     "Default interface: can0. Configure its bitrate externally (normally 1 Mbps).")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--reply-id", type=integer, help="optional host reply CAN ID; otherwise learn from feedback")
    parser.add_argument("--log", help="write TX/RX frames with phase and monotonic times as JSONL")
    subs = parser.add_subparsers(dest="command", required=True)
    scan_parser = subs.add_parser("scan", help="stop/query motor IDs; does not enable or send motion targets")
    scan_parser.add_argument("--start", type=integer, default=0)
    scan_parser.add_argument("--end", type=integer, default=255)
    scan_parser.add_argument("--scan-timeout", type=float, default=0.04)
    for command in ("status", "stop", "demo", "velocity", "hold"):
        sub = subs.add_parser(command)
        sub.add_argument("motor_id", type=integer, nargs="?", default=1)
        if command in ("demo", "velocity", "hold"):
            sub.add_argument("--motion", action="store_true", help="explicitly permit enabling and movement")
            sub.add_argument("--duration", type=float, default=1.5, help="seconds per moving phase (0.1..10)")
            sub.add_argument("--velocity", type=float, default=1.0, help="target speed in rad/s, NOT rpm (maximum magnitude 2)")
            sub.add_argument("--offset", type=float, default=0.2, help="hold target relative to initial position, rad (maximum magnitude 0.25)")
            sub.add_argument("--kp", type=float, default=3.0, help="hold proportional gain, Nm/rad (0..5)")
            sub.add_argument("--kd", type=float, default=0.5, help="MIT damping gain, Nm*s/rad (0.01..0.5)")
            sub.add_argument("--rate", type=float, default=100.0, help="command frequency, Hz (20..200)")
            sub.add_argument("--feedback-timeout", type=float, default=0.4, help="abort after this many seconds without feedback (0.05..1)")
            sub.add_argument("--max-speed", type=float, default=5.0, help="abort feedback speed magnitude, rad/s (0.1..10)")
            sub.add_argument("--max-temperature", type=float, default=60.0, help="abort winding temperature, Celsius (20..80)")
            sub.add_argument("--max-displacement", type=float, default=3.0, help="abort net displacement from initial position, rad (0.05..6)")
    args = parser.parse_args(argv)
    if args.reply_id is not None and not 0 <= args.reply_id <= 0x7FF:
        parser.error("reply ID must be 0..0x7FF")
    if args.command == "scan":
        if not 0 <= args.start <= args.end <= 255:
            parser.error("scan range must be within 0..255")
        if not math.isfinite(args.scan_timeout) or not 0.005 <= args.scan_timeout <= 1:
            parser.error("scan timeout must be 0.005..1 seconds")
    else:
        if not 0 <= args.motor_id <= 255:
            parser.error("motor ID must be 0..255")
    if args.command in ("demo", "velocity", "hold"):
        if not args.motion:
            parser.error("movement requires the explicit --motion flag")
        limits = {"duration": (0.1, 10), "velocity": (-2, 2), "offset": (-0.25, 0.25),
                  "kp": (0, 5), "kd": (0.01, 0.5), "rate": (20, 200),
                  "feedback_timeout": (0.05, 1), "max_speed": (0.1, 10),
                  "max_temperature": (20, 80), "max_displacement": (0.05, 6)}
        for key, (minimum, maximum) in limits.items():
            value = getattr(args, key)
            if not math.isfinite(value) or not minimum <= value <= maximum:
                parser.error(f"--{key.replace('_', '-')} must be within {minimum}..{maximum}")
        if args.command != "hold" and abs(args.velocity) >= args.max_speed:
            parser.error("target speed must be lower than --max-speed")
    return args


def main(argv=None):
    args = parse_args(argv)
    log = JsonLog(args.log)
    bus = None
    result = 0
    previous_handlers = {}

    def terminate(signum, _frame):
        raise InterruptedError(f"received {signal.Signals(signum).name}")

    try:
        if args.command != "scan":
            for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                previous_handlers[signum] = signal.signal(signum, terminate)
        bus = SocketCan(args.interface, log)
        if args.command == "scan":
            scan(bus, args)
        elif args.command in ("status", "stop"):
            bus.phase = args.command
            feedback = request_feedback(bus, args.motor_id, STOP, 0.4, args.reply_id)
            print(json.dumps({"event": "stop_reply", **feedback}), flush=True)
        else:
            run_motion(bus, args)
    except (OSError, TimeoutError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        print(f"ERROR: {error or 'interrupted'}", file=sys.stderr)
        log.write("error", bus.phase if bus else "connect", message=str(error))
        result = 1
    finally:
        try:
            # A second termination request must not interrupt the bounded
            # three-stop cleanup. SIGKILL and power loss cannot be handled.
            for signum in previous_handlers:
                signal.signal(signum, signal.SIG_IGN)
            try:
                if bus and args.command != "scan":
                    feedback, errors = repeated_stop(bus, args.motor_id, args.reply_id)
                    if feedback is not None:
                        print(json.dumps({"event": "final_stop_reply", **feedback}), flush=True)
                    else:
                        print("ERROR: final STOP has no acknowledgement: " + "; ".join(errors), file=sys.stderr)
                        result = 1
            finally:
                if bus:
                    bus.close()
                log.close()
        finally:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
    return result


if __name__ == "__main__":
    sys.exit(main())
