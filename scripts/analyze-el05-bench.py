#!/usr/bin/env python3
"""Offline EL05 bench analysis; never opens CAN or changes motor settings.

Usage: python3 scripts/analyze-el05-bench.py /tmp/el05-bench-can3-20261015 [--png]
Standard-library JSON/SVG output needs no plotting packages. --png additionally
uses a locally installed Chromium/Chrome; it never installs dependencies.
"""

import argparse
import bisect
import json
import math
from pathlib import Path
import shutil
import statistics
import subprocess
import tempfile
from xml.sax.saxutils import escape


WINDOWS = ((0.23, 0.39), (0.63, 0.79))
COLORS = {"command": "#566171", "feedback": "#1463c5", "position": "#16956d", "dictionary": "#d47718"}


def mean(values):
    values = list(values)
    return statistics.fmean(values) if values else None


def stats(values):
    values = sorted(values)
    if not values:
        return {"count": 0}
    def percentile(fraction):
        index = (len(values) - 1) * fraction
        lo, hi = math.floor(index), math.ceil(index)
        return values[lo] + (values[hi] - values[lo]) * (index - lo)
    return {"count": len(values), "mean": mean(values), "min": values[0], "max": values[-1],
            "median": statistics.median(values), "p95": percentile(0.95),
            "stdev": statistics.pstdev(values)}


def rate(points, key="t_s"):
    times = [p[key] for p in points]
    intervals = [(b - a) * 1000 for a, b in zip(times, times[1:])]
    return {"frames": len(points), "hz": (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else None,
            "interval_ms": stats(intervals)}


def regression(samples):
    t0 = mean(s["t_s"] for s in samples)
    p0 = mean(s["unwrapped_position_rad"] for s in samples)
    xx = sum((s["t_s"] - t0) ** 2 for s in samples)
    slope = sum((s["t_s"] - t0) * (s["unwrapped_position_rad"] - p0) for s in samples) / xx
    rms = math.sqrt(mean((s["unwrapped_position_rad"] - p0 - slope * (s["t_s"] - t0)) ** 2 for s in samples))
    return {"velocity_rad_s": slope, "position_fit_residual_rms_rad": rms}


def dictionary_updates(samples):
    """Deduplicate held values using inferred acquisition time (age rounded to ms).

    A >50ms advance marks a new read; a changed value also marks a read. The
    production dictionary polling interval is 500ms, well above this tolerance.
    Acquisition times are host receive estimates, not drive hardware timestamps.
    """
    updates = []
    for sample in samples:
        age = sample.get("dictionary_velocity_age_ms")
        value = sample.get("dictionary_velocity_rad_s")
        if age is None or value is None:
            continue
        acquired = sample["t_s"] - age / 1000
        if not updates or acquired - updates[-1]["t_s"] > 0.05 or value != updates[-1]["velocity_rad_s"]:
            updates.append({"t_s": acquired, "velocity_rad_s": value})
    return updates


def nearest_feedback(samples, instant):
    times = [s["t_s"] for s in samples]
    i = bisect.bisect_left(times, instant)
    candidates = samples[max(0, i - 1):min(len(samples), i + 1)]
    return min(candidates, key=lambda sample: abs(sample["t_s"] - instant))


def analyze(path):
    data = json.loads(path.read_text())
    samples = data["samples"]
    if len(samples) < 2:
        raise ValueError(f"{path}: fewer than two feedback samples")
    config = data["status"]["config"]
    duration = config["duration_s"]
    commands = data.get("commands", [])
    updates = dictionary_updates(samples)
    windows = []
    for start, end in WINDOWS:
        lo, hi = start * duration, end * duration
        selected = [s for s in samples if lo <= s["t_s"] < hi]
        if len(selected) < 2:
            continue
        fit = regression(selected)
        feedback = mean(s["velocity_rad_s"] for s in selected)
        command = mean(s["target"]["velocity_rad_s"] for s in selected if s.get("target"))
        readings = [u for u in updates if lo <= u["t_s"] < hi]
        aligned = [dict(u, nearest_feedback_velocity_rad_s=nearest_feedback(samples, u["t_s"])["velocity_rad_s"]) for u in readings]
        fresh_current = [s["current_a"] for s in selected
                         if s.get("current_a") is not None and s.get("current_age_ms") is not None and s["current_age_ms"] <= 100]
        windows.append({
            "window_s": [lo, hi], "frames": len(selected), "command_rad_s": command,
            "feedback_mean_rad_s": feedback, "feedback_by_message_type": {str(ty): {"frames": sum((s["can_id"] >> 24) & 31 == ty for s in selected), "mean_rad_s": mean(s["velocity_rad_s"] for s in selected if (s["can_id"] >> 24) & 31 == ty)} for ty in sorted({(s["can_id"] >> 24) & 31 for s in selected})}, "feedback_std_rad_s": statistics.pstdev(s["velocity_rad_s"] for s in selected),
            "position_regression": fit, "feedback_minus_position_rad_s": feedback - fit["velocity_rad_s"],
            "feedback_minus_position_percent": 100 * (feedback - fit["velocity_rad_s"]) / abs(fit["velocity_rad_s"]) if abs(fit["velocity_rad_s"]) > 0.05 else None,
            "command_minus_position_rad_s": command - fit["velocity_rad_s"] if command is not None else None,
            "mechvel_held_mean_rad_s": mean(s["dictionary_velocity_rad_s"] for s in selected if s.get("dictionary_velocity_rad_s") is not None),
            "mechvel_unique_updates": aligned, "mechvel_unique_mean_rad_s": mean(u["velocity_rad_s"] for u in readings),
            "mechvel_minus_nearest_feedback_mean_rad_s": mean(u["velocity_rad_s"] - u["nearest_feedback_velocity_rad_s"] for u in aligned),
            "drive_estimated_torque_mean_nm": mean(s["torque_nm"] for s in selected),
            "fresh_current_mean_a": mean(fresh_current), "fresh_current_held_samples": len(fresh_current),
            "mit_damping_term_mean_nm": mean(config["kd"] * (s["target"]["velocity_rad_s"] - s["velocity_rad_s"]) for s in selected if s.get("target")),
            "moving": abs(fit["velocity_rad_s"]) > 0.05,
        })
    position_values = [s["unwrapped_position_rad"] for s in samples]
    command_rate = rate(commands, "completed_t_s") if commands else None
    if command_rate:
        command_rate["submission_latency_ms"] = stats((c["completed_t_s"] - c["t_s"]) * 1000 for c in commands)
        command_rate["sequence_gaps"] = sum(max(0, b["sequence"] - a["sequence"] - 1) for a, b in zip(commands, commands[1:]))
    valid_updates = [u for u in updates if u["t_s"] >= 0]
    result = {
        "file": path.name, "motor_id": data.get("motor_id"), "uid": data.get("uid"), "config": config,
        "completion_reason": data["status"].get("reason"), "dropped_frames": data["status"].get("dropped_frames"),
        "feedback": dict(rate(samples), by_message_type={str(ty): rate([s for s in samples if (s["can_id"] >> 24) & 31 == ty]) for ty in sorted({(s["can_id"] >> 24) & 31 for s in samples})}, message_type_counts={str(ty): sum((s["can_id"] >> 24) & 31 == ty for s in samples) for ty in sorted({(s["can_id"] >> 24) & 31 for s in samples})}), "commands": command_rate,
        "feedback_sequence_gaps": sum(max(0, b["sequence"] - a["sequence"] - 1) for a, b in zip(samples, samples[1:])),
        "position_span_rad": max(position_values) - min(position_values),
        "max_fault_bits": max(s.get("fault_bits", 0) for s in samples),
        "temperature_c": stats(s["temperature_c"] for s in samples),
        "dictionary": dict(rate(valid_updates), age_ms=stats(s["dictionary_velocity_age_ms"] for s in samples if s.get("dictionary_velocity_age_ms") is not None)),
        "parameters": data.get("parameters", []), "version": data.get("version"), "windows": windows,
        "limitations": [
            "CAN motion feedback and position regression use the same encoder; this is internal consistency, not independent speed calibration.",
            "mechVel is sampled sparsely; repeated cached readings are not independent samples. Compare age-aligned updates, not held-value means.",
            "Timestamps and command completions are host observations, not drive execution timestamps or a bus analyzer capture.",
            "Torque is a drive estimate, not a force sensor. No-load effort is only apparent friction, not final static/dynamic calibration.",
        ],
    }
    return result, data, updates


class Svg:
    def __init__(self, width, height):
        self.parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                      '<style>text{font-family:Arial,sans-serif;fill:#263443} .small{font-size:13px}</style>',
                      f'<rect width="{width}" height="{height}" fill="#f7f9fc"/>']
    def add(self, item):
        self.parts.append(item)
    def text(self, x, y, label, size=15, color=None):
        style = f' style="fill:{color}"' if color else ""
        self.add(f'<text x="{x}" y="{y}" font-size="{size}"{style}>{escape(str(label))}</text>')
    def line(self, x0, y0, x1, y1, color="#dfe5ee", width=1, dash=None):
        dashed = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(f'<line x1="{x0:.2f}" y1="{y0:.2f}" x2="{x1:.2f}" y2="{y1:.2f}" stroke="{color}" stroke-width="{width}"{dashed}/>')
    def path(self, points, color, width=1.7, dash=None):
        if not points:
            return
        dashed = f' stroke-dasharray="{dash}"' if dash else ""
        coords = " ".join(f'{x:.2f},{y:.2f}' for x, y in points)
        self.add(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}"{dashed}/>')
    def save(self, path):
        path.write_text("\n".join(self.parts + ["</svg>"]))


def plot(trials, destination, effort=False):
    width, panel_height = 1440, 250
    height = 145 + len(trials) * panel_height + 45
    svg = Svg(width, height)
    svg.text(32, 36, "EL05 MIT no-load: " + ("drive-estimated effort" if effort else "command / CAN feedback / encoder-derived speed / mechVel"), 24)
    svg.text(32, 63, f'Motor ID {trials[0][0]["motor_id"]}, UID {trials[0][0]["uid"]} | host receive timestamps | shaded regions: steady comparison windows', 15)
    if effort:
        svg.text(32, 90, "Blue: drive torque estimate; gray dashed: Kd * (command - feedback). Neither is an independent torque measurement.", 14)
    else:
        for i, (key, label) in enumerate((("command", "Command"), ("feedback", "CAN feedback speed"), ("position", "Position-derived speed"), ("dictionary", "mechVel independent updates"))):
            x = 32 + i * 330
            svg.line(x, 90, x + 30, 90, COLORS[key], 3)
            svg.text(x + 40, 95, label, 14)
    for i, (result, data, updates) in enumerate(trials):
        top = 125 + i * panel_height
        left, right, ytop, bottom = 82, 1398, top + 38, top + 211
        samples, config = data["samples"], data["status"]["config"]
        duration = config["duration_s"]
        if effort:
            limit = max(0.05, max(abs(s["torque_nm"]) for s in samples) * 1.15, max(abs(config["kd"] * (s["target"]["velocity_rad_s"] - s["velocity_rad_s"])) for s in samples if s.get("target")) * 1.15)
        else:
            limit = max(0.15, max(abs(s["velocity_rad_s"]) for s in samples) * 1.10, config["velocity_rad_s"] * 1.12)
        x = lambda t: left + (right - left) * t / duration
        y = lambda v: bottom - (v + limit) / (2 * limit) * (bottom - ytop)
        svg.add(f'<rect x="{left}" y="{ytop}" width="{right-left}" height="{bottom-ytop}" fill="white"/>')
        for lo, hi in WINDOWS:
            svg.add(f'<rect x="{x(lo*duration):.2f}" y="{ytop}" width="{x(hi*duration)-x(lo*duration):.2f}" height="{bottom-ytop}" fill="#eef4df"/>')
        for j in range(5):
            value = -limit + j * limit / 2
            svg.line(left, y(value), right, y(value))
            svg.text(13, y(value) + 5, f"{value:.2f}", 13)
        for j in range(7):
            t = duration * j / 6
            svg.line(x(t), ytop, x(t), bottom)
            svg.text(x(t) - 6, bottom + 19, f"{t:g}", 13)
        svg.text(13, top + 25, "Nm" if effort else "rad/s", 13)
        svg.text(left, top + 23, f'{result["file"]} | target +/- {config["velocity_rad_s"]:g} rad/s, Kd={config["kd"]:g} | RX={result["feedback"]["hz"]:.3f} Hz', 16)
        if effort:
            svg.path([(x(s["t_s"]), y(s["torque_nm"])) for s in samples], COLORS["feedback"])
            svg.path([(x(s["t_s"]), y(config["kd"] * (s["target"]["velocity_rad_s"] - s["velocity_rad_s"]))) for s in samples if s.get("target")], COLORS["command"], 1.4, "5 3")
        else:
            svg.path([(x(c["t_s"]), y(c["target"]["velocity_rad_s"])) for c in data.get("commands", [])], COLORS["command"], 1.8, "6 3")
            svg.path([(x(s["t_s"]), y(s["position_velocity_rad_s"])) for s in samples if s.get("position_velocity_rad_s") is not None], COLORS["position"], 1.5)
            svg.path([(x(s["t_s"]), y(s["velocity_rad_s"])) for s in samples], COLORS["feedback"], 1.4)
            for update in updates:
                if 0 <= update["t_s"] <= duration:
                    svg.add(f'<circle cx="{x(update["t_s"]):.2f}" cy="{y(update["velocity_rad_s"]):.2f}" r="4" fill="{COLORS["dictionary"]}" stroke="white"/>')
        svg.text(right - 55, top + 23, "time (s)", 13)
    svg.text(32, height - 20, "Same-encoder consistency only. Sparse mechVel reads can alias ripple. Drive-estimated torque is not final friction calibration.", 14)
    svg.save(destination)
    return width, height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--png", action="store_true", help="also render PNG using a local Chromium/Chrome")
    args = parser.parse_args()
    paths = sorted(args.directory.glob("velocity-*.json"))
    if not paths:
        parser.error("no velocity-*.json files found")
    output = args.output or args.directory / "analysis"
    output.mkdir(parents=True, exist_ok=True)
    trials = [analyze(path) for path in paths]
    figures = {"speed-overlay.svg": plot(trials, output / "speed-overlay.svg"),
               "effort-overlay.svg": plot(trials, output / "effort-overlay.svg", effort=True)}
    if args.png:
        browser = next((shutil.which(name) for name in ("google-chrome", "chromium", "chromium-browser") if shutil.which(name)), None)
        if not browser:
            raise SystemExit("--png requires a locally installed Chrome/Chromium; SVG and JSON remain available")
        for name, (width, height) in figures.items():
            png = (output / name).with_suffix(".png").resolve()
            with tempfile.TemporaryDirectory(prefix="el05-analysis-chrome-") as profile:
                subprocess.run([browser, "--headless=new", "--no-sandbox", "--disable-gpu", "--disable-dev-shm-usage",
                                "--hide-scrollbars", "--no-first-run", f"--user-data-dir={profile}",
                                f"--window-size={width},{height}", f"--screenshot={png}", (output / name).resolve().as_uri()],
                               check=True, timeout=40, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    moving = [w for result, _, _ in trials for w in result["windows"] if w["moving"]]
    report = {"source_directory": str(args.directory.resolve()), "trials": [t[0] for t in trials],
              "summary": {"trials": len(trials), "moving_windows": len(moving),
                          "max_moving_feedback_vs_position_abs_rad_s": max(abs(w["feedback_minus_position_rad_s"]) for w in moving) if moving else None,
                          "max_moving_feedback_vs_position_abs_percent": max(abs(w["feedback_minus_position_percent"]) for w in moving) if moving else None,
                          "speed_mapping_consistent_against_encoder": len(moving) >= 4 and all(abs(w["feedback_minus_position_rad_s"]) <= 0.02 + 0.05 * abs(w["position_regression"]["velocity_rad_s"]) for w in moving),
                          "independent_metrology": False, "final_friction_calibration": False},
              "artifacts": [str(p.resolve()) for p in sorted(output.glob("*-overlay.*"))]}
    (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"report": str((output / "report.json").resolve()), **report["summary"], "artifacts": report["artifacts"]}, indent=2))


if __name__ == "__main__":
    main()
