#!/usr/bin/env python3
"""
Temporary waypoint visualization script.

Usage:
    python tools/visualize_waypoints.py [--file WAYPOINTS_FILE] [--out OUT.png]

This script parses the repository's `waypoints.txt` and plots each named path.
"""
from pathlib import Path
import re
import argparse
import matplotlib.pyplot as plt


def parse_waypoints_text(text):
    # Find sections like: name:\n\n[ ... ]
    pattern = re.compile(r"([A-Za-z0-9_]+):\s*\n\s*\[(.*?)\]", re.DOTALL)
    paths = {}
    for name, body in pattern.findall(text):
        # Split by semicolon and extract comma-separated floats
        coords = []
        for part in body.split(";"):
            part = part.strip()
            if not part:
                continue
            # find two floats inside the part
            nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", part)
            if len(nums) >= 2:
                x, y = float(nums[0]), float(nums[1])
                coords.append((x, y))
        if coords:
            paths[name] = coords
    return paths


def plot_paths(paths, out_path=None):
    plt.figure(figsize=(9, 7))
    ax = plt.gca()
    colors = plt.get_cmap("tab10")
    for i, (name, pts) in enumerate(paths.items()):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, '-o', color=colors(i), label=name)
        # mark start and end
        ax.scatter(xs[0], ys[0], c=colors(i), marker='s', s=60)
        ax.scatter(xs[-1], ys[-1], c=colors(i), marker='X', s=60)
    ax.set_aspect('equal', 'box')
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend()
    ax.set_title('Waypoints visualization')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=200)
        print(f"Saved plot to {out_path}")
    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'waypoints.txt',
                        help='Path to waypoints.txt')
    parser.add_argument('--out', '-o', type=Path, default=None,
                        help='Optional output PNG file')
    args = parser.parse_args()

    if not args.file.exists():
        print(f"Waypoints file not found: {args.file}")
        return
    text = args.file.read_text(encoding='utf-8')
    paths = parse_waypoints_text(text)
    if not paths:
        print("No paths parsed from file.")
        return
    plot_paths(paths, out_path=args.out)


if __name__ == '__main__':
    main()
