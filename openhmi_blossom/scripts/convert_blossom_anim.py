#!/usr/bin/env python3
"""
Convert original Blossom JSON animation files to the openhmi_blossom YAML format.

Original format:
  { "animation": "name",
    "frame_list": [
      { "millis": 0.0,
        "positions": [{"dof": "tower_1", "pos": <radians>}, ...] },
      ...
    ]
  }

Output YAML format (sequence_player keyframes):
  name:
    keyframes:
      - joints:
          motor_front: <raw 0-1023>
          ...
        duration: <seconds>

Usage:
    python3 convert_blossom_anim.py input.json [output.yaml]
    python3 convert_blossom_anim.py input.json            # prints to stdout
    python3 convert_blossom_anim.py *.json -o sequences/  # batch convert to dir
"""

import json
import sys
import os
import math
import yaml
import argparse


# XL-320 full range: 0–300 degrees = 0–1023 raw = 0–5.2360 radians
RAD_TO_RAW = 1023.0 / (300.0 * math.pi / 180.0)  # ≈ 195.3 raw/radian

# DOF name → our motor name
DOF_MAP = {
    'tower_1': 'motor_front',
    'tower_2': 'motor_back_left',
    'tower_3': 'motor_back_right',
    'base':    'lazy_susan',
}

# Minimum frame duration to avoid zero-time keyframes (seconds)
MIN_DURATION = 0.05


def rad_to_raw(radians: float) -> int:
    """Convert pypot radian position to XL-320 raw units (0–1023)."""
    return max(0, min(1023, round(float(radians) * RAD_TO_RAW)))


def convert(anim: dict) -> dict:
    """Convert a parsed Blossom animation dict to our keyframe format."""
    frames = anim.get('frame_list', [])
    if not frames:
        raise ValueError('No frames found in animation')

    keyframes = []
    prev_millis = None

    for frame in frames:
        millis = frame['millis']

        # Duration = time since previous frame (skip first frame's zero-time)
        if prev_millis is None:
            duration = 0.0
        else:
            duration = max(MIN_DURATION, (millis - prev_millis) / 1000.0)

        joints = {}
        for pos_entry in frame.get('positions', []):
            dof = pos_entry['dof']
            motor_name = DOF_MAP.get(dof)
            if motor_name is None:
                continue  # unknown DOF — skip
            joints[motor_name] = rad_to_raw(pos_entry['pos'])

        if joints:
            keyframes.append({'joints': joints, 'duration': round(duration, 4)})

        prev_millis = millis

    return {'keyframes': keyframes}


def main():
    parser = argparse.ArgumentParser(description='Convert Blossom JSON animations to YAML')
    parser.add_argument('inputs', nargs='+', help='Input JSON file(s)')
    parser.add_argument('-o', '--output', default=None,
                        help='Output file or directory (default: stdout for single file)')
    args = parser.parse_args()

    results = {}

    for input_path in args.inputs:
        with open(input_path, 'r') as f:
            anim = json.load(f)

        name = anim.get('animation') or os.path.splitext(os.path.basename(input_path))[0]
        sequence = convert(anim)
        results[name] = sequence

        print(f'Converted "{name}": {len(sequence["keyframes"])} keyframes', file=sys.stderr)

    # Single file → output as { name: {keyframes: [...]} }
    output_data = results if len(results) > 1 else {list(results.keys())[0]: list(results.values())[0]}

    yaml_str = yaml.dump(output_data, default_flow_style=False, sort_keys=False)

    if args.output is None:
        print(yaml_str)
    elif os.path.isdir(args.output) or args.output.endswith('/'):
        # Write one file per animation into the directory
        os.makedirs(args.output, exist_ok=True)
        for name, sequence in results.items():
            out_path = os.path.join(args.output, f'{name}.yaml')
            with open(out_path, 'w') as f:
                yaml.dump({name: sequence}, f, default_flow_style=False, sort_keys=False)
            print(f'Wrote {out_path}', file=sys.stderr)
    else:
        with open(args.output, 'w') as f:
            f.write(yaml_str)
        print(f'Wrote {args.output}', file=sys.stderr)


if __name__ == '__main__':
    main()
