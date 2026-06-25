#!/usr/bin/env python3
"""
Generate the ChArUco calibration board and save it to this folder.

Board dimensions match config/calibration.yaml:
  5 × 7 squares,  10 cm / square,  7.5 cm ArUco markers,  DICT_4X4_50

Print at A1 (594 × 841 mm) so each square is physically 10 cm.
After printing, measure the actual square size and update calibration.yaml.

Usage:
    python3 boards/generate_charuco_board.py
    python3 boards/generate_charuco_board.py --squares-x 5 --squares-y 7 --dpi 300
"""

import argparse
import sys
from pathlib import Path

try:
    import cv2
    import cv2.aruco as aruco
except ImportError:
    print('[ERROR] OpenCV not found. Install with: pip3 install opencv-python', file=sys.stderr)
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description='Generate ChArUco calibration board')
    parser.add_argument('--squares-x',    type=int,   default=5,           help='Columns of squares (default: 5)')
    parser.add_argument('--squares-y',    type=int,   default=7,           help='Rows of squares (default: 7)')
    parser.add_argument('--square-size',  type=float, default=0.10,        help='Square side in metres (default: 0.10)')
    parser.add_argument('--marker-size',  type=float, default=0.075,       help='ArUco marker side in metres (default: 0.075)')
    parser.add_argument('--dpi',          type=int,   default=300,         help='Print resolution in DPI (default: 300)')
    parser.add_argument('--dict',         type=str,   default='DICT_4X4_50', help='ArUco dictionary (default: DICT_4X4_50)')
    parser.add_argument('--output',       type=str,   default='',          help='Output filename (default: auto)')
    args = parser.parse_args()

    # Resolve ArUco dictionary
    dict_id = getattr(aruco, args.dict, None)
    if dict_id is None:
        print(f'[ERROR] Unknown ArUco dictionary: {args.dict}', file=sys.stderr)
        sys.exit(1)
    dictionary = aruco.Dictionary_get(dict_id)

    # Create board
    board = aruco.CharucoBoard_create(
        squaresX=args.squares_x,
        squaresY=args.squares_y,
        squareLength=args.square_size,
        markerLength=args.marker_size,
        dictionary=dictionary,
    )

    # Calculate pixel dimensions for the given DPI
    # Board physical size: squares_x * square_size  ×  squares_y * square_size (metres)
    metres_per_inch = 0.0254
    board_w_m = args.squares_x * args.square_size
    board_h_m = args.squares_y * args.square_size
    px_w = int(round(board_w_m / metres_per_inch * args.dpi))
    px_h = int(round(board_h_m / metres_per_inch * args.dpi))

    # Draw board
    img = board.draw((px_w, px_h), marginSize=0, borderBits=1)

    # Output path
    script_dir = Path(__file__).parent
    if args.output:
        out_path = Path(args.output)
    else:
        out_path = script_dir / (
            f'charuco_{args.squares_x}x{args.squares_y}'
            f'_sq{int(args.square_size * 100)}cm'
            f'_{args.dict}.png'
        )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)

    print(f'Board saved: {out_path}')
    print(f'  Squares:      {args.squares_x} × {args.squares_y}')
    print(f'  Square size:  {args.square_size * 100:.1f} cm  (measure after printing)')
    print(f'  Marker size:  {args.marker_size * 100:.1f} cm')
    print(f'  Dictionary:   {args.dict}')
    print(f'  Image size:   {px_w} × {px_h} px  @ {args.dpi} DPI')
    print(f'  Print size:   {board_w_m * 100:.0f} × {board_h_m * 100:.0f} cm  (A1 = 59.4 × 84.1 cm)')
    print()
    print('Next steps:')
    print('  1. Print at actual size (no scaling) on A1 or larger paper')
    print('  2. Glue to a rigid flat board (foam core, aluminium sheet)')
    print('  3. Measure the actual printed square size with a ruler')
    print('  4. Update config/calibration.yaml  →  board.square_size_m and board.marker_size_m')


if __name__ == '__main__':
    main()
