import argparse
import os
import sys
# Allow `python -m src.run` from repo root
if __package__ is None or __package__ == "":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src import config as cfg
from src.pipeline import run_all, run_one
def main():
    parser = argparse.ArgumentParser(description="KITTI LiDAR + GPS + IMU EKF fusion")
    parser.add_argument(
        "--data-root",
        default=cfg.DATA_ROOT,
        help="Path to 2011_09_26 folder",
    )
    parser.add_argument(
        "--output-dir",
        default=cfg.OUTPUT_DIR,
        help="Directory for CSVs and plots",
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--drive",
        metavar="NAME",
        help=f"Single drive folder name (default example: {cfg.DEFAULT_DRIVE})",
    )
    group.add_argument(
        "--all",
        action="store_true",
        help="Process all drive_*_sync folders",
    )
    args = parser.parse_args()
    if args.all:
        run_all(data_root=args.data_root, out_dir=args.output_dir)
    else:
        run_one(args.drive, data_root=args.data_root, out_dir=args.output_dir)
if __name__ == "__main__":
    main()