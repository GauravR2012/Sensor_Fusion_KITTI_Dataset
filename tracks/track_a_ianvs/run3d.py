import argparse
import os
import sys

# Allow: python -m src.run3d
if __package__ is None or __package__ == "":
    sys.path.insert(
        0,
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    )

from src import config as cfg
from src.pipeline3d import run_one_3d, run_all_3d


def main():
    parser = argparse.ArgumentParser(
        description="KITTI 3D LiDAR + GPS + IMU Sensor Fusion"
    )

    parser.add_argument(
        "--data-root",
        default=cfg.DATA_ROOT,
        help="Path to KITTI 2011_09_26 directory",
    )

    parser.add_argument(
        "--output-dir",
        default=cfg.OUTPUT_DIR,
        help="Directory for output CSVs and figures",
    )

    group = parser.add_mutually_exclusive_group(required=True)

    group.add_argument(
        "--drive",
        metavar="NAME",
        help=f"Drive folder (example: {cfg.DEFAULT_DRIVE})",
    )

    group.add_argument(
        "--all",
        action="store_true",
        help="Process every drive in DATA_ROOT",
    )

    args = parser.parse_args()

    if args.all:
        run_all_3d(
            data_root=args.data_root,
            out_dir=args.output_dir,
        )
    else:
        run_one_3d(
            args.drive,
            data_root=args.data_root,
            out_dir=args.output_dir,
        )


if __name__ == "__main__":
    main()