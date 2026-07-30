import argparse
from . import config
from .pipeline import run_aided, run_odom

def main():
    p = argparse.ArgumentParser(description="Track C — raw KITTI 0047")
    p.add_argument("--mode", choices=["aided", "odom"], required=True)
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--data-path", default=None)
    p.add_argument("--calib-root", default=None)
    p.add_argument("--no-plots", action="store_true")
    args = p.parse_args()
    if args.data_path: config.DATA_PATH = args.data_path
    if args.calib_root: config.CALIB_ROOT = args.calib_root
    if args.no_plots: config.SHOW_PLOTS = False
    run_aided(max_frames=args.max_frames) if args.mode == "aided" else run_odom(max_frames=args.max_frames)

if __name__ == "__main__":
    main()
