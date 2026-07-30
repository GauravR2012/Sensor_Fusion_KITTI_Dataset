import argparse
from . import config
from .pipeline import run_pipeline

def main():
    p = argparse.ArgumentParser(description="Track B — KITTI Odometry stereo VO")
    p.add_argument("--seq", default=config.SEQUENCE)
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--data-root", default=None)
    p.add_argument("--no-plots", action="store_true")
    args = p.parse_args()
    if args.data_root:
        config.DATA_ROOT = args.data_root
    if args.no_plots:
        config.SHOW_PLOTS = False
    run_pipeline(seq=args.seq, max_frames=args.max_frames)

if __name__ == "__main__":
    main()
