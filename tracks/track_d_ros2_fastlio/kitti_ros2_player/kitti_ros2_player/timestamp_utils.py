"""
Helpers for parsing KITTI Raw `timestamps.txt` files.

KITTI timestamps look like:
    2011-10-03 13:07:43.019283000

Python's datetime %f only supports microsecond (6-digit) precision, and
KITTI gives nanosecond (9-digit) precision, so we parse these manually
instead of using strptime.
"""

from typing import List, Tuple


def _parse_line(line: str) -> Tuple[float, Tuple[int, int, int]]:
    """Parse one timestamp line into (seconds_within_day, (year, month, day))."""
    date_part, time_part = line.strip().split(' ')
    year, month, day = (int(x) for x in date_part.split('-'))
    hh, mm, ss = time_part.split(':')
    seconds_within_day = int(hh) * 3600 + int(mm) * 60 + float(ss)
    return seconds_within_day, (year, month, day)


def load_timestamps(path: str) -> List[float]:
    """
    Load a KITTI `timestamps.txt` file and return a list of floats:
    seconds elapsed since the *first* timestamp in the file.

    Handles the (rare) case where the drive crosses midnight by detecting
    a decrease in seconds-within-day and adding a day offset.
    """
    with open(path, 'r') as f:
        lines = [line for line in f.readlines() if line.strip()]

    if not lines:
        return []

    seconds_within_day = []
    for line in lines:
        sec, _ = _parse_line(line)
        seconds_within_day.append(sec)

    abs_seconds = []
    day_offset = 0.0
    for i, sec in enumerate(seconds_within_day):
        if i > 0 and sec < seconds_within_day[i - 1] - 1.0:
            # Wrapped past midnight.
            day_offset += 86400.0
        abs_seconds.append(sec + day_offset)

    t0 = abs_seconds[0]
    return [t - t0 for t in abs_seconds]
