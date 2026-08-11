"""Print the first samples from the available LSL EEG stream."""

from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="")
    parser.add_argument("--count", type=int, default=10)
    args = parser.parse_args()

    try:
        from pylsl import StreamInlet, resolve_byprop
    except ImportError as exc:
        raise SystemExit("Install pylsl before monitoring LSL") from exc

    if args.name:
        streams = resolve_byprop("name", args.name, minimum=1, timeout=10)
    else:
        streams = resolve_byprop("type", "EEG", minimum=1, timeout=10)
    if not streams:
        raise SystemExit("No EEG LSL stream found")
    info = streams[0]
    print(
        f"name={info.name()} type={info.type()} channels={info.channel_count()} "
        f"nominal_rate={info.nominal_srate()}"
    )
    inlet = StreamInlet(info)
    for index in range(args.count):
        sample, timestamp = inlet.pull_sample(timeout=5.0)
        if sample is None:
            print("timeout waiting for sample")
            continue
        print(f"{index}: t={timestamp:.6f} values={sample}")


if __name__ == "__main__":
    main()
