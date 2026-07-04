import numpy as np
from scipy.signal import firwin
from pathlib import Path
from datetime import datetime


def design_filter(num_taps, cutoff_hz, sample_rate_hz):
    nyquist = sample_rate_hz / 2
    return firwin(num_taps, cutoff_hz / nyquist, window="hamming")
    # return firwin(
    #     num_taps,
    #     [100_000 / nyquist, 400_000 / nyquist],
    #     pass_zero=False,
    #     window="hamming",
    # )


def scale_to_i16(taps):
    max_val = np.max(np.abs(taps))
    scaled = taps / max_val * 32767
    return np.round(scaled).astype(np.int16)


def generate_ad9361_ftr(
    sample_rate_hz: float,
    bw_rx_hz: float,
    bw_tx_hz: float,
    num_taps: int = 128,
    output_file: str = "ad9361_custom.ftr",
):
    rx_taps = design_filter(num_taps, bw_rx_hz / 2, sample_rate_hz)
    tx_taps = design_filter(num_taps, bw_tx_hz / 2, sample_rate_hz)

    rx_scaled = scale_to_i16(rx_taps)
    tx_scaled = scale_to_i16(tx_taps)

    # Generate interleaved pairs
    pairs = [f"{rx},{tx}" for rx, tx in zip(rx_scaled, tx_scaled)]

    # Generate header
    header = [
        "TX 3 GAIN 0 INT 4",
        "RX 3 GAIN -12 DEC 4",
        "RTX 768000000 192000000 64000000 32000000 16000000 4000000",
        "RRX 768000000 192000000 64000000 32000000 16000000 4000000",
        f"BWTX {int(bw_tx_hz)}",
        f"BWRX {int(bw_rx_hz)}",
    ]

    # Write to file
    Path(output_file).write_text("\n".join(header + pairs) + "\n")
    print(f"Filter file written to: {output_file}")


# Example usage:
if __name__ == "__main__":
    generate_ad9361_ftr(
        sample_rate_hz=5_000_000,
        bw_rx_hz=1_000_000,
        bw_tx_hz=1_000_000,
        output_file="ad9361_custom.ftr",
    )
