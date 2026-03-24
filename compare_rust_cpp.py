#!/usr/bin/env python3
"""
Compare C++ and Rust GNSS IQ8 signal files.

Performs GPS L1CA acquisition (PRN 23, 27), nav bit extraction (PRN 23),
and Galileo E11 acquisition for both files, printing a comparison table.

Both files: signed int8 I/Q interleaved, 5 MHz sample rate, 1575.42 MHz center.
"""

import sys
import os
import struct
import numpy as np

# ============================================================
# Constants
# ============================================================
FS = 5e6           # sample rate (Hz)
F_IF = 0.0         # IF frequency (baseband — center freq matches L1)
CHIP_RATE = 1.023e6  # GPS L1CA chip rate
CODE_LEN = 1023    # GPS L1CA code length
CODE_PERIOD_MS = 1  # GPS L1CA code period in ms

# ============================================================
# GPS L1CA Gold code generator (matching Rust prngenerate.rs)
# ============================================================

# Per-SV G1 init states from prngenerate.rs (L1CA_PRN_INIT)
L1CA_PRN_INIT = [
    0x0df, 0x06f, 0x037, 0x01b, 0x1a4, 0x0d2, 0x1a6, 0x0d3,
    0x069, 0x0bb, 0x05d, 0x017, 0x00b, 0x005, 0x002, 0x001,
    0x191, 0x0c8, 0x064, 0x032, 0x019, 0x00c, 0x1cc, 0x039,
    0x01c, 0x00e, 0x007, 0x003, 0x1a8, 0x0d4, 0x06a, 0x035,
]

def lfsr_step(state, poly, length):
    """Single LFSR step: MSB output, shift left, feedback at LSB."""
    output_mask = 1 << (length - 1)
    output = 1 if (state & output_mask) else 0
    feedback = state & poly
    # Parity (count 1-bits)
    fb = bin(feedback).count('1') & 1
    state = ((state << 1) | fb) & ((1 << length) - 1)
    return state, output

def generate_gps_l1ca_prn(svid):
    """Generate GPS L1CA PRN code for given SVID (1-32). Returns numpy array of +1/-1."""
    g1_init = L1CA_PRN_INIT[svid - 1]
    g1_poly = 0x3a6
    g2_init = 0x3ff
    g2_poly = 0x204
    length = 10

    g1_state = g1_init
    g2_state = g2_init

    code = np.zeros(CODE_LEN, dtype=np.int8)
    for i in range(CODE_LEN):
        g1_state, g1_out = lfsr_step(g1_state, g1_poly, length)
        g2_state, g2_out = lfsr_step(g2_state, g2_poly, length)
        code[i] = g1_out ^ g2_out

    # Convert 0/1 to +1/-1 (0 -> +1, 1 -> -1)
    return 1 - 2 * code.astype(np.float32)


# ============================================================
# Galileo E1 memory code extractor
# ============================================================

def load_e1_memory_code_from_file():
    """Load Galileo E1 memory codes from src/memory_code_e1.rs"""
    path = os.path.join(os.path.dirname(__file__), "src", "memory_code_e1.rs")
    with open(path, 'r') as f:
        text = f.read()
    # Parse all hex values from the Rust array literal
    import re
    vals = re.findall(r'0x([0-9a-fA-F]+)', text)
    return [int(v, 16) for v in vals]

def extract_e1_memory_code(svid, is_pilot=False):
    """
    Extract Galileo E1 memory code for given SVID.
    Data channel: offset = (svid-1) * 128
    Pilot channel: offset = (svid+49) * 128
    Each sector = 1023 chips, 4 sectors = 4092 chips total.
    Returns numpy array of +1/-1.
    """
    all_words = load_e1_memory_code_from_file()

    if is_pilot:
        word_offset = (svid + 49) * 128
    else:
        word_offset = (svid - 1) * 128

    code = []
    for sector in range(4):
        for j in range(1023):
            word_idx = word_offset + sector * 32 + (j >> 5)
            bit_offset = 31 - (j & 0x1f)
            if word_idx < len(all_words):
                bit = (all_words[word_idx] >> bit_offset) & 1
            else:
                bit = 0
            code.append(bit)

    code = np.array(code, dtype=np.float32)
    return 1 - 2 * code  # 0->+1, 1->-1


# ============================================================
# IQ file reader
# ============================================================

def read_iq8(filepath, num_samples, offset_samples=0):
    """Read IQ8 file (signed int8 interleaved I,Q). Returns complex64 array."""
    with open(filepath, 'rb') as f:
        f.seek(offset_samples * 2)  # 2 bytes per sample (I + Q)
        raw = np.frombuffer(f.read(num_samples * 2), dtype=np.int8)
    if len(raw) < num_samples * 2:
        print(f"  WARNING: only got {len(raw)//2} samples (requested {num_samples})")
        num_samples = len(raw) // 2
    i_data = raw[0::2].astype(np.float32)
    q_data = raw[1::2].astype(np.float32)
    return i_data + 1j * q_data


# ============================================================
# Acquisition engine
# ============================================================

def acquire_prn(iq_data, prn_code, fs, code_len, doppler_range=5000, doppler_step=500,
                coherent_ms=1, num_coherent_blocks=20, code_period_samples=None):
    """
    Perform acquisition of a PRN code in IQ data.

    Returns: (best_code_phase, best_doppler, best_zscore, corr_map)
    """
    if code_period_samples is None:
        code_period_samples = int(fs * 1e-3)  # 1ms for GPS L1CA

    samples_per_block = code_period_samples * coherent_ms
    total_samples_needed = samples_per_block * num_coherent_blocks

    if len(iq_data) < total_samples_needed:
        num_coherent_blocks = len(iq_data) // samples_per_block
        if num_coherent_blocks < 1:
            return 0, 0, 0.0, None

    # Upsample PRN code to sample rate
    code_samples = int(fs * coherent_ms * 1e-3)
    t_idx = np.arange(code_samples)
    chip_idx = (t_idx * code_len / code_period_samples).astype(int) % code_len
    local_code = prn_code[chip_idx]

    # FFT of code (conjugate for correlation)
    code_fft = np.conj(np.fft.fft(local_code))

    doppler_bins = np.arange(-doppler_range, doppler_range + doppler_step, doppler_step)
    corr_map = np.zeros((len(doppler_bins), code_samples), dtype=np.float64)

    t = np.arange(code_samples) / fs

    for di, doppler in enumerate(doppler_bins):
        # Non-coherent accumulation across blocks
        for blk in range(num_coherent_blocks):
            start = blk * samples_per_block
            block = iq_data[start:start + code_samples]
            if len(block) < code_samples:
                break

            # Strip carrier (Doppler + IF)
            carrier = np.exp(-1j * 2 * np.pi * (F_IF + doppler) * t)
            mixed = block * carrier

            # Circular cross-correlation via FFT
            sig_fft = np.fft.fft(mixed)
            corr = np.fft.ifft(sig_fft * code_fft)
            corr_map[di] += np.abs(corr) ** 2

    # Find peak
    peak_idx = np.unravel_index(np.argmax(corr_map), corr_map.shape)
    best_doppler = doppler_bins[peak_idx[0]]
    best_code_phase = peak_idx[1]

    # Z-score: (peak - mean) / std of the correlation map
    peak_val = corr_map[peak_idx]
    # Exclude a window around the peak for noise estimation
    mask = np.ones_like(corr_map, dtype=bool)
    mask[peak_idx[0], max(0, peak_idx[1]-5):min(code_samples, peak_idx[1]+5)] = False
    noise = corr_map[mask]
    noise_mean = np.mean(noise)
    noise_std = np.std(noise)
    zscore = (peak_val - noise_mean) / noise_std if noise_std > 0 else 0.0

    return best_code_phase, best_doppler, zscore, corr_map


def acquire_galileo_e1(iq_data, svid, fs):
    """
    Acquire Galileo E1 signal using data channel memory code.
    E1 = 4092 chips, 4ms code period, BOC(1,1) -> 8184 subchips.
    For acquisition we search at chip level (no BOC subchip) over 4ms blocks.
    """
    code = extract_e1_memory_code(svid, is_pilot=False)
    code_len_e1 = 4092
    code_period_ms_e1 = 4
    code_period_samples = int(fs * code_period_ms_e1 * 1e-3)  # 20000 samples

    # Apply BOC(1,1) subchip modulation: double the code, alternate sign
    # BOC(1,1) for E1: each chip -> 2 subchips, second subchip flipped
    boc_code = np.zeros(code_len_e1 * 2, dtype=np.float32)
    for i in range(code_len_e1):
        boc_code[2*i] = code[i]
        boc_code[2*i+1] = -code[i]  # BOC(1,1) sign flip

    # Upsample BOC code to sample rate (8184 subchips in 4ms)
    t_idx = np.arange(code_period_samples)
    subchip_idx = (t_idx * (code_len_e1 * 2) / code_period_samples).astype(int) % (code_len_e1 * 2)
    local_code = boc_code[subchip_idx]

    code_fft = np.conj(np.fft.fft(local_code))

    doppler_range = 5000
    doppler_step = 500
    doppler_bins = np.arange(-doppler_range, doppler_range + doppler_step, doppler_step)

    num_blocks = 5  # 5 x 4ms = 20ms
    corr_map = np.zeros((len(doppler_bins), code_period_samples), dtype=np.float64)

    t = np.arange(code_period_samples) / fs

    for di, doppler in enumerate(doppler_bins):
        for blk in range(num_blocks):
            start = blk * code_period_samples
            block = iq_data[start:start + code_period_samples]
            if len(block) < code_period_samples:
                break
            carrier = np.exp(-1j * 2 * np.pi * (F_IF + doppler) * t)
            mixed = block * carrier
            sig_fft = np.fft.fft(mixed)
            corr = np.fft.ifft(sig_fft * code_fft)
            corr_map[di] += np.abs(corr) ** 2

    # Find peak
    peak_idx = np.unravel_index(np.argmax(corr_map), corr_map.shape)
    best_doppler = doppler_bins[peak_idx[0]]
    best_code_phase = peak_idx[1]

    peak_val = corr_map[peak_idx]
    mask = np.ones_like(corr_map, dtype=bool)
    mask[peak_idx[0], max(0, peak_idx[1]-10):min(code_period_samples, peak_idx[1]+10)] = False
    noise = corr_map[mask]
    noise_mean = np.mean(noise)
    noise_std = np.std(noise)
    zscore = (peak_val - noise_mean) / noise_std if noise_std > 0 else 0.0

    return best_code_phase, best_doppler, zscore


# ============================================================
# Nav bit extraction
# ============================================================

def extract_nav_bits(iq_data, prn_code, code_phase, doppler, fs, num_bits=30):
    """
    Extract nav bits by coherent 20ms integration.
    GPS L1CA: 50 bps -> 20ms per bit -> 20 x 1ms integrations.
    """
    samples_per_ms = int(fs * 1e-3)
    ms_per_bit = 20
    total_ms = num_bits * ms_per_bit

    # Build upsampled code for 1ms
    t_idx = np.arange(samples_per_ms)
    chip_idx = (t_idx * CODE_LEN / samples_per_ms).astype(int) % CODE_LEN
    local_code_1ms = prn_code[chip_idx]

    # Apply code phase shift (circular shift)
    local_code_1ms = np.roll(local_code_1ms, code_phase)

    t_1ms = np.arange(samples_per_ms) / fs
    carrier_1ms = np.exp(-1j * 2 * np.pi * (F_IF + doppler) * t_1ms)

    bits = []
    prompt_accum = []

    for bit_idx in range(num_bits):
        bit_sum = 0.0
        for ms in range(ms_per_bit):
            global_ms = bit_idx * ms_per_bit + ms
            start = global_ms * samples_per_ms
            block = iq_data[start:start + samples_per_ms]
            if len(block) < samples_per_ms:
                break

            # Carrier phase advances with time
            t_offset = global_ms * 1e-3
            phase_offset = np.exp(-1j * 2 * np.pi * (F_IF + doppler) * t_offset)
            mixed = block * carrier_1ms * phase_offset

            # Correlate
            corr = np.sum(mixed * local_code_1ms)
            bit_sum += np.real(corr)

        prompt_accum.append(bit_sum)
        bits.append(1 if bit_sum > 0 else 0)

    return bits, prompt_accum


# ============================================================
# Main comparison
# ============================================================

def main():
    cpp_file = sys.argv[1] if len(sys.argv) > 1 else "generated_files/moscow_cpp_300s.C8"
    rust_file = sys.argv[2] if len(sys.argv) > 2 else "generated_files/moscow_triple_l1_300s.C8"

    base_dir = os.path.dirname(os.path.abspath(__file__))
    cpp_path = os.path.join(base_dir, cpp_file)
    rust_path = os.path.join(base_dir, rust_file)

    for path, label in [(cpp_path, "C++"), (rust_path, "Rust")]:
        if not os.path.exists(path):
            print(f"ERROR: {label} file not found: {path}")
            sys.exit(1)
        size_mb = os.path.getsize(path) / 1e6
        print(f"{label}: {path} ({size_mb:.1f} MB)")

    # Read first 20ms for acquisition + 600ms for nav bits
    acq_ms = 20
    nav_ms = 600
    total_ms = nav_ms  # nav_ms includes acq_ms worth of data
    total_samples = int(total_ms * FS / 1000)

    print(f"\nReading {total_ms}ms ({total_samples} samples) from each file...")
    iq_cpp = read_iq8(cpp_path, total_samples)
    iq_rust = read_iq8(rust_path, total_samples)

    print(f"  C++ : {len(iq_cpp)} samples, RMS={np.std(np.abs(iq_cpp)):.3f}")
    print(f"  Rust: {len(iq_rust)} samples, RMS={np.std(np.abs(iq_rust)):.3f}")

    # ================================================================
    # 1. GPS PRN 23 acquisition
    # ================================================================
    print("\n" + "="*70)
    print("1. GPS PRN 23 ACQUISITION (first 20ms)")
    print("="*70)

    prn23 = generate_gps_l1ca_prn(23)

    print("\n  Searching C++...")
    cp_cpp_23, dop_cpp_23, z_cpp_23, _ = acquire_prn(iq_cpp, prn23, FS, CODE_LEN)
    print(f"  C++  PRN23: code_phase={cp_cpp_23}, Doppler={dop_cpp_23} Hz, z-score={z_cpp_23:.1f}")

    print("  Searching Rust...")
    cp_rust_23, dop_rust_23, z_rust_23, _ = acquire_prn(iq_rust, prn23, FS, CODE_LEN)
    print(f"  Rust PRN23: code_phase={cp_rust_23}, Doppler={dop_rust_23} Hz, z-score={z_rust_23:.1f}")

    # ================================================================
    # 2. GPS PRN 27 acquisition
    # ================================================================
    print("\n" + "="*70)
    print("2. GPS PRN 27 ACQUISITION (first 20ms)")
    print("="*70)

    prn27 = generate_gps_l1ca_prn(27)

    print("\n  Searching C++...")
    cp_cpp_27, dop_cpp_27, z_cpp_27, _ = acquire_prn(iq_cpp, prn27, FS, CODE_LEN)
    print(f"  C++  PRN27: code_phase={cp_cpp_27}, Doppler={dop_cpp_27} Hz, z-score={z_cpp_27:.1f}")

    print("  Searching Rust...")
    cp_rust_27, dop_rust_27, z_rust_27, _ = acquire_prn(iq_rust, prn27, FS, CODE_LEN)
    print(f"  Rust PRN27: code_phase={cp_rust_27}, Doppler={dop_rust_27} Hz, z-score={z_rust_27:.1f}")

    # ================================================================
    # 3. Nav bit extraction for GPS PRN 23
    # ================================================================
    print("\n" + "="*70)
    print("3. NAV BIT EXTRACTION — GPS PRN 23 (600ms = 30 bits)")
    print("="*70)

    print("\n  Extracting C++ nav bits...")
    bits_cpp, prompt_cpp = extract_nav_bits(iq_cpp, prn23, cp_cpp_23, dop_cpp_23, FS, num_bits=30)
    print(f"  C++  bits: {''.join(map(str, bits_cpp))}")
    print(f"  C++  prompt magnitudes (first 10): {['%.1f' % abs(p) for p in prompt_cpp[:10]]}")

    print("\n  Extracting Rust nav bits...")
    bits_rust, prompt_rust = extract_nav_bits(iq_rust, prn23, cp_rust_23, dop_rust_23, FS, num_bits=30)
    print(f"  Rust bits: {''.join(map(str, bits_rust))}")
    print(f"  Rust prompt magnitudes (first 10): {['%.1f' % abs(p) for p in prompt_rust[:10]]}")

    # Compare
    matching = sum(1 for a, b in zip(bits_cpp, bits_rust) if a == b)
    print(f"\n  Nav bit match: {matching}/30 ({100*matching/30:.0f}%)")
    if bits_cpp == bits_rust:
        print("  RESULT: IDENTICAL nav bits")
    else:
        diff_positions = [i for i, (a, b) in enumerate(zip(bits_cpp, bits_rust)) if a != b]
        print(f"  RESULT: DIFFERENT at bit positions: {diff_positions}")

    # ================================================================
    # 4. Galileo E11 acquisition
    # ================================================================
    print("\n" + "="*70)
    print("4. GALILEO E11 ACQUISITION (E1 signal, SVID 11, first 20ms)")
    print("="*70)

    print("\n  Searching C++...")
    cp_cpp_e11, dop_cpp_e11, z_cpp_e11 = acquire_galileo_e1(iq_cpp[:int(20*FS/1000)], 11, FS)
    print(f"  C++  E11: code_phase={cp_cpp_e11}, Doppler={dop_cpp_e11} Hz, z-score={z_cpp_e11:.1f}")

    print("  Searching Rust...")
    cp_rust_e11, dop_rust_e11, z_rust_e11 = acquire_galileo_e1(iq_rust[:int(20*FS/1000)], 11, FS)
    print(f"  Rust E11: code_phase={cp_rust_e11}, Doppler={dop_rust_e11} Hz, z-score={z_rust_e11:.1f}")

    # ================================================================
    # Summary table
    # ================================================================
    print("\n" + "="*70)
    print("SUMMARY COMPARISON TABLE")
    print("="*70)
    print(f"{'Signal':<15} {'Metric':<15} {'C++':<20} {'Rust':<20} {'Match':>8}")
    print("-" * 78)

    # PRN 23
    cp_match_23 = "YES" if cp_cpp_23 == cp_rust_23 else "NO"
    dop_match_23 = "YES" if dop_cpp_23 == dop_rust_23 else f"d={abs(dop_cpp_23-dop_rust_23)}"
    print(f"{'GPS PRN23':<15} {'Code Phase':<15} {cp_cpp_23:<20} {cp_rust_23:<20} {cp_match_23:>8}")
    print(f"{'':<15} {'Doppler (Hz)':<15} {dop_cpp_23:<20} {dop_rust_23:<20} {dop_match_23:>8}")
    print(f"{'':<15} {'Z-score':<15} {z_cpp_23:<20.1f} {z_rust_23:<20.1f} {'':>8}")
    detected_cpp_23 = "DETECTED" if z_cpp_23 > 15 else "NOT FOUND"
    detected_rust_23 = "DETECTED" if z_rust_23 > 15 else "NOT FOUND"
    print(f"{'':<15} {'Status':<15} {detected_cpp_23:<20} {detected_rust_23:<20} {'':>8}")
    print("-" * 78)

    # PRN 27
    cp_match_27 = "YES" if cp_cpp_27 == cp_rust_27 else "NO"
    dop_match_27 = "YES" if dop_cpp_27 == dop_rust_27 else f"d={abs(dop_cpp_27-dop_rust_27)}"
    print(f"{'GPS PRN27':<15} {'Code Phase':<15} {cp_cpp_27:<20} {cp_rust_27:<20} {cp_match_27:>8}")
    print(f"{'':<15} {'Doppler (Hz)':<15} {dop_cpp_27:<20} {dop_rust_27:<20} {dop_match_27:>8}")
    print(f"{'':<15} {'Z-score':<15} {z_cpp_27:<20.1f} {z_rust_27:<20.1f} {'':>8}")
    detected_cpp_27 = "DETECTED" if z_cpp_27 > 15 else "NOT FOUND"
    detected_rust_27 = "DETECTED" if z_rust_27 > 15 else "NOT FOUND"
    print(f"{'':<15} {'Status':<15} {detected_cpp_27:<20} {detected_rust_27:<20} {'':>8}")
    print("-" * 78)

    # Nav bits
    print(f"{'GPS PRN23 Nav':<15} {'30 bits':<15} {''.join(map(str, bits_cpp[:15]))+'...':<20} {''.join(map(str, bits_rust[:15]))+'...':<20} {f'{matching}/30':>8}")
    avg_prompt_cpp = np.mean(np.abs(prompt_cpp))
    avg_prompt_rust = np.mean(np.abs(prompt_rust))
    print(f"{'':<15} {'Avg |prompt|':<15} {avg_prompt_cpp:<20.1f} {avg_prompt_rust:<20.1f} {'':>8}")
    print("-" * 78)

    # Galileo E11
    cp_match_e11 = "YES" if cp_cpp_e11 == cp_rust_e11 else "NO"
    dop_match_e11 = "YES" if dop_cpp_e11 == dop_rust_e11 else f"d={abs(dop_cpp_e11-dop_rust_e11)}"
    print(f"{'GAL E11':<15} {'Code Phase':<15} {cp_cpp_e11:<20} {cp_rust_e11:<20} {cp_match_e11:>8}")
    print(f"{'':<15} {'Doppler (Hz)':<15} {dop_cpp_e11:<20} {dop_rust_e11:<20} {dop_match_e11:>8}")
    print(f"{'':<15} {'Z-score':<15} {z_cpp_e11:<20.1f} {z_rust_e11:<20.1f} {'':>8}")
    detected_cpp_e11 = "DETECTED" if z_cpp_e11 > 10 else "NOT FOUND"
    detected_rust_e11 = "DETECTED" if z_rust_e11 > 10 else "NOT FOUND"
    print(f"{'':<15} {'Status':<15} {detected_cpp_e11:<20} {detected_rust_e11:<20} {'':>8}")
    print("=" * 78)

    # Additional multi-PRN scan for context
    print("\n" + "="*70)
    print("BONUS: Quick scan of ALL GPS PRNs (1-32) in both files")
    print("="*70)
    print(f"{'PRN':<6} {'C++ z-score':<15} {'Rust z-score':<15} {'C++ Doppler':<14} {'Rust Doppler':<14} {'Status'}")
    print("-" * 78)

    for svid in range(1, 33):
        code = generate_gps_l1ca_prn(svid)
        _, dop_c, z_c, _ = acquire_prn(iq_cpp, code, FS, CODE_LEN, num_coherent_blocks=20)
        _, dop_r, z_r, _ = acquire_prn(iq_rust, code, FS, CODE_LEN, num_coherent_blocks=20)

        if z_c > 15 or z_r > 15:
            status = ""
            if z_c > 15 and z_r > 15:
                status = "BOTH"
            elif z_c > 15:
                status = "C++ ONLY"
            else:
                status = "RUST ONLY"
            print(f"G{svid:<5} {z_c:<15.1f} {z_r:<15.1f} {dop_c:<14} {dop_r:<14} {status}")

    print("=" * 78)
    print("Done.")


if __name__ == "__main__":
    main()
