#!/usr/bin/env python3
"""
Enhanced GNSS Signal Verification Tool with Diagnostic Charts.

Generates a 3-page PDF report:
  Page 1: Signal overview (spectrum, I/Q histogram, constellation, RMS stability)
  Page 2: Acquisition results (z-score bars, skyplot, CN0, Doppler comparison)
  Page 3: Correlation details (zoomed peaks, 2D heatmaps)

Usage:
  python verify_signal_enhanced.py <iq_file> [--preset presets/GPS_BDS_GAL_triple_system.json]
  python verify_signal_enhanced.py generated_files/GPS_L1_only_10s.C8 --sample-rate 5.0
"""

import numpy as np
from scipy.fft import fft, ifft
from scipy.signal import welch as scipy_welch
import sys
import os
import re
import json
import argparse
import math
from datetime import datetime, timedelta

# ============================================================
# Constants
# ============================================================

# WGS84 ellipsoid
WGS_A = 6378137.0
WGS_E2 = 0.00669437999014
WGS_EP2 = 0.00673949674228
WGS_B = 6356752.314245179

# Earth parameters
EARTH_GM = 3.986004418e14
OMEGA_E = 7.2921151467e-5
SPEED_OF_LIGHT = 299792458.0

# GPS time
GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0)
GPS_LEAP_SECONDS = 18
BDT_WEEK_OFFSET = 1356

# L1 carrier
F_L1 = 1575.42e6

# Signal defaults
DEFAULT_SAMPLE_RATE = 5.0e6
IF_FREQ = 0.0
GPS_CODE_LENGTH = 1023
GPS_CODE_PERIOD_MS = 1
BDS_CODE_LENGTH = 10230
BDS_CODE_PERIOD_MS = 10
GAL_CODE_LENGTH = 4092
GAL_CODE_PERIOD_MS = 4

# Acquisition defaults
DOPPLER_RANGE = 7000
DOPPLER_STEP = 250
ZSCORE_THRESHOLD = 30
NON_COHERENT_GPS = 10
NON_COHERENT_BDS = 10
NON_COHERENT_GAL = 20
NON_COHERENT_GLO = 10

# GLONASS FDMA
GLO_CODE_LENGTH = 511
GLO_CODE_PERIOD_MS = 1
GLO_F_BASE = 1602.0e6       # G1 base frequency (Hz)
GLO_F_STEP = 0.5625e6       # FDMA channel step (Hz)

# GPS L5
F_L5 = 1176.45e6
GPS_L5_CODE_LENGTH = 10230
GPS_L5_CODE_PERIOD_MS = 1      # 10.23 Mchip/s -> 10230/10.23e6 = 1 ms
NON_COHERENT_L5 = 10

# GPS L2C
F_L2 = 1227.60e6
GPS_L2C_CODE_LENGTH = 10230
GPS_L2C_CODE_PERIOD_MS = 10    # 1.023 Mchip/s -> 10230/1.023e6 = 10 ms
NON_COHERENT_L2C = 5

# GPS L1C
GPS_L1C_CODE_LENGTH = 10230
GPS_L1C_CODE_PERIOD_MS = 10    # 1.023 Mchip/s -> 10230/1.023e6 = 10 ms
NON_COHERENT_L1C = 5

PI2 = 2.0 * math.pi


# ============================================================
# GPS C/A Code Generation (Gold Codes)
# ============================================================

GPS_CA_TAPS = {
    1: (2, 6), 2: (3, 7), 3: (4, 8), 4: (5, 9), 5: (1, 9),
    6: (2, 10), 7: (1, 8), 8: (2, 9), 9: (3, 10), 10: (2, 3),
    11: (3, 4), 12: (5, 6), 13: (6, 7), 14: (7, 8), 15: (8, 9),
    16: (9, 10), 17: (1, 4), 18: (2, 5), 19: (3, 6), 20: (4, 7),
    21: (5, 8), 22: (6, 9), 23: (1, 3), 24: (4, 6), 25: (5, 7),
    26: (6, 8), 27: (7, 9), 28: (8, 10), 29: (1, 6), 30: (2, 7),
    31: (3, 8), 32: (4, 9),
}


def generate_ca_code(prn):
    """Generate GPS C/A code for given PRN (1-32). Returns +/-1 array."""
    if prn not in GPS_CA_TAPS:
        return None
    g1 = np.ones(10, dtype=int)
    g2 = np.ones(10, dtype=int)
    tap1, tap2 = GPS_CA_TAPS[prn]
    code = np.zeros(GPS_CODE_LENGTH, dtype=np.int8)
    for i in range(GPS_CODE_LENGTH):
        g1_out = g1[9]
        g2_out = g2[tap1 - 1] ^ g2[tap2 - 1]
        code[i] = g1_out ^ g2_out
        g1_fb = g1[2] ^ g1[9]
        g1 = np.roll(g1, 1)
        g1[0] = g1_fb
        g2_fb = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]
        g2 = np.roll(g2, 1)
        g2[0] = g2_fb
    return 1.0 - 2.0 * code.astype(np.float64)


# ============================================================
# BeiDou B1C Code Generation (Weil/Legendre + BOC)
# ============================================================

B1C_DATA_TRUNCATION = [
    699, 694, 7318, 2127, 715, 6682, 7850, 5495, 1162, 7682, 6792, 9973, 6596, 2092, 19, 10151,
    6297, 5766, 2359, 7136, 1706, 2128, 6827, 693, 9729, 1620, 6805, 534, 712, 1929, 5355, 6139,
    6339, 1470, 6867, 7851, 1162, 7659, 1156, 2672, 6043, 2862, 180, 2663, 6940, 1645, 1582, 951,
    6878, 7701, 1823, 2391, 2606, 822, 6403, 239, 442, 6769, 2560, 2502, 5072, 7268, 341,
]

B1C_DATA_PHASE_DIFF = [
    2678, 4802, 958, 859, 3843, 2232, 124, 4352, 1816, 1126, 1860, 4800, 2267, 424, 4192, 4333,
    2656, 4148, 243, 1330, 1593, 1470, 882, 3202, 5095, 2546, 1733, 4795, 4577, 1627, 3638, 2553,
    3646, 1087, 1843, 216, 2245, 726, 1966, 670, 4130, 53, 4830, 182, 2181, 2006, 1080, 2288, 2027,
    271, 915, 497, 139, 3693, 2054, 4342, 3342, 2592, 1007, 310, 4203, 455, 4318,
]


def generate_legendre_sequence(length):
    seq = np.zeros(length, dtype=np.int32)
    for i in range(1, length):
        seq[(i * i) % length] = 1
    return seq


def generate_b1c_weil(svid):
    if svid < 1 or svid > 63:
        return None
    trunc = B1C_DATA_TRUNCATION[svid - 1]
    phase = B1C_DATA_PHASE_DIFF[svid - 1]
    legendre = generate_legendre_sequence(10243)
    code = np.zeros(10230, dtype=np.int32)
    idx1 = trunc - 1
    idx2 = trunc + phase - 1
    for i in range(10230):
        if idx1 >= 10243:
            idx1 -= 10243
        if idx2 >= 10243:
            idx2 -= 10243
        code[i] = legendre[idx1] ^ legendre[idx2]
        idx1 += 1
        idx2 += 1
    return code


def apply_boc11(code_01):
    """Apply BOC(1,1): each chip -> [+chip, -chip]. Input 0/1, output +/-1."""
    code_pm = np.where(code_01 == 0, 1.0, -1.0)
    boc = np.zeros(len(code_pm) * 2, dtype=np.float64)
    boc[0::2] = code_pm
    boc[1::2] = -code_pm
    return boc


def generate_b1c_boc(svid):
    code = generate_b1c_weil(svid)
    if code is None:
        return None
    return apply_boc11(code)


# ============================================================
# Galileo E1 Code Generation (Memory Codes + BOC)
# ============================================================

_e1_memory_code = None


def load_e1_memory_codes():
    global _e1_memory_code
    if _e1_memory_code is not None:
        return _e1_memory_code
    filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'memory_code_e1.rs')
    if not os.path.exists(filepath):
        print(f"  E1 memory code file not found: {filepath}")
        return None
    with open(filepath, 'r') as f:
        content = f.read()
    hex_values = re.findall(r'0x([0-9a-fA-F]+)', content)
    _e1_memory_code = np.array([int(h, 16) for h in hex_values], dtype=np.uint32)
    return _e1_memory_code


def extract_e1_code(svid, use_pilot=False):
    mem = load_e1_memory_codes()
    if mem is None:
        return None
    offset = (svid + 49) * 128 if use_pilot else (svid - 1) * 128
    if offset + 128 > len(mem):
        return None
    binary_seq = mem[offset:offset + 128]
    code = np.zeros(4092, dtype=np.int32)
    for sector in range(4):
        for j in range(1023):
            bit_index = j >> 5
            bit_offset = 31 - (j & 0x1f)
            word_index = sector * 32 + bit_index
            if word_index < len(binary_seq):
                code[sector * 1023 + j] = 1 if (int(binary_seq[word_index]) & (1 << bit_offset)) != 0 else 0
    return code


def generate_e1_boc(svid):
    code = extract_e1_code(svid, use_pilot=False)
    if code is None:
        return None
    return apply_boc11(code)


# ============================================================
# GLONASS G1 Code Generation (FDMA — single code for all SVs)
# ============================================================

_GLO_G1_CODE = None

def generate_glonass_g1_code(svid=None):
    """Generate GLONASS G1 ranging code (511 chips). Same for all satellites (FDMA).
    svid parameter is accepted for API compatibility but ignored."""
    global _GLO_G1_CODE
    if _GLO_G1_CODE is not None:
        return _GLO_G1_CODE.copy()

    state = 0x1FC  # G1 LFSR init
    poly = 0x110   # G1 polynomial (x^5 + x^9)
    code = np.zeros(GLO_CODE_LENGTH, dtype=np.int8)
    for i in range(GLO_CODE_LENGTH):
        code[i] = 1 if (state & 0x100) else 0  # output = bit 8
        fb = bin(state & poly).count('1') & 1   # feedback = parity
        state = (state << 1) | fb
    _GLO_G1_CODE = 1 - 2 * code  # 0/1 → +1/-1
    return _GLO_G1_CODE.copy()


# ============================================================
# Generic LFSR / Gold Code Generator
# ============================================================

def generate_gold_code(g1_init, g1_poly, g2_init, g2_poly, length, depth, reset_pos):
    """Gold code: G1 XOR G2 Fibonacci LFSRs, with G2 reset at reset_pos.
    If g2_init=0 and g2_poly=0, G2 always outputs 0 (single-LFSR mode)."""
    output_mask = 1 << (depth - 1)
    depth_mask = (1 << depth) - 1
    g1 = g1_init
    g2 = g2_init
    code = np.zeros(length, dtype=np.int8)
    for i in range(length):
        if i == reset_pos:
            g2 = g2_init
        out1 = 1 if (g1 & output_mask) else 0
        fb1 = bin(g1 & g1_poly).count('1') & 1
        g1 = ((g1 << 1) | fb1) & depth_mask
        out2 = 1 if (g2 & output_mask) else 0
        fb2 = bin(g2 & g2_poly).count('1') & 1
        g2 = ((g2 << 1) | fb2) & depth_mask
        code[i] = out1 ^ out2
    return code


# ============================================================
# GPS L5I Code Generation (Gold Code, 13-bit LFSRs)
# ============================================================

L5I_PRN_INIT = [
    0x04ea, 0x1583, 0x0202, 0x0c8d, 0x1d77, 0x0be6, 0x1f25, 0x04bd,
    0x1a9f, 0x0f7e, 0x0b90, 0x13e7, 0x0738, 0x1c82, 0x0b56, 0x1278,
    0x1e32, 0x0f0f, 0x1f13, 0x16d6, 0x0204, 0x1ef7, 0x0fe1, 0x05a3,
    0x16cb, 0x0d35, 0x0f6a, 0x0d5e, 0x10fa, 0x1da1, 0x0f28, 0x13a0,
]


def generate_l5i_code(svid):
    """Generate GPS L5I code (10230 chips). Returns +/-1 array."""
    if svid < 1 or svid > 32:
        return None
    code = generate_gold_code(
        L5I_PRN_INIT[svid - 1], 0x18ed,   # G1: per-SV init, poly
        0x1fff, 0x1b00,                     # G2: all-ones init, poly
        10230, 13, 8190)                    # 10230 chips, 13-bit, G2 reset at 8190
    return 1.0 - 2.0 * code.astype(np.float64)


# ============================================================
# GPS L2C CM Code Generation (single 27-bit LFSR)
# ============================================================

L2CM_PRN_INIT = [
    0x15ef0f5, 0x50f811e, 0x10e553d, 0x16b0258, 0x416f3bc, 0x65bc21e, 0x0f5be58, 0x496777f,
    0x4a5a8e2, 0x36e44d6, 0x5e84705, 0x345ea19, 0x6965b5b, 0x447fb02, 0x0043a6e, 0x35e5896,
    0x3059ddd, 0x5c16d2a, 0x10c80db, 0x1c754b4, 0x650324e, 0x7fb4e14, 0x74e048f, 0x0663507,
    0x1f887f9, 0x487c247, 0x5fd6d8c, 0x20818d1, 0x1ece400, 0x7aeb923, 0x656b597, 0x602e157,
]


def generate_l2c_cm_code(svid):
    """Generate GPS L2C CM code (10230 chips). Returns +/-1 array.
    Single LFSR (G2 disabled via init=0, poly=0)."""
    if svid < 1 or svid > 32:
        return None
    code = generate_gold_code(
        L2CM_PRN_INIT[svid - 1], 0x0494953c,  # G1: per-SV init, 27-bit poly
        0x0, 0x0,                                # G2: disabled
        10230, 27, 10230)                        # 10230 chips, 27-bit, no reset
    return 1.0 - 2.0 * code.astype(np.float64)


# ============================================================
# GPS L1C Code Generation (Weil/Legendre + BOC(1,1))
# ============================================================

L1C_DATA_INSERT_INDEX = [
    181, 359, 72, 1110, 1480, 5034, 4622, 1, 4547, 826,
    6284, 4195, 368, 1, 4796, 523, 151, 713, 9850, 5734,
    34, 6142, 190, 644, 467, 5384, 801, 594, 4450, 9437,
    4307, 5906, 378, 9448, 9432, 5849, 5547, 9546, 9132, 403,
    3766, 3, 684, 9711, 333, 6124, 10216, 4251, 9893, 9884,
    4627, 4449, 9798, 985, 4272, 126, 10024, 434, 1029, 561,
    289, 638, 4353,
]

L1C_DATA_PHASE_DIFF = [
    5097, 5110, 5079, 4403, 4121, 5043, 5042, 5104, 4940, 5035,
    4372, 5064, 5084, 5048, 4950, 5019, 5076, 3736, 4993, 5060,
    5061, 5096, 4983, 4783, 4991, 4815, 4443, 4769, 4879, 4894,
    4985, 5056, 4921, 5036, 4812, 4838, 4855, 4904, 4753, 4483,
    4942, 4813, 4957, 4618, 4669, 4969, 5031, 5038, 4740, 4073,
    4843, 4979, 4867, 4964, 5025, 4579, 4390, 4763, 4612, 4784,
    3716, 4703, 4851,
]


def generate_l1c_weil(svid):
    """Generate GPS L1C data code (Weil, 10230 chips, 0/1).
    Legendre prime = 10223, 7-chip insertion sequence at insert_index."""
    if svid < 1 or svid > 63:
        return None
    insert_index = L1C_DATA_INSERT_INDEX[svid - 1]
    phase_diff = L1C_DATA_PHASE_DIFF[svid - 1]
    legendre = generate_legendre_sequence(10223)
    insert_seq = [0, 1, 1, 0, 1, 0, 0]
    code = np.zeros(10230, dtype=np.int32)
    idx1 = 0
    idx2 = phase_diff
    for i in range(10230):
        if idx1 >= 10223:
            idx1 -= 10223
        if idx2 >= 10223:
            idx2 -= 10223
        if i >= insert_index - 1 and i < insert_index + 6:
            code[i] = insert_seq[i - insert_index + 1]
        else:
            code[i] = legendre[idx1] ^ legendre[idx2]
            idx1 += 1
            idx2 += 1
    return code


def generate_l1c_boc(svid):
    """Generate GPS L1C BOC(1,1) code. Returns +/-1 array."""
    code = generate_l1c_weil(svid)
    if code is None:
        return None
    return apply_boc11(code)


# ============================================================
# CLI & Preset Parser
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description='Enhanced GNSS Signal Verification Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('iq_file', help='Path to IQ8 binary file')
    parser.add_argument('--preset', help='JSON preset file')
    parser.add_argument('--sample-rate', type=float, default=None, help='Sample rate in MHz')
    parser.add_argument('--output', default=None, help='Output report path (default: generated_files/verification_report.pdf)')
    parser.add_argument('--fast', action='store_true', help='Fast mode: only search RINEX-visible SVIDs')
    parser.add_argument('--threshold', type=float, default=ZSCORE_THRESHOLD, help=f'Z-score threshold (default: {ZSCORE_THRESHOLD})')
    return parser.parse_args()


def parse_preset(preset_path):
    """Parse JSON preset file and extract configuration."""
    with open(preset_path) as f:
        data = json.load(f)

    config = {}

    if 'output' in data and 'sampleFreq' in data['output']:
        config['sample_rate'] = data['output']['sampleFreq'] * 1e6

    if 'trajectory' in data and 'initPosition' in data['trajectory']:
        pos = data['trajectory']['initPosition']
        config['rx_lat'] = pos.get('latitude', 0)
        config['rx_lon'] = pos.get('longitude', 0)
        config['rx_alt'] = pos.get('altitude', 0)

    if 'time' in data:
        t = data['time']
        config['utc_time'] = datetime(
            t.get('year', 2025), t.get('month', 1), t.get('day', 1),
            t.get('hour', 0), t.get('minute', 0), t.get('second', 0))

    if 'ephemeris' in data and 'name' in data['ephemeris']:
        rinex_rel = data['ephemeris']['name']
        # Try relative to CWD first, then relative to preset parent dir
        if os.path.exists(rinex_rel):
            config['rinex_path'] = os.path.abspath(rinex_rel)
        else:
            preset_parent = os.path.dirname(os.path.dirname(os.path.abspath(preset_path)))
            candidate = os.path.join(preset_parent, rinex_rel)
            config['rinex_path'] = os.path.normpath(candidate)

    enabled = set()

    def _process_signal(sys_name, sig):
        """Map (system, signal) pair to enabled set entry."""
        if sys_name == 'GPS':
            if sig == 'L1CA':
                enabled.add('GPS')
            elif sig == 'L5':
                enabled.add('GPS_L5')
            elif sig == 'L2C':
                enabled.add('GPS_L2C')
            elif sig == 'L1C':
                enabled.add('GPS_L1C')
        elif sys_name == 'BDS' and sig == 'B1C':
            enabled.add('BDS')
        elif sys_name == 'Galileo' and sig == 'E1':
            enabled.add('GAL')
        elif sys_name == 'GLONASS' and sig == 'G1':
            enabled.add('GLO')

    # Format 1: systemSelect inside output (flat list with system+signal)
    if 'output' in data and 'systemSelect' in data['output']:
        for entry in data['output']['systemSelect']:
            if entry.get('enable', False):
                _process_signal(entry.get('system', ''), entry.get('signal', ''))

    # Format 2: systemSelect at top level with nested signalSelect
    if 'systemSelect' in data:
        for sys_entry in data['systemSelect']:
            sys_name = sys_entry.get('system', '')
            if sys_name == 'GPS' and sys_entry.get('enable', False):
                for sig_entry in sys_entry.get('signalSelect', []):
                    if sig_entry.get('enable', False):
                        _process_signal('GPS', sig_entry.get('signal', ''))
            elif sys_entry.get('enable', False):
                # Non-GPS systems with signalSelect
                for sig_entry in sys_entry.get('signalSelect', []):
                    if sig_entry.get('enable', False):
                        _process_signal(sys_name, sig_entry.get('signal', ''))

    config['enabled_systems'] = enabled if enabled else {'GPS', 'BDS', 'GAL'}

    if 'output' in data and 'centerFreq' in data['output']:
        config['center_freq'] = data['output']['centerFreq'] * 1e6

    if 'output' in data and 'config' in data['output']:
        config['elevation_mask'] = data['output']['config'].get('elevationMask', 5)

    return config


# ============================================================
# Time Conversion
# ============================================================

def utc_to_gps_time(utc_dt):
    """Convert UTC datetime to (gps_week, seconds_of_week)."""
    gps_dt = utc_dt + timedelta(seconds=GPS_LEAP_SECONDS)
    delta = gps_dt - GPS_EPOCH
    total_sec = delta.total_seconds()
    gps_week = int(total_sec) // 604800
    gps_sow = total_sec - gps_week * 604800
    return gps_week, gps_sow


# ============================================================
# RINEX 3.04 Navigation Parser
# ============================================================

def parse_rinex_line_values(line):
    """Extract scientific-notation floats from a RINEX data line."""
    return [float(v) for v in re.findall(r'[+-]?\d+\.\d+[eE][+-]?\d+', line)]


# Number of data lines per system in RINEX 3.04
_RINEX_DATA_LINES = {'G': 7, 'C': 7, 'E': 7, 'R': 3, 'J': 7, 'I': 7, 'S': 3}


def parse_rinex_nav(filepath, target_gps_week, target_gps_sow, time_window=7200):
    """
    Parse RINEX 3.04 nav file. Returns {system: {svid: [eph_dicts]}}.
    Filters to ephemerides within time_window of target.
    """
    if not os.path.exists(filepath):
        print(f"  RINEX not found: {filepath}")
        return {}

    target_time = target_gps_week * 604800.0 + target_gps_sow
    result = {'GPS': {}, 'BDS': {}, 'GAL': {}, 'GLO': {}}

    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Find end of header
    start = 0
    for i, line in enumerate(lines):
        if 'END OF HEADER' in line:
            start = i + 1
            break

    i = start
    while i < len(lines):
        line = lines[i]
        if len(line.rstrip()) < 4:
            i += 1
            continue

        sys_char = line[0]
        n_data = _RINEX_DATA_LINES.get(sys_char, 0)
        if n_data == 0:
            i += 1
            continue

        if sys_char not in ('G', 'C', 'E', 'R'):
            i += n_data + 1
            continue

        # Parse header: SV, epoch, clock
        try:
            svid = int(line[1:3])
        except ValueError:
            i += n_data + 1
            continue

        header_vals = parse_rinex_line_values(line[23:])
        af0 = header_vals[0] if len(header_vals) > 0 else 0.0

        # Collect all data values from data lines
        data_values = []
        for j in range(1, n_data + 1):
            if i + j < len(lines):
                data_values.extend(parse_rinex_line_values(lines[i + j]))

        i += n_data + 1

        # --- GLONASS ephemeris (3 data lines: X/Y/Z in km, V in km/s, A in km/s²) ---
        if sys_char == 'R':
            if len(data_values) < 12:
                continue

            # Parse epoch from header line (GLONASS uses UTC)
            try:
                epoch_year = int(line[4:8])
                epoch_month = int(line[9:11])
                epoch_day = int(line[12:14])
                epoch_hour = int(line[15:17])
                epoch_min = int(line[18:20])
                epoch_sec = int(line[21:23])
            except (ValueError, IndexError):
                continue

            # GLONASS epoch is in UTC, convert to GPS seconds
            try:
                utc_dt = datetime(epoch_year, epoch_month, epoch_day,
                                  epoch_hour, epoch_min, epoch_sec)
                gps_w, gps_s = utc_to_gps_time(utc_dt)
                toe_gps = gps_w * 604800.0 + gps_s
            except Exception:
                continue

            # Time filter
            if abs(toe_gps - target_time) > time_window:
                continue

            # Line 1: X(km), Xdot(km/s), Xdotdot(km/s²), health
            # Line 2: Y(km), Ydot(km/s), Ydotdot(km/s²), freq_number
            # Line 3: Z(km), Zdot(km/s), Zdotdot(km/s²), age
            eph = {
                'svid': svid,
                'x': data_values[0],    # km
                'vx': data_values[1],   # km/s
                'ax': data_values[2],   # km/s²
                'health': data_values[3],
                'y': data_values[4],    # km
                'vy': data_values[5],   # km/s
                'ay': data_values[6],   # km/s²
                'freq_num': int(data_values[7]),  # FDMA channel number k
                'z': data_values[8],    # km
                'vz': data_values[9],   # km/s
                'az': data_values[10],  # km/s²
                'toe_gps': toe_gps,
                'af0': af0,
            }
            result['GLO'].setdefault(svid, []).append(eph)
            continue

        if len(data_values) < 20:
            continue

        # Orbital parameters (same layout for GPS/BDS/GAL lines 1-4)
        eph = {
            'svid': svid, 'af0': af0,
            'crs': data_values[1], 'dn': data_values[2], 'M0': data_values[3],
            'cuc': data_values[4], 'ecc': data_values[5], 'cus': data_values[6],
            'sqrtA': data_values[7],
            'toe': data_values[8],
            'cic': data_values[9], 'omega0': data_values[10], 'cis': data_values[11],
            'i0': data_values[12], 'crc': data_values[13], 'w': data_values[14],
            'omega_dot': data_values[15],
            'idot': data_values[16],
        }

        # Week number is at index 18 for all systems
        raw_week = int(data_values[18]) if len(data_values) > 18 else target_gps_week

        # System-specific week correction
        if sys_char == 'C':
            eph['week'] = raw_week + BDT_WEEK_OFFSET
        else:
            eph['week'] = raw_week

        # Derived quantities
        eph['axis'] = eph['sqrtA'] ** 2
        n0 = math.sqrt(EARTH_GM / (eph['axis'] ** 3))
        eph['n'] = n0 + eph['dn']
        eph['root_ecc'] = math.sqrt(1.0 - eph['ecc'] ** 2)
        eph['toe_gps'] = eph['week'] * 604800.0 + eph['toe']

        # Time filter
        if abs(eph['toe_gps'] - target_time) > time_window:
            continue

        sys_key = {'G': 'GPS', 'C': 'BDS', 'E': 'GAL'}[sys_char]
        result[sys_key].setdefault(svid, []).append(eph)

    return result


def select_best_ephemeris(ephemerides, target_time):
    """For each SV, pick the ephemeris closest to target_time."""
    best = {}
    for system, sv_dict in ephemerides.items():
        best[system] = {}
        for svid, eph_list in sv_dict.items():
            best[system][svid] = min(eph_list, key=lambda e: abs(e['toe_gps'] - target_time))
    return best


# ============================================================
# Keplerian Orbit Propagation (GPS ICD-200)
# ============================================================

def kepler_propagate(eph, transmit_time, system='GPS'):
    """
    Propagate satellite ECEF position from ephemeris.
    Handles BeiDou GEO satellites (C01-C05, C59-C63) with special transform.
    """
    delta_t = transmit_time - eph['toe_gps']
    if delta_t > 302400:
        delta_t -= 604800
    if delta_t < -302400:
        delta_t += 604800

    # Kepler equation: E = M + e*sin(E)
    mk = eph['M0'] + eph['n'] * delta_t
    ek = mk
    for _ in range(10):
        ek_new = mk + eph['ecc'] * math.sin(ek)
        if abs(ek_new - ek) < 1e-14:
            break
        ek = ek_new
    ek = ek_new

    # True anomaly + argument of perigee
    phi = math.atan2(eph['root_ecc'] * math.sin(ek), math.cos(ek) - eph['ecc']) + eph['w']
    sin2phi = math.sin(2.0 * phi)
    cos2phi = math.cos(2.0 * phi)

    # Perturbation corrections
    uk = phi + eph['cuc'] * cos2phi + eph['cus'] * sin2phi
    rk = eph['axis'] * (1.0 - eph['ecc'] * math.cos(ek)) + eph['crc'] * cos2phi + eph['crs'] * sin2phi
    ik = eph['i0'] + eph['idot'] * delta_t + eph['cic'] * cos2phi + eph['cis'] * sin2phi

    # Orbital plane position
    xp = rk * math.cos(uk)
    yp = rk * math.sin(uk)

    is_geo = (system == 'BDS' and (eph['svid'] <= 5 or eph['svid'] >= 59))

    if is_geo:
        # BDS GEO: omega without Earth rotation subtraction
        omega_k = eph['omega0'] + eph['omega_dot'] * delta_t - OMEGA_E * eph['toe']
    else:
        # Standard ICD formula
        omega_k = eph['omega0'] + (eph['omega_dot'] - OMEGA_E) * delta_t - OMEGA_E * eph['toe']

    cos_o = math.cos(omega_k)
    sin_o = math.sin(omega_k)
    cos_i = math.cos(ik)
    sin_i = math.sin(ik)

    x = xp * cos_o - yp * cos_i * sin_o
    y = xp * sin_o + yp * cos_i * cos_o
    z = yp * sin_i

    if is_geo:
        # Apply Rx(-5 deg) then Rz(omega_e * delta_t)
        phi_rot = math.radians(-5.0)
        cp, sp = math.cos(phi_rot), math.sin(phi_rot)
        x_r = x
        y_r = cp * y - sp * z
        z_r = sp * y + cp * z

        theta = OMEGA_E * delta_t
        ct, st = math.cos(theta), math.sin(theta)
        x = ct * x_r - st * y_r
        y = st * x_r + ct * y_r
        z = z_r

    return x, y, z


def glonass_propagate(eph, transmit_time):
    """Propagate GLONASS satellite position using 2nd-order Taylor expansion.
    GLONASS ephemeris has XYZ in km, velocities in km/s, accelerations in km/s².
    Returns ECEF position in meters."""
    dt = transmit_time - eph['toe_gps']
    # Clamp to ±15 minutes for validity
    dt = max(-900.0, min(900.0, dt))
    x = (eph['x'] + eph['vx'] * dt + 0.5 * eph['ax'] * dt ** 2) * 1000.0
    y = (eph['y'] + eph['vy'] * dt + 0.5 * eph['ay'] * dt ** 2) * 1000.0
    z = (eph['z'] + eph['vz'] * dt + 0.5 * eph['az'] * dt ** 2) * 1000.0
    return x, y, z


# ============================================================
# Coordinate Conversion & Satellite Visibility
# ============================================================

def lla_to_ecef(lat_deg, lon_deg, alt):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    N = WGS_A / math.sqrt(1.0 - WGS_E2 * math.sin(lat) ** 2)
    return (
        (N + alt) * math.cos(lat) * math.cos(lon),
        (N + alt) * math.cos(lat) * math.sin(lon),
        (N * (1.0 - WGS_E2) + alt) * math.sin(lat),
    )


def ecef_to_el_az(rx_ecef, sat_ecef, rx_lat_deg, rx_lon_deg):
    """Compute elevation and azimuth (degrees) from receiver to satellite."""
    lat = math.radians(rx_lat_deg)
    lon = math.radians(rx_lon_deg)

    dx = sat_ecef[0] - rx_ecef[0]
    dy = sat_ecef[1] - rx_ecef[1]
    dz = sat_ecef[2] - rx_ecef[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist < 1.0:
        return 0.0, 0.0
    dx /= dist
    dy /= dist
    dz /= dist

    # ECEF -> ENU
    e = -math.sin(lon) * dx + math.cos(lon) * dy
    n = -math.sin(lat) * math.cos(lon) * dx - math.sin(lat) * math.sin(lon) * dy + math.cos(lat) * dz
    u = math.cos(lat) * math.cos(lon) * dx + math.cos(lat) * math.sin(lon) * dy + math.sin(lat) * dz

    el = math.degrees(math.asin(max(-1.0, min(1.0, u))))
    az = math.degrees(math.atan2(e, n))
    if az < 0:
        az += 360
    return el, az


def compute_satellite_doppler(eph, target_time, rx_ecef, system='GPS', carrier_freq=None):
    """Doppler via numerical velocity (central difference, dt=0.5s)."""
    dt = 0.5
    if system == 'GLO':
        propagate = glonass_propagate
        freq = GLO_F_BASE + eph.get('freq_num', 0) * GLO_F_STEP
    else:
        propagate = lambda e, t: kepler_propagate(e, t, system)
        freq = carrier_freq if carrier_freq is not None else F_L1
    pos1 = propagate(eph, target_time - dt)
    pos2 = propagate(eph, target_time + dt)
    vel = [(pos2[i] - pos1[i]) / (2 * dt) for i in range(3)]
    pos_t = propagate(eph, target_time)
    los = [pos_t[i] - rx_ecef[i] for i in range(3)]
    dist = math.sqrt(sum(x * x for x in los))
    unit_los = [x / dist for x in los]
    v_r = sum(vel[i] * unit_los[i] for i in range(3))
    return -v_r / SPEED_OF_LIGHT * freq


def compute_visible_satellites(best_eph, target_time, rx_lat, rx_lon, rx_alt, el_mask=5, carrier_freq=None):
    """Propagate all SVs and compute elevation/azimuth/doppler."""
    rx_ecef = lla_to_ecef(rx_lat, rx_lon, rx_alt)
    visible = {}
    for system, sv_dict in best_eph.items():
        visible[system] = {}
        for svid, eph in sv_dict.items():
            try:
                if system == 'GLO':
                    pos = glonass_propagate(eph, target_time)
                else:
                    pos = kepler_propagate(eph, target_time, system)
                el, az = ecef_to_el_az(rx_ecef, pos, rx_lat, rx_lon)
                doppler = compute_satellite_doppler(eph, target_time, rx_ecef, system, carrier_freq)
                info = {
                    'el': el, 'az': az, 'doppler': doppler,
                    'visible': el >= el_mask,
                }
                if system == 'GLO':
                    info['freq_num'] = eph.get('freq_num', 0)
                visible[system][svid] = info
            except Exception:
                pass
    return visible


# ============================================================
# Signal Loading
# ============================================================

def load_iq8(filename, sample_rate, offset_ms=0, duration_ms=10):
    samples_per_ms = int(sample_rate * 1e-3)
    n_samples = duration_ms * samples_per_ms
    byte_offset = offset_ms * samples_per_ms * 2

    filesize = os.path.getsize(filename)
    max_samples = filesize // 2
    if offset_ms * samples_per_ms + n_samples > max_samples:
        n_samples = max_samples - offset_ms * samples_per_ms

    raw = np.fromfile(filename, dtype=np.uint8, count=n_samples * 2, offset=byte_offset)
    iq = raw.astype(np.int8).astype(np.float64)
    return iq[0::2] + 1j * iq[1::2]


def resample_code(code, n_samples):
    indices = np.arange(n_samples) * len(code) / n_samples
    return code[indices.astype(int) % len(code)]


# ============================================================
# RMS Stability
# ============================================================

def compute_rms_stability(filename, sample_rate, block_ms=100):
    """Compute RMS per block across the entire file (memory-efficient)."""
    samples_per_ms = int(sample_rate * 1e-3)
    block_samples = block_ms * samples_per_ms
    block_bytes = block_samples * 2
    filesize = os.path.getsize(filename)
    n_blocks = filesize // block_bytes

    rms_values = []
    with open(filename, 'rb') as f:
        for _ in range(n_blocks):
            raw = np.frombuffer(f.read(block_bytes), dtype=np.uint8)
            if len(raw) < block_bytes:
                break
            iq = raw.astype(np.int8).astype(np.float64)
            I = iq[0::2]
            Q = iq[1::2]
            rms_values.append(np.sqrt(np.mean(I ** 2 + Q ** 2)))
    return np.array(rms_values)


# ============================================================
# Acquisition Engine
# ============================================================

def acquire_satellite_generic(signal_blocks, local_code_resampled, doppler_range, doppler_step,
                              sample_rate, non_coherent_count, save_heatmap=False,
                              if_offset=IF_FREQ):
    """
    FFT-based parallel code phase search.
    Returns (found, doppler, code_phase, zscore, best_corr, heatmap, doppler_bins).
    if_offset: carrier IF offset in Hz (default IF_FREQ=0 for L1 baseband, non-zero for GLONASS FDMA).
    """
    n = len(local_code_resampled)
    code_fft_conj = np.conj(fft(local_code_resampled))
    doppler_bins = np.arange(-doppler_range, doppler_range + 1, doppler_step)
    t = np.arange(n) / sample_rate

    best_zscore = 0.0
    best_doppler = 0
    best_code_phase = 0
    best_corr = None

    heatmap = np.zeros((len(doppler_bins), n)) if save_heatmap else None

    for d_idx, doppler in enumerate(doppler_bins):
        carrier = np.exp(-1j * 2 * np.pi * (if_offset + doppler) * t)
        correlation_sum = np.zeros(n)
        count = min(non_coherent_count, len(signal_blocks))

        for blk in signal_blocks[:count]:
            baseband = blk[:n] * carrier
            corr = np.abs(ifft(fft(baseband) * code_fft_conj)) ** 2
            correlation_sum += corr

        if save_heatmap:
            heatmap[d_idx, :] = correlation_sum

        peak_idx = np.argmax(correlation_sum)
        peak_val = correlation_sum[peak_idx]

        mask = np.ones(n, dtype=bool)
        exclude = max(50, n // 100)
        mask[max(0, peak_idx - exclude):min(n, peak_idx + exclude)] = False
        noise = correlation_sum[mask]
        noise_mean = np.mean(noise)
        noise_std = np.std(noise)
        zscore = (peak_val - noise_mean) / noise_std if noise_std > 0 else 0

        if zscore > best_zscore:
            best_zscore = zscore
            best_doppler = doppler
            best_code_phase = peak_idx
            best_corr = correlation_sum.copy()

    found = best_zscore > ZSCORE_THRESHOLD
    return found, best_doppler, best_code_phase, best_zscore, best_corr, heatmap, doppler_bins


def acquire_system(signal, system_name, svid_range, code_gen_fn, code_period_ms,
                   sample_rate, non_coherent, threshold=ZSCORE_THRESHOLD):
    """Run acquisition for one GNSS system. Returns results dict."""
    samples_per_ms = int(sample_rate * 1e-3)
    samples_per_code = code_period_ms * samples_per_ms

    blocks = [signal[i * samples_per_code:(i + 1) * samples_per_code]
              for i in range(len(signal) // samples_per_code)]

    all_results = []
    found_sats = []
    total = len(svid_range)

    print(f"\n{'=' * 60}")
    print(f"  {system_name} ACQUISITION")
    print(f"{'=' * 60}")
    print(f"  Doppler: +/-{DOPPLER_RANGE} Hz, step {DOPPLER_STEP} Hz, z-threshold={threshold}")

    for idx, svid in enumerate(svid_range):
        print(f"\r  Searching {system_name} SV{svid:02d}... ({idx + 1}/{total})", end='', flush=True)
        code = code_gen_fn(svid)
        if code is None:
            continue

        local_code = resample_code(code, samples_per_code)
        found, doppler, cp, zscore, _, _, _ = acquire_satellite_generic(
            blocks, local_code, DOPPLER_RANGE, DOPPLER_STEP,
            sample_rate, non_coherent, save_heatmap=False)

        r = {'svid': svid, 'doppler': doppler, 'code_phase': cp, 'zscore': zscore, 'found': found}
        all_results.append(r)
        if found:
            found_sats.append(r)

    print(f"\r{'':65}")

    # Top-10 diagnostics
    top = sorted(all_results, key=lambda x: -x['zscore'])[:10]
    print(f"  {'SV':>4}  {'Doppler':>10}  {'Z-score':>10}  Status")
    print(f"  {'---':>4}  {'---':>10}  {'---':>10}  ------")
    for r in top:
        st = "FOUND" if r['zscore'] > threshold else "noise"
        print(f"  {r['svid']:4d}  {r['doppler']:>+8.0f} Hz  {r['zscore']:>9.1f}  {st}")
    print(f"\n  RESULT: {len(found_sats)} {system_name} satellites found")

    # Heatmap for strongest SV
    best_heatmap = None
    best_corr = None
    best_doppler_bins = None
    if found_sats:
        best_sv = sorted(found_sats, key=lambda x: -x['zscore'])[0]
        print(f"  Computing 2D heatmap for SV{best_sv['svid']:02d}...")
        code = code_gen_fn(best_sv['svid'])
        local_code = resample_code(code, samples_per_code)
        _, _, _, _, best_corr, best_heatmap, best_doppler_bins = acquire_satellite_generic(
            blocks, local_code, DOPPLER_RANGE, DOPPLER_STEP,
            sample_rate, non_coherent, save_heatmap=True)

    return {
        'system': system_name,
        'found': found_sats,
        'all': all_results,
        'heatmap': best_heatmap,
        'correlation': best_corr,
        'doppler_bins': best_doppler_bins,
        'code_period_ms': code_period_ms,
        'samples_per_code': samples_per_code,
    }


def acquire_glonass_system(signal, sat_info_glo, center_freq, sample_rate,
                           non_coherent=NON_COHERENT_GLO, threshold=ZSCORE_THRESHOLD):
    """GLONASS FDMA acquisition: same PRN code, different carrier per satellite.
    Each SV is searched at its FDMA IF offset + Doppler range.
    Reuses acquire_satellite_generic with per-satellite if_offset."""
    samples_per_ms = int(sample_rate * 1e-3)
    samples_per_code = GLO_CODE_PERIOD_MS * samples_per_ms

    blocks = [signal[i * samples_per_code:(i + 1) * samples_per_code]
              for i in range(len(signal) // samples_per_code)]

    g1_code = generate_glonass_g1_code()
    local_code = resample_code(g1_code, samples_per_code)

    # Build search list: (svid, freq_num, if_offset)
    search_list = []
    for svid, info in sorted(sat_info_glo.items()):
        if not info['visible']:
            continue
        freq_num = info.get('freq_num', 0)
        carrier_freq = GLO_F_BASE + freq_num * GLO_F_STEP
        if_offset = carrier_freq - center_freq
        search_list.append((svid, freq_num, if_offset))

    all_results = []
    found_sats = []
    total = len(search_list)

    print(f"\n{'=' * 60}")
    print(f"  GLONASS G1 FDMA ACQUISITION")
    print(f"{'=' * 60}")
    print(f"  Center freq: {center_freq / 1e6:.4f} MHz")
    print(f"  Doppler: +/-{DOPPLER_RANGE} Hz, step {DOPPLER_STEP} Hz, z-threshold={threshold}")
    print(f"  Searching {total} visible GLONASS SVs")

    for idx, (svid, freq_num, if_offset) in enumerate(search_list):
        print(f"\r  Searching GLO R{svid:02d} (k={freq_num:+d}, IF={if_offset / 1e3:+.1f} kHz)... ({idx + 1}/{total})",
              end='', flush=True)

        found, doppler, cp, zscore, _, _, _ = acquire_satellite_generic(
            blocks, local_code, DOPPLER_RANGE, DOPPLER_STEP,
            sample_rate, non_coherent, save_heatmap=False, if_offset=if_offset)

        r = {'svid': svid, 'doppler': doppler, 'code_phase': cp,
             'zscore': zscore, 'found': found, 'freq_num': freq_num, 'if_offset': if_offset}
        all_results.append(r)
        if found:
            found_sats.append(r)

    print(f"\r{'':75}")

    # Top-10 diagnostics
    top = sorted(all_results, key=lambda x: -x['zscore'])[:10]
    print(f"  {'SV':>4}  {'k':>3}  {'IF':>10}  {'Doppler':>10}  {'Z-score':>10}  Status")
    print(f"  {'---':>4}  {'---':>3}  {'---':>10}  {'---':>10}  {'---':>10}  ------")
    for r in top:
        st = "FOUND" if r['zscore'] > threshold else "noise"
        print(f"  R{r['svid']:02d}  {r.get('freq_num', 0):+3d}  "
              f"{r.get('if_offset', 0) / 1e3:>+8.1f}kHz  {r['doppler']:>+8.0f} Hz  "
              f"{r['zscore']:>9.1f}  {st}")
    print(f"\n  RESULT: {len(found_sats)} GLONASS satellites found")

    # Heatmap for strongest SV
    best_heatmap = None
    best_corr = None
    best_doppler_bins = None
    if found_sats:
        best_sv = sorted(found_sats, key=lambda x: -x['zscore'])[0]
        print(f"  Computing 2D heatmap for R{best_sv['svid']:02d}...")
        _, _, _, _, best_corr, best_heatmap, best_doppler_bins = acquire_satellite_generic(
            blocks, local_code, DOPPLER_RANGE, DOPPLER_STEP,
            sample_rate, non_coherent, save_heatmap=True, if_offset=best_sv['if_offset'])

    return {
        'system': 'GLONASS G1',
        'found': found_sats,
        'all': all_results,
        'heatmap': best_heatmap,
        'correlation': best_corr,
        'doppler_bins': best_doppler_bins,
        'code_period_ms': GLO_CODE_PERIOD_MS,
        'samples_per_code': samples_per_code,
    }


# ============================================================
# CN0 Estimation
# ============================================================

def estimate_cn0(zscore, code_period_ms, non_coherent_count):
    """Estimate CN0 (dB-Hz) from z-score: CN0 = 10*log10(z^2 / (2*N*T))."""
    T = code_period_ms * 1e-3
    if zscore <= 0:
        return 0.0
    return 10 * math.log10(zscore ** 2 / (2 * non_coherent_count * T))


# ============================================================
# Report Generation (3 Pages)
# ============================================================

SYS_COLORS = {
    'GPS': '#2196F3', 'GPS_L5': '#1565C0', 'GPS_L2C': '#42A5F5', 'GPS_L1C': '#0D47A1',
    'BDS': '#F44336', 'GAL': '#4CAF50', 'GLO': '#FF9800',
}
SYS_PREFIX = {
    'GPS L1CA': ('G', 'GPS'), 'GPS L5': ('G', 'GPS_L5'),
    'GPS L2C': ('G', 'GPS_L2C'), 'GPS L1C': ('G', 'GPS_L1C'),
    'BeiDou B1C': ('C', 'BDS'),
    'Galileo E1': ('E', 'GAL'), 'GLONASS G1': ('R', 'GLO'),
}


def _all_found_bars(results_list, threshold):
    """Collect all found satellites for the bar chart."""
    items = []
    for res in results_list:
        if not res:
            continue
        prefix, sys_key = SYS_PREFIX.get(res['system'], ('?', ''))
        color = SYS_COLORS.get(sys_key, 'gray')
        for sat in res['found']:
            items.append((f'{prefix}{sat["svid"]:02d}', sat['zscore'], color, sys_key))
    items.sort(key=lambda x: x[1])
    return items


def _cn0_bars(results_list):
    """Collect CN0 estimations for all found sats."""
    nc_map = {
        'GPS L1CA': NON_COHERENT_GPS, 'GPS L5': NON_COHERENT_L5,
        'GPS L2C': NON_COHERENT_L2C, 'GPS L1C': NON_COHERENT_L1C,
        'BeiDou B1C': NON_COHERENT_BDS,
        'Galileo E1': NON_COHERENT_GAL, 'GLONASS G1': NON_COHERENT_GLO,
    }
    items = []
    for res in results_list:
        if not res:
            continue
        prefix, sys_key = SYS_PREFIX.get(res['system'], ('?', ''))
        color = SYS_COLORS.get(sys_key, 'gray')
        nc = nc_map.get(res['system'], 10)
        for sat in res['found']:
            cn0 = estimate_cn0(sat['zscore'], res['code_period_ms'], nc)
            items.append((f'{prefix}{sat["svid"]:02d}', cn0, color))
    items.sort(key=lambda x: -x[1])
    return items


def generate_report(iq_file, signal, sample_rate, rms_values,
                    results_list, sat_info, config, output_path, threshold):
    """Generate 3-page PDF report.
    results_list: [gps_res, bds_res, gal_res, glo_res] (None for absent systems)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    from matplotlib.patches import Patch

    with PdfPages(output_path) as pdf:
        # ======================================================
        # PAGE 1 — Signal Overview (2x2)
        # ======================================================
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(f'GNSS Signal Verification: {os.path.basename(iq_file)}',
                     fontsize=14, fontweight='bold')

        # [0,0] Power Spectrum (Welch)
        ax = axes[0, 0]
        f_welch, psd_welch = scipy_welch(signal, fs=sample_rate, nperseg=4096, return_onesided=False)
        idx = np.argsort(f_welch)
        ax.plot(f_welch[idx] / 1e6, 10 * np.log10(psd_welch[idx] + 1e-30), linewidth=0.5, color='navy')
        ax.axvline(0, color='red', linestyle='--', alpha=0.5, label='L1 center')
        ax.set_xlabel('Frequency offset (MHz)')
        ax.set_ylabel('PSD (dB/Hz)')
        ax.set_title('Power Spectrum (Welch)')
        ax.set_xlim(-sample_rate / 2e6, sample_rate / 2e6)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # [0,1] I/Q Histogram + Gaussian fit
        ax = axes[0, 1]
        n_hist = min(100000, len(signal))
        I_vals = np.real(signal[:n_hist])
        Q_vals = np.imag(signal[:n_hist])
        bins = np.arange(-130, 131, 2)
        ax.hist(I_vals, bins=bins, alpha=0.6, label=f'I (std={np.std(I_vals):.1f})', density=True, color='blue')
        ax.hist(Q_vals, bins=bins, alpha=0.6, label=f'Q (std={np.std(Q_vals):.1f})', density=True, color='red')
        x_g = np.linspace(-128, 128, 256)
        sigma = np.std(I_vals)
        gauss = np.exp(-x_g ** 2 / (2 * sigma ** 2)) / (sigma * np.sqrt(2 * np.pi))
        ax.plot(x_g, gauss, 'k--', linewidth=1.5, label='Gaussian fit')
        ax.set_xlabel('Sample value')
        ax.set_ylabel('Density')
        ax.set_title('I/Q Histogram')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # [1,0] I/Q Constellation
        ax = axes[1, 0]
        n_plot = min(5000, len(signal))
        Ip = np.real(signal[:n_plot])
        Qp = np.imag(signal[:n_plot])
        ax.scatter(Ip, Qp, s=0.3, alpha=0.3, color='steelblue')
        rms_iq = np.sqrt(np.mean(Ip ** 2 + Qp ** 2))
        theta = np.linspace(0, 2 * np.pi, 100)
        ax.plot(rms_iq * np.cos(theta), rms_iq * np.sin(theta), 'r-', linewidth=1.5, label=f'RMS={rms_iq:.1f}')
        ax.set_xlabel('I')
        ax.set_ylabel('Q')
        ax.set_title('I/Q Constellation')
        ax.set_aspect('equal')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # [1,1] RMS Stability
        ax = axes[1, 1]
        if len(rms_values) > 0:
            t_rms = np.arange(len(rms_values)) * 0.1
            ax.plot(t_rms, rms_values, linewidth=1, color='navy')
            ax.axhline(31.75, color='red', linestyle='--', alpha=0.7, label='Target 31.75')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('RMS')
            ax.set_title(f'RMS Stability (mean={np.mean(rms_values):.2f}, std={np.std(rms_values):.3f})')
            ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        pdf.savefig(fig, dpi=150)
        plt.close(fig)

        # ======================================================
        # PAGE 2 — Acquisition Results (2x2)
        # ======================================================
        fig = plt.figure(figsize=(14, 10))
        fig.suptitle('Acquisition Results & Satellite Visibility', fontsize=14, fontweight='bold')

        # [0,0] Z-Score Bar Chart
        ax_zscore = fig.add_subplot(2, 2, 1)
        bars_data = _all_found_bars(results_list, threshold)
        if bars_data:
            labels = [x[0] for x in bars_data]
            values = [x[1] for x in bars_data]
            bar_colors = [x[2] for x in bars_data]
            ax_zscore.barh(range(len(bars_data)), values, color=bar_colors, alpha=0.8)
            ax_zscore.set_yticks(range(len(bars_data)))
            ax_zscore.set_yticklabels(labels, fontsize=6)
            ax_zscore.axvline(threshold, color='k', linestyle='--', alpha=0.7)
            ax_zscore.set_xlabel('Z-score')
            handles = []
            for sk, c in SYS_COLORS.items():
                cnt = sum(1 for x in bars_data if x[3] == sk)
                if cnt > 0:
                    handles.append(Patch(facecolor=c, alpha=0.8, label=f'{sk} ({cnt})'))
            handles.append(plt.Line2D([0], [0], color='k', linestyle='--', label=f'z={threshold:.0f}'))
            ax_zscore.legend(handles=handles, fontsize=7, loc='lower right')
            ax_zscore.set_title(f'Detected Satellites ({len(bars_data)} total)')
        else:
            ax_zscore.text(0.5, 0.5, 'No satellites detected', ha='center', va='center',
                           transform=ax_zscore.transAxes)
            ax_zscore.set_title('Detected Satellites')
        ax_zscore.grid(True, alpha=0.3, axis='x')

        # [0,1] Skyplot (polar)
        ax_sky = fig.add_subplot(2, 2, 2, projection='polar')
        ax_sky.set_theta_zero_location('N')
        ax_sky.set_theta_direction(-1)
        ax_sky.set_rlim(0, 90)
        ax_sky.set_rticks([0, 30, 60, 90])
        ax_sky.set_yticklabels(['90', '60', '30', '0'], fontsize=7)
        ax_sky.set_title('Skyplot', pad=15)

        found_set = set()
        for res in results_list:
            if not res:
                continue
            _, sys_key = SYS_PREFIX.get(res['system'], ('?', ''))
            for sat in res['found']:
                found_set.add((sys_key, sat['svid']))

        if sat_info:
            for system, sv_dict in sat_info.items():
                color = SYS_COLORS.get(system, 'gray')
                prefix = {'GPS': 'G', 'BDS': 'C', 'GAL': 'E', 'GLO': 'R'}.get(system, '?')
                for svid, info in sv_dict.items():
                    if info['el'] < 0:
                        continue
                    az_rad = math.radians(info['az'])
                    r = 90 - info['el']
                    detected = (system, svid) in found_set
                    if detected:
                        ax_sky.scatter(az_rad, r, s=50, c=color, marker='o',
                                       zorder=5, edgecolors='black', linewidth=0.5)
                    else:
                        ax_sky.scatter(az_rad, r, s=50, facecolors='none', edgecolors=color,
                                       marker='o', zorder=4, linewidth=1.5)
                    ax_sky.annotate(f'{prefix}{svid}', (az_rad, r), fontsize=5,
                                    ha='center', va='bottom', xytext=(0, 4),
                                    textcoords='offset points')
        else:
            ax_sky.text(0, 45, 'No RINEX\n(use --preset)', ha='center', va='center',
                        fontsize=10, color='gray')

        # [1,0] CN0 Estimation
        ax_cn0 = fig.add_subplot(2, 2, 3)
        cn0_data = _cn0_bars(results_list)
        if cn0_data:
            labels = [x[0] for x in cn0_data]
            values = [x[1] for x in cn0_data]
            bar_colors = [x[2] for x in cn0_data]
            ax_cn0.bar(range(len(cn0_data)), values, color=bar_colors, alpha=0.8)
            ax_cn0.set_xticks(range(len(cn0_data)))
            ax_cn0.set_xticklabels(labels, rotation=45, fontsize=6, ha='right')
            ax_cn0.set_ylabel('CN0 (dB-Hz)')
            ax_cn0.axhline(45, color='green', linestyle='--', alpha=0.5, label='45 dB-Hz')
            ax_cn0.legend(fontsize=8)
            ax_cn0.set_title('Estimated CN0')
        else:
            ax_cn0.text(0.5, 0.5, 'No data', ha='center', va='center', transform=ax_cn0.transAxes)
            ax_cn0.set_title('CN0 Estimation')
        ax_cn0.grid(True, alpha=0.3, axis='y')

        # [1,1] Doppler: measured vs expected
        ax_dop = fig.add_subplot(2, 2, 4)
        if sat_info and bars_data:
            meas_dop, exp_dop, d_colors, d_labels = [], [], [], []
            for res in results_list:
                if not res:
                    continue
                _, sys_key = SYS_PREFIX.get(res['system'], ('?', ''))
                prefix = SYS_PREFIX.get(res['system'], ('?', ''))[0]
                color = SYS_COLORS.get(sys_key, 'gray')
                for sat in res['found']:
                    svid = sat['svid']
                    if sys_key in sat_info and svid in sat_info[sys_key]:
                        info = sat_info[sys_key][svid]
                        meas_dop.append(sat['doppler'])
                        exp_dop.append(info['doppler'])
                        d_colors.append(color)
                        d_labels.append(f'{prefix}{svid}')

            if meas_dop:
                ax_dop.scatter(exp_dop, meas_dop, c=d_colors, s=30, zorder=5,
                               edgecolors='black', linewidth=0.5)
                for i, lbl in enumerate(d_labels):
                    ax_dop.annotate(lbl, (exp_dop[i], meas_dop[i]), fontsize=5,
                                    xytext=(3, 3), textcoords='offset points')
                lim = max(abs(min(meas_dop + exp_dop)), abs(max(meas_dop + exp_dop))) * 1.15
                lim = max(lim, 100)
                ax_dop.plot([-lim, lim], [-lim, lim], 'k--', linewidth=0.5, alpha=0.5, label='Ideal')
                ax_dop.set_xlim(-lim, lim)
                ax_dop.set_ylim(-lim, lim)
                ax_dop.set_xlabel('Expected Doppler (Hz)')
                ax_dop.set_ylabel('Measured Doppler (Hz)')
                ax_dop.legend(fontsize=8)
                ax_dop.set_aspect('equal')
            else:
                ax_dop.text(0.5, 0.5, 'No matching data', ha='center', va='center',
                            transform=ax_dop.transAxes)
            ax_dop.set_title('Doppler: Measured vs Expected')
        else:
            ax_dop.text(0.5, 0.5, 'No RINEX\n(use --preset)', ha='center', va='center',
                        transform=ax_dop.transAxes, fontsize=10, color='gray')
            ax_dop.set_title('Doppler Comparison')
        ax_dop.grid(True, alpha=0.3)

        plt.tight_layout()
        pdf.savefig(fig, dpi=150)
        plt.close(fig)

        # ======================================================
        # PAGE 3 — Correlation Details (dynamic columns)
        # ======================================================

        # Code chip counts for correlation zoom
        _CODE_CHIP_MAP = {
            'GPS L1CA': (GPS_CODE_LENGTH, 50),
            'GPS L5': (GPS_L5_CODE_LENGTH, 50),
            'GPS L2C': (GPS_L2C_CODE_LENGTH, 50),
            'GPS L1C': (GPS_L1C_CODE_LENGTH * 2, 5),  # BOC doubles subchips
            'BeiDou B1C': (BDS_CODE_LENGTH, 5),
            'Galileo E1': (GAL_CODE_LENGTH, 5),
            'GLONASS G1': (GLO_CODE_LENGTH, 50),
        }

        # Build sys_plots from actual results
        sys_plots = []
        for res in results_list:
            if res is None:
                continue
            sn = res['system']
            _, sys_key = SYS_PREFIX.get(sn, ('?', 'GPS'))
            color = SYS_COLORS.get(sys_key, '#2196F3')
            chip_count, zoom_chips = _CODE_CHIP_MAP.get(sn, (1023, 50))
            sys_plots.append((sn, res, chip_count, zoom_chips, color))

        n_cols = max(len(sys_plots), 1)
        fig, axes = plt.subplots(2, n_cols, figsize=(5.5 * n_cols, 10), squeeze=False)
        fig.suptitle('Correlation Analysis', fontsize=14, fontweight='bold')

        for col, (sys_name, res, chip_count, zoom_chips, color) in enumerate(sys_plots):
            # Row 0: Correlation zoom around peak (circular roll to center)
            ax = axes[0, col]
            if res and res['correlation'] is not None:
                corr = res['correlation']
                n_c = len(corr)
                samples_per_chip = n_c / chip_count

                corr_norm = corr / np.max(corr)
                peak_idx = np.argmax(corr)

                # Circular roll so peak is at center of array
                shift = n_c // 2 - peak_idx
                corr_centered = np.roll(corr_norm, shift)

                zoom_s = int(zoom_chips * samples_per_chip)
                center = n_c // 2
                s0 = center - zoom_s
                s1 = center + zoom_s
                chip_offsets = (np.arange(s0, s1) - center) / samples_per_chip

                ax.plot(chip_offsets, corr_centered[s0:s1], linewidth=1, color=color)
                ax.set_xlabel('Code offset (chips)')
                ax.set_ylabel('Normalized')
                ax.set_xlim(-zoom_chips, zoom_chips)
                best_sv = sorted(res['found'], key=lambda x: -x['zscore'])[0]
                prefix = SYS_PREFIX.get(sys_name, ('?', ''))[0]
                ax.set_title(f'{sys_name} {prefix}{best_sv["svid"]:02d} '
                             f'(z={best_sv["zscore"]:.0f}, D={best_sv["doppler"]:+.0f}Hz)')
            else:
                ax.text(0.5, 0.5, f'No {sys_name}', ha='center', va='center', transform=ax.transAxes)
                ax.set_title(f'{sys_name} Correlation')
            ax.grid(True, alpha=0.3)

            # Row 1: 2D Heatmap zoomed around peak (Doppler x Code Phase)
            ax = axes[1, col]
            if res and res['heatmap'] is not None:
                hm = res['heatmap']
                dbins = res['doppler_bins']
                n_c = hm.shape[1]
                samples_per_chip = n_c / chip_count

                # Find peak in heatmap
                peak_flat = np.argmax(hm)
                peak_dop_idx, peak_cp_idx = np.unravel_index(peak_flat, hm.shape)

                # Zoom: ±200 chips for GPS, ±code_length/4 for BDS/GAL
                zoom_cp_chips = min(200, chip_count // 4)
                zoom_cp_s = int(zoom_cp_chips * samples_per_chip)

                # Circular roll code phase axis to center the peak
                shift_cp = n_c // 2 - peak_cp_idx
                hm_rolled = np.roll(hm, shift_cp, axis=1)
                center_cp = n_c // 2
                cp_s0 = max(0, center_cp - zoom_cp_s)
                cp_s1 = min(n_c, center_cp + zoom_cp_s)
                hm_zoom = hm_rolled[:, cp_s0:cp_s1]

                # Downsample for display
                step = max(1, hm_zoom.shape[1] // 500)
                hm_ds = hm_zoom[:, ::step]
                chip_ax = (np.arange(hm_ds.shape[1]) * step + (cp_s0 - center_cp)) / samples_per_chip

                hm_db = 10 * np.log10(hm_ds / np.max(hm_ds) + 1e-10)
                im = ax.pcolormesh(chip_ax, dbins, hm_db, cmap='jet', vmin=-20, vmax=0, shading='auto')
                fig.colorbar(im, ax=ax, label='dB', shrink=0.8)
                ax.set_xlabel('Code offset (chips)')
                ax.set_ylabel('Doppler (Hz)')
                ax.set_title(f'{sys_name} 2D Acquisition (zoom)')
            else:
                ax.text(0.5, 0.5, f'No {sys_name} data', ha='center', va='center', transform=ax.transAxes)
                ax.set_title(f'{sys_name} 2D Heatmap')
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        pdf.savefig(fig, dpi=150)
        plt.close(fig)

    print(f"\n  Report saved: {output_path}")


# ============================================================
# Main
# ============================================================

def main():
    args = parse_args()

    if not os.path.exists(args.iq_file):
        print(f"File not found: {args.iq_file}")
        return

    sample_rate = DEFAULT_SAMPLE_RATE
    config = {}
    threshold = args.threshold

    if args.preset:
        if not os.path.exists(args.preset):
            print(f"Preset not found: {args.preset}")
        else:
            config = parse_preset(args.preset)
            if 'sample_rate' in config:
                sample_rate = config['sample_rate']

    if args.sample_rate is not None:
        sample_rate = args.sample_rate * 1e6

    samples_per_ms = int(sample_rate * 1e-3)

    # File info
    filesize = os.path.getsize(args.iq_file)
    total_ms = filesize // (samples_per_ms * 2)
    print(f"\nFile: {args.iq_file}")
    print(f"Size: {filesize / 1e6:.1f} MB")
    print(f"Duration: {total_ms} ms ({total_ms / 1000:.1f} s)")
    print(f"Sample rate: {sample_rate / 1e6:.1f} MHz")

    # ---- RINEX Parsing & Satellite Visibility ----
    sat_info = None
    if 'rinex_path' in config and 'utc_time' in config:
        rpath = config['rinex_path']
        print(f"\nRINEX: {rpath}")
        gps_week, gps_sow = utc_to_gps_time(config['utc_time'])
        target_time = gps_week * 604800.0 + gps_sow
        print(f"  UTC: {config['utc_time']}  ->  GPS week {gps_week}, SoW {gps_sow:.0f}")

        all_eph = parse_rinex_nav(rpath, gps_week, gps_sow)
        for sk, sv_dict in all_eph.items():
            n_sv = len(sv_dict)
            n_eph = sum(len(v) for v in sv_dict.values())
            if n_eph > 0:
                print(f"  {sk}: {n_sv} SVs, {n_eph} ephemerides loaded")

        best_eph = select_best_ephemeris(all_eph, target_time)

        if 'rx_lat' in config:
            el_mask = config.get('elevation_mask', 5)
            sat_info = compute_visible_satellites(
                best_eph, target_time,
                config['rx_lat'], config['rx_lon'], config['rx_alt'], el_mask,
                carrier_freq=config.get('center_freq'))
            for system in ['GPS', 'BDS', 'GAL', 'GLO']:
                if system in sat_info:
                    vis = sum(1 for s in sat_info[system].values() if s['visible'])
                    tot = len(sat_info[system])
                    if system == 'GLO' and vis > 0:
                        channels = ', '.join(f'R{sv}(k={info["freq_num"]:+d})'
                                             for sv, info in sorted(sat_info[system].items())
                                             if info['visible'])
                        print(f"  {system} visible: {vis}/{tot} (el>{el_mask} deg): {channels}")
                    else:
                        print(f"  {system} visible: {vis}/{tot} (el>{el_mask} deg)")

    # ---- RMS Stability ----
    print("\nComputing RMS stability...")
    rms_values = compute_rms_stability(args.iq_file, sample_rate, block_ms=100)
    if len(rms_values) > 0:
        print(f"  RMS: mean={np.mean(rms_values):.2f}, std={np.std(rms_values):.3f}")

    # ---- Load signal for acquisition ----
    load_ms = min(500, total_ms)
    print(f"\nLoading first {load_ms} ms...")
    signal = load_iq8(args.iq_file, sample_rate, offset_ms=0, duration_ms=load_ms)

    I = np.real(signal)
    Q = np.imag(signal)
    print(f"  I: mean={np.mean(I):.2f}, std={np.std(I):.2f}")
    print(f"  Q: mean={np.mean(Q):.2f}, std={np.std(Q):.2f}")
    print(f"  RMS: {np.sqrt(np.mean(np.abs(signal) ** 2)):.2f}")

    # ---- Determine search ranges ----
    enabled = config.get('enabled_systems', {'GPS', 'BDS', 'GAL'})
    any_gps = any(s in enabled for s in ('GPS', 'GPS_L5', 'GPS_L2C', 'GPS_L1C'))
    gps_range = list(range(1, 33)) if any_gps else []
    bds_range = list(range(1, 64)) if 'BDS' in enabled else []
    gal_range = list(range(1, 37)) if 'GAL' in enabled else []

    if args.fast and sat_info:
        print("\n  ** Fast mode: searching only RINEX-visible SVIDs **")
        if 'GPS' in sat_info and gps_range:
            gps_range = sorted(s for s in sat_info['GPS'] if sat_info['GPS'][s]['visible'])
            print(f"  GPS: {len(gps_range)} SVs")
        if 'BDS' in sat_info and bds_range:
            bds_range = sorted(s for s in sat_info['BDS'] if sat_info['BDS'][s]['visible'])
            print(f"  BDS: {len(bds_range)} SVs")
        if 'GAL' in sat_info and gal_range:
            gal_range = sorted(s for s in sat_info['GAL'] if sat_info['GAL'][s]['visible'])
            print(f"  GAL: {len(gal_range)} SVs")

    # ---- Acquisition ----
    results_list = []

    # GPS L1CA
    if 'GPS' in enabled and gps_range:
        res = acquire_system(signal, 'GPS L1CA', gps_range, generate_ca_code,
                             GPS_CODE_PERIOD_MS, sample_rate, NON_COHERENT_GPS, threshold)
        results_list.append(res)

    # GPS L5
    if 'GPS_L5' in enabled and gps_range:
        res = acquire_system(signal, 'GPS L5', gps_range, generate_l5i_code,
                             GPS_L5_CODE_PERIOD_MS, sample_rate, NON_COHERENT_L5, threshold)
        results_list.append(res)

    # GPS L2C
    if 'GPS_L2C' in enabled and gps_range:
        res = acquire_system(signal, 'GPS L2C', gps_range, generate_l2c_cm_code,
                             GPS_L2C_CODE_PERIOD_MS, sample_rate, NON_COHERENT_L2C, threshold)
        results_list.append(res)

    # GPS L1C
    if 'GPS_L1C' in enabled and gps_range:
        res = acquire_system(signal, 'GPS L1C', gps_range, generate_l1c_boc,
                             GPS_L1C_CODE_PERIOD_MS, sample_rate, NON_COHERENT_L1C, threshold)
        results_list.append(res)

    # BeiDou B1C
    if bds_range:
        res = acquire_system(signal, 'BeiDou B1C', bds_range, generate_b1c_boc,
                             BDS_CODE_PERIOD_MS, sample_rate, NON_COHERENT_BDS, threshold)
        results_list.append(res)

    # Galileo E1
    if gal_range:
        res = acquire_system(signal, 'Galileo E1', gal_range, generate_e1_boc,
                             GAL_CODE_PERIOD_MS, sample_rate, NON_COHERENT_GAL, threshold)
        results_list.append(res)

    # GLONASS FDMA acquisition (requires sat_info for frequency channels)
    center_freq = config.get('center_freq', F_L1)
    if 'GLO' in enabled and sat_info and 'GLO' in sat_info:
        glo_visible = {sv: info for sv, info in sat_info['GLO'].items() if info['visible']}
        if glo_visible:
            res = acquire_glonass_system(signal, sat_info['GLO'], center_freq,
                                         sample_rate, NON_COHERENT_GLO, threshold)
            results_list.append(res)

    # ---- Summary ----
    total_found = sum(len(r['found']) for r in results_list)

    print(f"\n{'=' * 60}")
    print(f"  SUMMARY")
    print(f"{'=' * 60}")
    for res in results_list:
        n = len(res['found'])
        print(f"  {res['system']:14s} {n} satellites")
    print(f"  {'TOTAL':14s} {total_found} satellites")

    for res in results_list:
        if res['found']:
            prefix = SYS_PREFIX.get(res['system'], ('?', ''))[0]
            if 'GLO' in res['system']:
                sats_str = ', '.join(
                    f'{prefix}{s["svid"]:02d}(k={s.get("freq_num", 0):+d},z={s["zscore"]:.0f})'
                    for s in sorted(res['found'], key=lambda x: -x['zscore']))
            else:
                sats_str = ', '.join(
                    f'{prefix}{s["svid"]:02d}(z={s["zscore"]:.0f})'
                    for s in sorted(res['found'], key=lambda x: -x['zscore']))
            print(f"\n  {res['system']}: {sats_str}")

    # ---- Generate report ----
    output_path = args.output or 'generated_files/verification_report.pdf'
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    print(f"\nGenerating report: {output_path}")
    try:
        generate_report(args.iq_file, signal, sample_rate, rms_values,
                        results_list,
                        sat_info, config, output_path, threshold)
    except Exception as e:
        print(f"  Error: {e}")
        import traceback
        traceback.print_exc()

    print("\nDone!")


if __name__ == '__main__':
    main()
