# gnss-signal-simulator-rs
[![Boosty](https://img.shields.io/badge/Boosty-Buy_me_a_coffee-FF7143?logo=boosty&logoColor=white&style=for-the-badge)](https://boosty.to/danusha/donate)

🌐 **English** · [Русский](README.ru.md)

**Multi-constellation GNSS IF signal simulator written in Rust**

Generate realistic IQ baseband samples for GPS, Galileo, BeiDou, and GLONASS from RINEX 3.04 navigation data. The only GNSS signal simulator written in Rust — zero-copy, memory-safe, with AVX-512 and optional CUDA acceleration.

---

## Supported Signals

| System | Signal | Frequency | Modulation | Code Length | Verified z-score |
|--------|--------|-----------|------------|-------------|-----------------|
| GPS | L1CA | 1575.42 MHz | BPSK(1) | 1023 | 58–88 |
| GPS | L5 | 1176.45 MHz | BPSK(10) | 10230 | 44–59 |
| GPS | L2C | 1227.60 MHz | BPSK(1) | 10230 | Verified |
| GPS | L1C | 1575.42 MHz | BOC(1,1) | 10230 | Verified |
| Galileo | E1 | 1575.42 MHz | CBOC(6,1,1/11) | 4092 | 63–103 |
| Galileo | E5a | 1176.45 MHz | BPSK(10) | 10230 | 48–64 |
| Galileo | E5b | 1207.14 MHz | BPSK(10) | 10230 | 449–514 |
| Galileo | E6 | 1278.75 MHz | BPSK(5) | 5115 | 270–299 |
| BeiDou | B1C | 1575.42 MHz | BOC(1,1)+QMBOC(6,1,4/33) | 10230 | 132–247 |
| BeiDou | B1I | 1561.098 MHz | BPSK(2) | 2046 | 65–96 |
| BeiDou | B2a | 1176.45 MHz | BPSK(10) | 10230 | 48–72 |
| GLONASS | G1 | 1602+k×0.5625 MHz | BPSK(0.5) FDMA | 511 | 54–56 |

> z-score = (correlation peak − mean) / std, threshold ≥ 30. Values above 50 indicate reliable detection.

## Key Features

- **4 GNSS constellations, 12 verified signals** — simultaneous multi-system generation into a single IQ file
- **RINEX 3.04 ephemeris parsing** — GPS (7-line), BeiDou (7-line with BDT correction), Galileo (7-line), GLONASS (3-line with km→m)
- **Keplerian orbit propagation** — GPS ICD-200 algorithm; GLONASS RK4 in PZ-90
- **Unified epoch algorithm** — forces all satellites to use ephemeris from the same time epoch
- **Physically correct AGC** — RMS-based controller, Gaussian I/Q distribution, ~0% clipping
- **AVX-512 acceleration** — automatic runtime detection, SIMD-optimized hot paths
- **Optional CUDA GPU offload** — mass PRN×carrier×amplitude computation per millisecond
- **Multithreading** — Rayon-based parallel satellite processing
- **IQ8 output** — signed 8-bit I/Q interleaved samples, compatible with SDR receivers (GNURadio, SDR#)
- **Python verification tool** — 3-page PDF report with PSD, acquisition, correlation analysis
- **Minimal dependencies** — only `rand`, `rayon`, `wide`, `serde_json` (+ optional `cudarc`)
- **~55,000 lines of Rust** — full GNSS stack implemented from scratch

## Quick Start

### Requirements

- Stable Rust (edition 2021) and `cargo`
- Recommended: x86_64 CPU (AVX-512 used automatically if available)
- Optional: NVIDIA driver + CUDA 12.5 for GPU acceleration

### Build and Run

```bash
# Build optimized release
cargo build --release

# Generate GPS L1CA signal (simplest example)
cargo run --release -- presets/gps_l1ca.json

# Generate triple-system GPS + BeiDou + Galileo
cargo run --release -- presets/gps_bds_gal_l1.json

# Generate quad-system GPS + BeiDou + Galileo + GLONASS (46.5 MHz)
cargo run --release -- presets/quad_l1g1.json

# Verbose output
cargo run --release -- -v presets/gps_l1ca.json
```

Output: binary IQ file (e.g. `generated_files/gps_l1ca.C8`).

### Build with GPU (optional)

```bash
cargo build --release --features gpu
cargo run --release --features gpu -- presets/gps_l1ca.json
```

Requires CUDA 12.5 with NVRTC. Falls back to CPU automatically if CUDA is unavailable.

## Presets

Ready-to-use JSON configurations in `presets/`:

| Preset | Systems | Sample Rate |
|--------|---------|-------------|
| `gps_l1ca.json` | GPS L1CA | 5 MHz |
| `gps_l5.json` | GPS L5 | 21 MHz |
| `bds_b1c.json` | BeiDou B1C | 5 MHz |
| `gal_e1.json` | Galileo E1 | 5 MHz |
| `gal_e5a.json` | Galileo E5a | 21 MHz |
| `gal_e5b.json` | Galileo E5b | 21 MHz |
| `gal_e6.json` | Galileo E6 | 11 MHz |
| `glo_g1.json` | GLONASS G1 | 10 MHz |
| `gps_bds_gal_l1.json` | GPS+BDS+GAL L1 | 5 MHz |
| `gps_bds_gal_l1_300s.json` | GPS+BDS+GAL L1 (over-air, 52 dBHz) | 8 MHz |
| `quad_l1g1.json` | GPS+BDS+GAL+GLO L1/G1 | 46.5 MHz |

The repository ships 46 presets. Most use the Montana location, 10s duration, elevation mask 5°, full RINEX; the `moscow_*` and `usa_*` presets are longer real-receiver scenarios (60–300s) at their respective locations.

Custom presets: copy any JSON, edit receiver position (LLA), sample rate, RINEX path, duration, and signal selection.

### Over-the-Air Replay (HackRF / PortaPack → real receiver)

When replaying a generated file over the air into a real receiver (phone, u-blox), the defaults tuned for the software verifier are too weak. For a phone to **acquire and sustain tracking** (decode ephemeris → get a fix):

- **`initPower` ≥ 52 dBHz** in the preset. The AGC normalizes the file to a fixed output RMS regardless of `initPower`, so this sets the in-file CN0 (signal-to-noise margin), not the loudness. The default 45 dBHz acquires in the verifier but a phone cannot hold tracking.
- **Sample rate ≥ 8 MHz.** HackRF One does 2–20 Msps but `<8 MHz` is out of DAC spec; PortaPack replay is SD-throughput-limited, so ~8–10 Msps is the practical sweet spot.
- **E1/B1C use pure BOC(1,1) below 12.276 MHz automatically.** The true CBOC/QMBOC BOC(6,1) subcarrier (6.138 MHz) aliases into the data channel at low sample rates and blocks nav-message decode on real receivers; the generator falls back to clean BOC(1,1) (which a phone's narrow L1 front-end uses anyway) and switches to true CBOC only at `Fs ≥ 12.276 MHz`.
- **BeiDou on phones is B1I (1561.098 MHz), not B1C (1575.42 MHz).** B1I is outside an L1-centered file, so use a dedicated B1I preset for BeiDou on a phone.

`presets/gps_bds_gal_l1_300s.json` is preconfigured for this (8 MHz, 52 dBHz, 300 s).

## Signal Verification

The included Python tool `verify_signal_enhanced.py` generates a 3-page PDF diagnostic report.

### Usage

```bash
# Install dependencies
pip install numpy matplotlib

# Verify with auto-configuration from preset
python verify_signal_enhanced.py generated_files/gps_bds_gal_l1.C8 \
    --preset presets/gps_bds_gal_l1.json

# Fast mode (search only RINEX-visible satellites)
python verify_signal_enhanced.py generated_files/gps_l1ca.C8 \
    --preset presets/gps_l1ca.json --fast

# Custom parameters
python verify_signal_enhanced.py generated_files/output.C8 \
    --sample-rate 5.0 --threshold 30 --output report.pdf
```

### PDF Report Contents

- **Page 1 — Signal Overview**: Welch PSD, I/Q histogram with Gaussian fit, constellation diagram, RMS stability
- **Page 2 — Acquisition Results**: z-score bar chart (color-coded by system), polar skyplot (expected vs detected), CN0 estimation, Doppler accuracy
- **Page 3 — Correlation Analysis**: Zoomed correlation peaks per system, 2D Doppler×Code heatmaps

### Verification Results

**Triple-system (GPS + BeiDou + Galileo, 5 MHz, 10s, Chicago):**

| System | RINEX Visible | Detected | z-score Range |
|--------|--------------|----------|---------------|
| GPS L1CA | 11 | 11 | 58–88 |
| BeiDou B1C | 8 | 6 | 574–955 |
| Galileo E1 | 10 | 6 | 364–425 |
| **Total** | **29** | **23** | **100% of generated** |

**Galileo multi-band (Montana, 10s each):**

| Signal | Sample Rate | Detected | z-score Range |
|--------|-------------|----------|---------------|
| E5a | 21 MHz | 7/7 | 434–502 |
| E5b | 21 MHz | 7/7 | 449–514 |
| E6 | 11 MHz | 7/7 | 270–299 |

**Single-system results after the June 2026 passes (true CBOC/QMBOC waveforms + NH-robust verifier):**

| Signal | Sample Rate | RINEX Visible | Detected | z-score Range |
|--------|-------------|--------------|----------|---------------|
| Galileo E1 (CBOC) | 5 MHz | 10 | 10/10 | 63–103 |
| BeiDou B1C (QMBOC, pilot acquisition) | 5 MHz | 12 | 12/12 | 132–247 |
| BeiDou B1I | 5 MHz | 12 | 12/12 | 65–96 |
| GPS L5 | 21 MHz | 10 | 10/10 | 44–59 |
| Galileo E5a | 21 MHz | 10 | 10/10 | 48–64 |
| BeiDou B2a | 21 MHz | 12 | 12/12 | 48–72 |

> PRN codes cross-verified bit-exact against independent references: B1I vs gnss-sdr (63/63 PRN); B1C data/pilot/secondary, E5a I/Q, B2a data/pilot and all NH/secondary codes vs PocketSDR. GPS L5 XA/XB is ICD-literal (free-running XB; the references' mod-10230 roll diverges in the last `adv` chips per PRN, <0.12 dB effect).
>
> The verifier uses double-window linear correlation (immune to NH/secondary chip flips at code epochs) with code-Doppler drift compensation — the earlier 0/10 GPS L5 result was a verifier artifact, confirmed by a clean-room dump showing ideal per-period correlation and bit-exact NH wiring.

**GLONASS G1 (10 MHz, Montana, 10s):**

| Generated | Detected | z-score Range |
|-----------|----------|---------------|
| 7 SVs | 5 | 54–56 |

## Architecture

```
src/
├── main.rs              # Entry point — reads JSON preset, runs generation
├── lib.rs               # Public module exports
├── ifdatagen.rs         # Core IF data generation (AGC, satellite signals, noise)
├── sat_if_signal.rs     # Per-satellite IF sample generation (PRN codes, BOC)
├── coordinate.rs        # Keplerian propagator, ECEF/LLA/ENU conversions
├── json_interpreter.rs  # RINEX 3.04 parser, JSON preset parser
├── gnsstime.rs          # GPS/BDT/GST/GLONASS time system conversions
├── types.rs             # Core types: GpsEphemeris, BeiDouEphemeris, KinematicInfo
├── constants.rs         # WGS84, PZ-90, CGCS2000 constants
├── prngenerate.rs       # PRN code generation (Gold, Weil/Legendre, memory codes)
├── memory_code_e1.rs    # Galileo E1 memory codes (4092 chips)
├── memory_code_e6.rs    # Galileo E6 memory codes (5115 chips)
├── fastmath.rs          # Optimized sin/cos/atan2 approximations
├── complex_number.rs    # Lightweight complex arithmetic
├── satellite_param.rs   # Satellite parameters and signal configuration
├── satellite_signal.rs  # Signal-level satellite processing
├── almanac.rs           # Almanac parsing and type detection
├── trajectory.rs        # Receiver trajectory generation
├── *navbit.rs           # Navigation message generators (GPS LNAV, Galileo I/NAV,
│                        #   F/NAV, BeiDou BCNav1/2/3, GLONASS G-NAV, etc.)
├── nav_decode.rs        # Reverse decoders + ICD parity/CRC verifiers for nav messages
├── gps_pilot/mod.rs     # Clean minimal phase-continuous GPS L1CA generator
└── bin/
    ├── spectrum_analyzer.rs  # IF spectrum analysis (PSD, peaks, CSV export)
    ├── nav_diagnostics.rs    # Nav message encode→decode→compare vs RINEX (ICD verify)
    ├── decode_iq.rs          # GPS L1CA IQ8 decoder — acquisition, tracking, LNAV decode
    ├── gnss_pilot.rs         # Clean GPS L1CA + Galileo E1 + BeiDou B1C L1-band generator
    ├── gps_pilot.rs          # Clean minimal GPS L1CA generator
    ├── nav_test.rs           # Navigation bit unit tests
    ├── bench.rs              # CPU/AVX-512/GPU benchmarks
    └── extreme_bench.rs      # Stress benchmarks
```

### RINEX Data

- `Rinex_Data/rinex_v3_20251560000.rnx` — 2025-06-05, GPS/BDS/GAL
- `Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx` — 2025-06-05, full merged GPS/BDS/GAL/GLO

## Output Format

**IQ8**: signed 8-bit interleaved I/Q samples (`.C8` extension)

```
[I₀, Q₀, I₁, Q₁, I₂, Q₂, ...]   — each sample is int8 (-128..+127)
```

- AGC target RMS: 0.25 (quantized: ~31.75 in int8 units)
- Compatible with GNURadio (`file_source` → `char_to_float`), SDR#, and custom SDR receivers
- File size: `2 × sample_rate × duration` bytes (e.g. 5 MHz × 10s = 100 MB)

## Spectrum Analyzer

Built-in IF spectrum analysis tool:

```bash
cargo run --release --bin spectrum_analyzer -- generated_files/gps_l1ca.C8 iq8 5000000 0 --csv
```

Outputs PSD to terminal and optionally to `spectrum.csv`.

## Development

```bash
cargo check          # Fast syntax/type check
cargo test           # Run unit tests
cargo fmt --all      # Format code
cargo clippy         # Lint
```

Detailed verification status is tracked in `docs/verification_report.md` and
`docs/gnss_icd_conformance_report.md`.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "Configuration file not found" | Use presets from `presets/` or edit generated `config.json` |
| "RINEX file not found" | Check relative paths in preset, ensure files exist in `Rinex_Data/` |
| CUDA not found | Install CUDA 12.5, verify NVRTC in PATH/LD_LIBRARY_PATH |
| Slow generation | Use `--release`, use `--features gpu` |
| 0 visible satellites | Check RINEX date matches preset time, verify receiver coordinates |

## License

See source files for license information.

---

## Описание (Русский)

**Мультисистемный симулятор GNSS-сигналов на Rust**

Генерация реалистичных IQ-отсчётов промежуточной частоты для GPS, Galileo, BeiDou и GLONASS из навигационных данных RINEX 3.04.

### Поддерживаемые сигналы

- **GPS**: L1CA, L5, L2C, L1C
- **Galileo**: E1 (CBOC), E5a, E5b, E6
- **BeiDou**: B1C (BOC+QMBOC), B1I, B2a
- **GLONASS**: G1 (FDMA)

### Быстрый старт

```bash
# Сборка
cargo build --release

# Генерация GPS L1CA
cargo run --release -- presets/gps_l1ca.json

# Тройная система GPS + BeiDou + Galileo
cargo run --release -- presets/gps_bds_gal_l1.json

# Верификация сигнала (Python)
python verify_signal_enhanced.py generated_files/gps_l1ca.C8 \
    --preset presets/gps_l1ca.json
```

### Основные возможности

- 4 навигационные системы, 10 сигналов, одновременная генерация
- Парсинг эфемерид RINEX 3.04 (GPS/BDS/GAL/GLO)
- Кеплеровская пропагация орбит + ГЛОНАСС RK4 в ПЗ-90
- Физически корректная модель АРУ (AGC)
- Ускорение AVX-512 + многопоточность (Rayon)
- Опциональный GPU offload (CUDA 12.5)
- Выход IQ8 (8-бит I/Q), совместимый с SDR-приёмниками
- Инструмент верификации с 3-страничным PDF-отчётом
- ~55 000 строк Rust, минимум зависимостей
