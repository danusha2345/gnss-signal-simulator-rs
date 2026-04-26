# gnss-signal-simulator-rs

**Multi-constellation GNSS IF signal simulator written in Rust**

Generate realistic IQ baseband samples for GPS, Galileo, BeiDou, and GLONASS from RINEX 3.04 navigation data. The only GNSS signal simulator written in Rust — zero-copy, memory-safe, with AVX-512 and optional CUDA acceleration.

---

## Supported Signals

| System | Signal | Frequency | Modulation | Code Length | Verified z-score |
|--------|--------|-----------|------------|-------------|-----------------|
| GPS | L1CA | 1575.42 MHz | BPSK(1) | 1023 | 58–88 |
| GPS | L5 | 1176.45 MHz | BPSK(10) | 10230 | Verified |
| GPS | L2C | 1227.60 MHz | BPSK(1) | 10230 | Verified |
| GPS | L1C | 1575.42 MHz | BOC(1,1) | 10230 | Verified |
| Galileo | E1 | 1575.42 MHz | CBOC(6,1,1/11) | 4092 | 364–425 |
| Galileo | E5a | 1176.45 MHz | BPSK(10) | 10230 | 434–502 |
| Galileo | E5b | 1207.14 MHz | BPSK(10) | 10230 | 449–514 |
| Galileo | E6 | 1278.75 MHz | BPSK(5) | 5115 | 270–299 |
| BeiDou | B1C | 1575.42 MHz | BOC(1,1)+QMBOC | 10230 | 574–955 |
| GLONASS | G1 | 1602+k×0.5625 MHz | BPSK(0.5) FDMA | 511 | 54–56 |

> z-score = (correlation peak − mean) / std, threshold ≥ 30. Values above 50 indicate reliable detection.

## Key Features

- **4 GNSS constellations, 10 signals** — simultaneous multi-system generation into a single IQ file
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
- **~38,000 lines of Rust** — full GNSS stack implemented from scratch

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
| `quad_l1g1.json` | GPS+BDS+GAL+GLO L1/G1 | 46.5 MHz |

All 36 presets use Montana location, 10s duration, elevation mask 5°, full RINEX.

Custom presets: copy any JSON, edit receiver position (LLA), sample rate, RINEX path, duration, and signal selection.

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
└── bin/
    ├── spectrum_analyzer.rs  # IF spectrum analysis (PSD, peaks, CSV export)
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
- **BeiDou**: B1C (BOC+QMBOC)
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
- ~38 000 строк Rust, минимум зависимостей
