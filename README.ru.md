# gnss-signal-simulator-rs
[![Boosty](https://img.shields.io/badge/Boosty-Buy_me_a_coffee-FF7143?logo=boosty&logoColor=white&style=for-the-badge)](https://boosty.to/danusha/donate)

🌐 [English](README.md) · **Русский**

**Мультисистемный симулятор IF-сигналов GNSS на Rust**

Генерация реалистичных IQ-сэмплов baseband для GPS, Galileo, BeiDou и GLONASS из навигационных данных RINEX 3.04. Единственный GNSS signal simulator, написанный на Rust — zero-copy, memory-safe, с AVX-512 и опциональным CUDA-ускорением.

---

## Поддерживаемые сигналы

| Система | Сигнал | Частота | Modulation | Code Length | Verified z-score |
|---------|--------|---------|------------|-------------|-----------------|
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

> z-score = (correlation peak − mean) / std, threshold ≥ 30. Значения выше 50 указывают на надёжную детекцию.

## Ключевые возможности

- **4 GNSS-созвездия, 12 верифицированных сигналов** — одновременная мультисистемная генерация в один IQ-файл
- **Парсинг эфемерид RINEX 3.04** — GPS (7-line), BeiDou (7-line с BDT correction), Galileo (7-line), GLONASS (3-line с km→m)
- **Кеплеровская пропагация орбит** — алгоритм GPS ICD-200; GLONASS RK4 в PZ-90
- **Алгоритм единой эпохи** — заставляет все спутники использовать эфемериды с одного эпохи времени
- **Физически корректный AGC** — RMS-based контроллер, Gaussian I/Q distribution, ~0% clipping
- **AVX-512 ускорение** — авто-детект на рантайме, SIMD-оптимизированные горячие пути
- **Опциональный CUDA GPU offload** — массовое вычисление PRN×carrier×amplitude на миллисекунду
- **Многопоточность** — параллельная обработка спутников на базе Rayon
- **IQ8 output** — signed 8-bit interleaved I/Q сэмплы, совместимо с SDR-приёмниками (GNURadio, SDR#)
- **Python verification tool** — 3-страничный PDF-отчёт с PSD, acquisition, correlation analysis
- **Минимум зависимостей** — только `rand`, `rayon`, `wide`, `serde_json` (+ опционально `cudarc`)
- **~55 000 строк Rust** — полный GNSS stack, реализованный с нуля

## Быстрый старт

### Требования

- Stable Rust (edition 2021) и `cargo`
- Рекомендуется: x86_64 CPU (AVX-512 используется автоматически при наличии)
- Опционально: NVIDIA driver + CUDA 12.5 для GPU-ускорения

### Сборка и запуск

```bash
# Optimized release build
cargo build --release

# Генерация GPS L1CA (простейший пример)
cargo run --release -- presets/gps_l1ca.json

# Тройная система GPS + BeiDou + Galileo
cargo run --release -- presets/gps_bds_gal_l1.json

# Quad-система GPS + BeiDou + Galileo + GLONASS (46.5 MHz)
cargo run --release -- presets/quad_l1g1.json

# Подробный вывод
cargo run --release -- -v presets/gps_l1ca.json
```

Вывод: бинарный IQ-файл (например, `generated_files/gps_l1ca.C8`).

### Сборка с GPU (опционально)

```bash
cargo build --release --features gpu
cargo run --release --features gpu -- presets/gps_l1ca.json
```

Требует CUDA 12.5 с NVRTC. Автоматический fallback на CPU при отсутствии CUDA.

## Пресеты

Готовые JSON-конфиги в `presets/`:

| Preset | Системы | Sample Rate |
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

В репозитории 44 пресета. Большинство используют Montana location, длительность 10s, elevation mask 5°, полный RINEX; пресеты `moscow_*` и `usa_*` — длительные сценарии для реальных приёмников (60–300s) в соответствующих локациях.

Свои пресеты: скопировать любой JSON, отредактировать receiver position (LLA), sample rate, путь к RINEX, длительность и выбор сигналов.

## Верификация сигнала

Прилагаемый Python-инструмент `verify_signal_enhanced.py` генерирует 3-страничный PDF-отчёт.

### Использование

```bash
# Установка зависимостей
pip install numpy matplotlib

# Верификация с авто-конфигурацией из пресета
python verify_signal_enhanced.py generated_files/gps_bds_gal_l1.C8 \
    --preset presets/gps_bds_gal_l1.json

# Fast mode (поиск только по видимым в RINEX спутникам)
python verify_signal_enhanced.py generated_files/gps_l1ca.C8 \
    --preset presets/gps_l1ca.json --fast

# С кастомными параметрами
python verify_signal_enhanced.py generated_files/output.C8 \
    --sample-rate 5.0 --threshold 30 --output report.pdf
```

### Содержимое PDF-отчёта

- **Страница 1 — Signal Overview**: Welch PSD, I/Q гистограмма с Gaussian fit, диаграмма созвездия, RMS stability
- **Страница 2 — Acquisition Results**: z-score bar chart (цвет по системе), polar skyplot (expected vs detected), CN0 estimation, Doppler accuracy
- **Страница 3 — Correlation Analysis**: zoomed correlation peaks по системам, 2D Doppler×Code heatmaps

### Результаты верификации

**Triple-system (GPS + BeiDou + Galileo, 5 MHz, 10s, Chicago):**

| Система | RINEX Visible | Detected | z-score Range |
|---------|--------------|----------|---------------|
| GPS L1CA | 11 | 11 | 58–88 |
| BeiDou B1C | 8 | 6 | 574–955 |
| Galileo E1 | 10 | 6 | 364–425 |
| **Total** | **29** | **23** | **100% сгенерированных** |

**Galileo multi-band (Montana, по 10s):**

| Сигнал | Sample Rate | Detected | z-score Range |
|--------|-------------|----------|---------------|
| E5a | 21 MHz | 7/7 | 434–502 |
| E5b | 21 MHz | 7/7 | 449–514 |
| E6 | 11 MHz | 7/7 | 270–299 |

**Single-system после июньских проходов 2026 (честные CBOC/QMBOC + NH-устойчивый верификатор):**

| Сигнал | Sample Rate | RINEX Visible | Detected | z-score Range |
|--------|-------------|--------------|----------|---------------|
| Galileo E1 (CBOC) | 5 MHz | 10 | 10/10 | 63–103 |
| BeiDou B1C (QMBOC, акквизиция по пилоту) | 5 MHz | 12 | 12/12 | 132–247 |
| BeiDou B1I | 5 MHz | 12 | 12/12 | 65–96 |
| GPS L5 | 21 MHz | 10 | 10/10 | 44–59 |
| Galileo E5a | 21 MHz | 10 | 10/10 | 48–64 |
| BeiDou B2a | 21 MHz | 12 | 12/12 | 48–72 |

> PRN-коды сверены бит-в-бит с независимыми эталонами: B1I — gnss-sdr (63/63 PRN); B1C data/pilot/secondary, E5a I/Q, B2a data/pilot и все NH/вторичные коды — PocketSDR. GPS L5 XA/XB у нас ICD-буквальный (свободный регистр XB; mod-10230 свёртка эталонов расходится в последних `adv` чипах, эффект <0.12 дБ).
>
> Верификатор использует линейную корреляцию двойным окном (устойчив к флипам NH/вторичных чипов на эпохах кода) с компенсацией кодового допплера — прежний результат 0/10 по GPS L5 был артефактом верификатора, что подтверждено чистым дампом с идеальной корреляцией каждого периода и бит-точной NH-разводкой.

**GLONASS G1 (10 MHz, Montana, 10s):**

| Generated | Detected | z-score Range |
|-----------|----------|---------------|
| 7 SVs | 5 | 54–56 |

## Архитектура

```
src/
├── main.rs              # Точка входа — читает JSON-пресет, запускает генерацию
├── lib.rs               # Публичные экспорты модулей
├── ifdatagen.rs         # Core IF data generation (AGC, satellite signals, noise)
├── sat_if_signal.rs     # Per-satellite IF sample generation (PRN codes, BOC)
├── coordinate.rs        # Кеплеровский пропагатор, конверсии ECEF/LLA/ENU
├── json_interpreter.rs  # RINEX 3.04 парсер, JSON-пресет парсер
├── gnsstime.rs          # Конверсии time-систем GPS/BDT/GST/GLONASS
├── types.rs             # Core-типы: GpsEphemeris, BeiDouEphemeris, KinematicInfo
├── constants.rs         # Константы WGS84, PZ-90, CGCS2000
├── prngenerate.rs       # Генерация PRN-кодов (Gold, Weil/Legendre, memory codes)
├── memory_code_e1.rs    # Galileo E1 memory codes (4092 чипа)
├── memory_code_e6.rs    # Galileo E6 memory codes (5115 чипов)
├── fastmath.rs          # Оптимизированные приближения sin/cos/atan2
├── complex_number.rs    # Lightweight комплексная арифметика
├── satellite_param.rs   # Параметры спутников и конфигурация сигнала
├── satellite_signal.rs  # Signal-level обработка спутника
├── almanac.rs           # Парсинг альманаха и type detection
├── trajectory.rs        # Генерация траектории приёмника
├── *navbit.rs           # Генераторы навигационных сообщений (GPS LNAV, Galileo I/NAV,
│                        #   F/NAV, BeiDou BCNav1/2/3, GLONASS G-NAV и т.д.)
├── nav_decode.rs        # Обратные декодеры + ICD parity/CRC верификаторы nav-сообщений
├── gps_pilot/mod.rs     # Чистый минимальный phase-continuous генератор GPS L1CA
└── bin/
    ├── spectrum_analyzer.rs  # IF spectrum analysis (PSD, peaks, CSV export)
    ├── nav_diagnostics.rs    # Nav-сообщения encode→decode→compare с RINEX (ICD verify)
    ├── decode_iq.rs          # Декодер GPS L1CA IQ8 — acquisition, tracking, LNAV decode
    ├── gnss_pilot.rs         # Чистый генератор L1-band GPS L1CA + Galileo E1 + BeiDou B1C
    ├── gps_pilot.rs          # Чистый минимальный генератор GPS L1CA
    ├── nav_test.rs           # Юнит-тесты навигационных битов
    ├── bench.rs              # Бенчмарки CPU/AVX-512/GPU
    └── extreme_bench.rs      # Стресс-бенчмарки
```

### Данные RINEX

- `Rinex_Data/rinex_v3_20251560000.rnx` — 2025-06-05, GPS/BDS/GAL
- `Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx` — 2025-06-05, полный merged GPS/BDS/GAL/GLO

## Формат вывода

**IQ8**: signed 8-bit interleaved I/Q сэмплы (расширение `.C8`)

```
[I₀, Q₀, I₁, Q₁, I₂, Q₂, ...]   — каждый сэмпл — int8 (-128..+127)
```

- AGC target RMS: 0.25 (quantized: ~31.75 в единицах int8)
- Совместимо с GNURadio (`file_source` → `char_to_float`), SDR# и кастомными SDR-приёмниками
- Размер файла: `2 × sample_rate × duration` байт (например, 5 MHz × 10s = 100 MB)

## Spectrum Analyzer

Встроенный инструмент анализа IF-спектра:

```bash
cargo run --release --bin spectrum_analyzer -- generated_files/gps_l1ca.C8 iq8 5000000 0 --csv
```

Выводит PSD в терминал и опционально в `spectrum.csv`.

## Разработка

```bash
cargo check          # Быстрая syntax/type проверка
cargo test           # Запуск юнит-тестов
cargo fmt --all      # Форматирование кода
cargo clippy         # Линтинг
```

Подробный статус верификации в `docs/verification_report.md` и
`docs/gnss_icd_conformance_report.md`.

## Диагностика проблем

| Проблема | Решение |
|---------|----------|
| "Configuration file not found" | Использовать пресеты из `presets/` или отредактировать сгенерированный `config.json` |
| "RINEX file not found" | Проверить относительные пути в пресете, убедиться что файлы есть в `Rinex_Data/` |
| CUDA not found | Установить CUDA 12.5, проверить NVRTC в PATH/LD_LIBRARY_PATH |
| Медленная генерация | Использовать `--release`, добавить `--features gpu` |
| 0 видимых спутников | Проверить, что дата RINEX совпадает с временем в пресете, верифицировать координаты приёмника |

## Лицензия

Информация о лицензии в исходных файлах.

---

## Description (English)

**Multi-constellation GNSS IF signal simulator written in Rust**

Generate realistic IQ baseband samples for GPS, Galileo, BeiDou, and GLONASS from RINEX 3.04 navigation data.

### Supported Signals

- **GPS**: L1CA, L5, L2C, L1C
- **Galileo**: E1 (CBOC), E5a, E5b, E6
- **BeiDou**: B1C (BOC+QMBOC), B1I, B2a
- **GLONASS**: G1 (FDMA)

### Quick Start

```bash
# Build
cargo build --release

# Generate GPS L1CA
cargo run --release -- presets/gps_l1ca.json

# Triple-system GPS + BeiDou + Galileo
cargo run --release -- presets/gps_bds_gal_l1.json

# Signal verification (Python)
python verify_signal_enhanced.py generated_files/gps_l1ca.C8 \
    --preset presets/gps_l1ca.json
```

### Main Features

- 4 navigation systems, 10 signals, simultaneous generation
- RINEX 3.04 ephemeris parsing (GPS/BDS/GAL/GLO)
- Keplerian orbit propagation + GLONASS RK4 in PZ-90
- Physically correct AGC model
- AVX-512 acceleration + multithreading (Rayon)
- Optional GPU offload (CUDA 12.5)
- IQ8 output (8-bit I/Q) compatible with SDR receivers
- Verification tool with 3-page PDF report
- ~55 000 lines of Rust, minimal dependencies
