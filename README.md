# GNSS IF Data Generator (Rust)

Генератор промежуточных частотных (IF) данных сигналов ГНСС (GPS/GLONASS/BeiDou/Galileo) на Rust.
Поддерживает многопоточность, кэширование горячих путей, AVX‑512 и (опционально) ускорение на GPU (CUDA).

## Быстрый старт

- Требуется: стабильный Rust (edition 2021) и `cargo`.
- Рекомендуется: x86_64 CPU. AVX‑512 используется автоматически, если доступен на вашей машине.
- Опционально (для GPU): NVIDIA драйвер + CUDA 12.5 (NVRTC), см. раздел «GPU» ниже.

Проверка сборки и тестов:

```
cargo check
cargo test
```

Оптимизированная сборка:

```
cargo build --release
```

## Запуск генератора

Основной бинарь читает JSON‑пресеты из папки `presets/` и формирует IF‑файл в `generated_files/`.

Простой запуск (GPS L1CA):

```
cargo run --release -- presets/GPS_L1_only.json
```

Полезные опции:

- `-v, --verbose` — подробные логи
- `-q, --quiet` — только ошибки
- `--help` — краткая справка

Пример:

```
cargo run --release -- -v presets/GPS_BDS_GAL_triple_system.json
```

Параллельный режим (Rayon): включён по умолчанию. Чтобы отключить принудительно:

```
GNSS_PARALLEL_MODE=false cargo run --release -- presets/GPS_BDS_GAL_triple_system.json
```

Ожидаемый результат: бинарный IF‑файл (IQ4/IQ8) по пути, указанному в пресете (`output.name`), например:
`generated_files/GPS_L1_only.C8`.

### Входные данные (RINEX/Eph)

- Эфемериды/альманахи берутся из `Rinex_Data/` и/или `EphData/`.
- В пресетах указывайте относительные пути (пример есть в готовых JSON).
- Эти каталоги read‑only в Git; вы можете добавлять свои файлы локально.

Если конфиг не найден, программа создаст пример `config.json` и предложит отредактировать.

## Ускорение: AVX‑512 и GPU (CUDA)

- AVX‑512: включается автоматически (детектируется на рантайме). Без AVX‑512 используется высокоэффективный CPU‑путь.
- GPU (опционально): оффлоад горячего участка (массовое `PRN × carrier × amp × NAV`) реализован для каждой миллисекунды.
  Включается при сборке с фичей `gpu` и наличии CUDA в системе.

Сборка и запуск с GPU:

```
cargo run --release --features gpu -- presets/GPS_L1_only.json
```

Требования к CUDA:

- CUDA 12.5 (совместимо с фичей `cudarc` = `cuda-12050`), установленный NVRTC.
- Windows: убедитесь, что `CUDA_PATH` задан и `nvrtc64_120_0.dll` видна в `PATH`.
- Linux: проверьте наличие `libnvrtc.so` в `LD_LIBRARY_PATH`.

Проверить работу GPU можно бенчмарком:

```
cargo run --release --features gpu --bin bench
```

Если CUDA недоступна, код автоматически падает обратно на AVX‑512/CPU.

## Анализ сгенерированного IF

В комплекте есть спектральный анализатор IF‑файлов:

```
cargo run --release --bin spectrum_analyzer -- <file> <iq4|iq8> [sample_rate] [center_freq]
# Пример:
cargo run --release --bin spectrum_analyzer -- generated_files/GPS_L1_only.C8 iq8 5000000 0 --csv
```

Флаг `--csv` сохраняет спектр в `spectrum.csv`.

## Полезные бинарники

- `spectrum_analyzer` — анализ IF выборок (PSD, пики).
- `nav_test` — минимальный тест L1CA навигационных бит.
- `bench` — сводные бенчмарки CPU/AVX‑512/GPU.
- `extreme_bench` — стресс‑бенч (долго и тяжело; запускать осознанно).

Запуск:

```
cargo run --release --bin nav_test
cargo run --release --bin bench
```

## Структура проекта (важные каталоги)

- `src/` — библиотека и главное приложение (`lib.rs`, `main.rs`, `ifdatagen.rs`, GNSS‑модули).
- `src/bin/` — вспомогательные бинарники: `spectrum_analyzer`, `nav_test`, бенчи.
- `presets/` — JSON‑пресеты сценариев.
- `Rinex_Data/`, `EphData/` — входные эфемериды/альманахи (RO в Git).
- `generated_files/` — крупные выходы (игнорируются Git).

## Разработка

Форматирование и линт:

```
cargo fmt --all
cargo clippy -- -D warnings
```

Тесты:

```
cargo test
```

Оптимизированная сборка (с GPU):

```
cargo build --release --features gpu
```

## Типичные проблемы

- «Configuration file not found»: используйте пресеты из `presets/` или отредактируйте созданный `config.json`.
- «RINEX/ephemeris file not found»: проверьте относительные пути в пресете и наличие файлов в `Rinex_Data/`/`EphData/`.
- CUDA не находится: установите CUDA 12.5, проверьте NVRTC (`nvrtc64_120_0.dll`/`libnvrtc.so`) и переменные окружения.
- Медленная генерация: используйте `--release`, включите `GNSS_PARALLEL_MODE=true`, при наличии — `--features gpu`.

## Лицензия

См. исходные файлы/заметки проекта. Не коммитьте крупные бинарники и секреты.
