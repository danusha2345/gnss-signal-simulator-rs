# GNSS ICD conformance report

## Продолжение полного пайплайна 2026-10-17

Актуальные итоги и ограничения: [трекер](PIPELINE_TRACKER.md),
[контракт](GENERATION_CONTRACT.md), [матрица независимого приёма](RECEIVER_MATRIX.md).
Исправлены GNAV, TMBOC, Galileo source/clock/BGD, переходы времени/траектории;
добавлены динамическое обновление каналов, IQ/TXT/manifest и проверка SHA-256.
187 тестов прошли, 6 пропущены; тяжёлый code-phase дополнительно прошёл.
Все targets проходят строгий Clippy, общий rustfmt чистый.
Прежние разделы ниже — исторические результаты соответствующих этапов,
включая прежнее число каналов и открытые на тот момент пункты.



## F/NAV E5a: продолжение ревизии 2026-10-17

База предыдущего этапа закоммичена как `24f8580`.

- Исправлена структура: 214 payload + 24 CRC24Q + 6 tail → 488 FEC symbols,
  затем 12 sync = 500 symbols / 10 с. CRC вычисляется по 214 битам без
  округления до 32-битной границы.
- Страницы 1–4 упакованы по ICD MSB-first: SVID/IODnav, clock/orbit/SISA/BGD,
  E5a HS/DVS, WN/TOW, NeQuick, UTC/GGTO. Страницы 5–6 содержат связанный
  альманах, включая поле Omega0 второго спутника, разделённое между страницами.
- F/NAV получает `IonoNequick` через общий Galileo setter; преобразование GPS
  Klobuchar в NeQuick удалено. UTC использует часы и недели GST.
- TOW привязан к началу 10-секундной страницы; нормализуются отрицательные
  миллисекунды и переход недели. Малый выходной буфер/невалидный SVID
  возвращают ошибку, а не повреждают память. Без эфемериды передаётся dummy.
- `tests/fnav_decode.rs`: независимый Viterbi (обратное направление регистра,
  octal 171/133), полиномиальное деление CRC, поля всех страниц, 1200-секундный
  цикл альманаха, week rollover, ошибочный аргумент, одиночная ошибка канала.
  Фиксированный 500-символьный вектор и отдельный Python reference encoder —
  `tests/fixtures/fnav_page1_symbols.txt`, `tests/fixtures/fnav_reference.py`.

Первый IQ-прогон выявил обратное направление интерливинга: тестовый декодер
повторял ту же перестановку и маскировал ошибку. После исправления теста по
реальному GNSS-SDR старый код провалил все пять тестов. Исправлено направление:
запись столбцами в матрицу 8×61, передача строками. Добавленный фиксированный
вектор предотвращает повторение такой взаимной компенсации encoder/decoder.
Первый неуспешный прогон сохранён отдельно, он не считается подтверждением.

Источники: [структура E5a](https://raw.githubusercontent.com/gnss-sdr/gnss-sdr/v0.0.20/src/core/system_parameters/Galileo_E5a.h),
[поля F/NAV](https://raw.githubusercontent.com/gnss-sdr/gnss-sdr/v0.0.20/src/core/system_parameters/Galileo_FNAV.h),
[независимый приёмник: интерливинг/FEC](https://raw.githubusercontent.com/gnss-sdr/gnss-sdr/v0.0.20/src/algorithms/telemetry_decoder/gnuradio_blocks/galileo_telemetry_decoder_gs.cc).

Полный unit/integration проход: 170 passed, 6 ignored. GPS+Galileo L1/E1
проверяется отдельным побайтовым golden; NAV-04 (источник RINEX и BGD)
остаётся самостоятельной открытой задачей.

Программный контроль исправленного E5a: 120 с, 21 MS/s, IQ8, GNSS-SDR 0.0.20.
57 CRC correct / 0 errors; первый фикс 82,4 с, до 7 спутников, 373 эпохи PVT.
Горизонтальная ошибка: median 1,68 м, p95 3,46 м. Коррекции ионосферы и
тропосферы приёмника отключены; медианное отклонение высоты +22,60 м.
Из IQ подтверждены NeQuick 140.5 / 0.1484375 / 0.011474609375, TLS=18,
t0t=259200 с, GST WN=1345. Логи и XML:
`/home/danik/storage/gnss_rust-fnav-review-20261017/fixed/`.
Файл и настройки на USB: `gnss_rust/fnav_review/USA_E5A.C8`, рядом TXT/JSON
и verification manifest. Приём аппаратным устройством не проверялся.

## Ревизия GPS/Galileo от 2026-10-17

Область: RINEX → навигационные слова → параметры спутников → IQ8,
включая окружность 100 м при 10 км/ч. Проверка программная; приём
исправленного сигнала дроном пока не подтверждён.

| Исправление | Независимая проверка |
| --- | --- |
| `GAL`/`GAUT` передаются из RINEX через `CNavData`/`IFDataGen` в I/NAV; NeQuick не подменяется Klobuchar. `tot` Galileo в часах, недели в GST; порядок UTC/LEAP SECONDS не теряет поля | FEC/Viterbi/CRC и поля Word 5/6: ai0=140.5, ai1=0.1484375, ai2=0.011474609375, t0t=259200 с, TLS=18 |
| Раздельные шкалы GPS URA и Galileo SISA | GPS 2 м → индекс 0; Galileo 3,12 м → 107; NAPA → 255; граничные значения |
| Релятивистская поправка по текущему E вместо устаревшего `eph.Ek`; исправлен знак и добавлена производная для Доплера | Независимое выражение −2r·v/c², численная производная, независимость от кэша, переход недели |
| В `SatelliteParamCalculator` устранено повторное добавление релятивистской поправки | Общая функция часов, существующий тест смены эфемерид; полный аудит blending пока не выполнен |
| GPS использует μ=3.986005e14 для среднего движения, Galileo — μ=3.986004418e14 | Восстановление μ из разобранных эфемерид обеих систем |
| E5b DVS берётся из бита RINEX 6; WN/TOW Word 0/5 учитывают переход недели | Побитовые независимые health-флаги и FEC-декодирование граничных времён |
| Событие мощности применяется в 50 мс, не в 49 мс | Фактическая генерация длительностью 50 и 51 мс, контроль CN0 последнего отсчёта |
| Коррекция по углу места действует на мощность по умолчанию; явный положительный override сохраняется | Зенит, горизонт, отрицательная высота; отсутствие NaN |
| Стартовый AGC измеряет первый готовый блок, учитывая разные CN0 и обе компоненты шума | RMS первого IQ8 блока около 0,4; целевой уровень и дальнейшая адаптация сохранены |
| Тест фазы удаляет BPSK-переворот на каждой границе, а не только у глобального максимума | Скачок 0,5 рад не скрывается переворотом π; 24 динамических окна по 250 мс, GPS 1/13/30 и Galileo 2/29/30, начала 0/110/224/298 с, границы блоков 50 мс |

`cargo test`: **157 passed, 0 failed, 6 ignored**. Release-сборка успешна.
Остаток фазы в проверенных динамических окнах меньше 0,0002 рад.
`git diff --check` проходит. Общие fmt/Clippy пока не проходят из-за
существующего состояния проекта: 37 ошибок библиотеки, 52 при проверке
library tests. Полного форматирования посторонних файлов не выполнялось.

Программный контроль **300 с, IQ8, 4,2 МГц**, GNSS-SDR 0.0.20:

| Режим | Первый фикс | Максимум спутников | Эпохи PVT | Горизонтальная ошибка: медиана / p95 |
| --- | --- | --- | --- | --- |
| GPS + Galileo | 30,2 с | 14 | 2695 | 3,25 / 5,65 м |
| Только Galileo | 130,2 с | 7 | 1695 | 4,70 / 5,29 м |

Ошибки сравнивались с независимой аналитической окружностью ENU, R=100 м,
v=10 км/ч. Ионосферная/тропосферная коррекции приёмника отключены для
сопоставимости с прежними прогонами; медианная вертикальная ошибка
составила +14,16 м и +30,42 м соответственно. Это не проверка абсолютной
высотной точности. В реально декодированном IQ подтверждены SISA=107,
NeQuick из RINEX и UTC: TLS=18, t0t=259200 с, WNot=65 (1345 mod 256).
120-секундный контроль оказался короче холодного старта Galileo-only;
его байты совпали с первыми 120 с полного файла. Отсутствие фикса в
коротком прогоне не было доказательством регрессии.

Артефакт: `/run/media/danik/external_ssd_2tb/gnss_rust/review_20261017/USA_TEST.C8`,
2 520 000 000 байт, рядом `USA_TEST.TXT`, JSON и verification manifest.
SHA-256: `8563c87d0a3ba8446c96c456370d13ca96c56ffdcfd42b74e8e54d17a09f2842`.
Логи и исходные результаты: `/home/danik/storage/gnss_rust-review-20261017/`.
SD-карта не изменялась; commit/push не выполнялись.

Ограничения: задержка Galileo в IF пока моделируется Klobuchar GPS при
наличии его коэффициентов; правильные NeQuick-параметры в сообщении не
означают реализацию NeQuick G propagation. F/NAV, остальные сигналы, GPU
и полный алгоритм смешивания эфемерид этим проходом не сертифицированы.
Отдельно замечена шкала GPS URA для Galileo в `var_uraeph` GNSS-SDR 0.0.20;
её влияние на конкретный фикс не установлено. Число спутников в программном
решении не доказывает приём дроном или причину прежнего исчезновения фикса.

Источники: [RTKLIB: шкалы и орбиты](https://raw.githubusercontent.com/tomojitakasu/RTKLIB/rtklib_2.4.3/src/ephemeris.c),
[ESA: часы](https://gssc.esa.int/navipedia/index.php/Relativistic_Clock_Correction),
[GNSS-SDR: поля I/NAV](https://raw.githubusercontent.com/gnss-sdr/gnss-sdr/v0.0.20/src/core/system_parameters/Galileo_INAV.h),
[GNSS-SDR 0.0.20: дисперсия URA](https://raw.githubusercontent.com/gnss-sdr/gnss-sdr/v0.0.20/src/algorithms/libs/rtklib/rtklib_ephemeris.cc).


Date checked: 2026-04-26; updated 2026-06-02 after the May 31 – June 1
audit and ICD-conformance pass.

This report tracks the current bit, carrier, modulation, and IF-output
conformance checks against the public ICD/source set listed below. The tests are
small, hand-derived fixtures plus encode→decode→compare round-trips against the
RINEX ephemeris. Fixed mismatches are marked in the findings; any remaining spec
gaps are recorded explicitly instead of being hidden by sanity checks.

> **2026-06-02 status.** Full `cargo test` passes: 65 library unit tests and all
> integration suites green, 0 failures. The previously open Galileo I/NAV
> Viterbi/CRC decode finding (`F-GAL-INAV-VITERBI-CRC`) is resolved — the test
> now passes in the default suite and is no longer `#[ignore]`. A complete set of
> ephemeris round-trip decoders (GPS LNAV/CNAV/CNAV-2, Galileo I/NAV & F/NAV,
> BeiDou D1/D2 & B-CNAV1/2/3, GLONASS G-NAV) was added during the pass; see
> "Ephemeris round-trip coverage" and "June 2026 conformance pass" below.

## Primary ICD sources

| Area | Source | URL |
| --- | --- | --- |
| RINEX NAV records | RINEX 4.02, sections 5.4 and appendix A.8.3 | https://files.igs.org/pub/data/format/rinex_4.02.pdf |
| GPS LNAV/CNAV/L2C | IS-GPS-200N, sections 3.3 and 20.3 | https://www.navcen.uscg.gov/sites/default/files/pdf/gps/IS-GPS-200N.pdf |
| GPS L5 CNAV | IS-GPS-705J, sections 3.2.2, 3.2.3, 3.3 | https://www.navcen.uscg.gov/sites/default/files/pdf/gps/IS-GPS-705J.pdf |
| GPS L1C CNAV-2 | IS-GPS-800J, sections 3.2.3 and 3.5 | https://www.navcen.uscg.gov/sites/default/files/pdf/gps/IS-GPS-800J.pdf |
| Galileo OS SIS | Galileo OS SIS ICD v2.1, sections 2.3, 4.1, 4.3, 5.1 | https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.1.pdf |
| BeiDou ICD family | BeiDou official document index | https://en.beidou.gov.cn/SYSTEMS/Officialdocument/ |
| BeiDou B1I | BDS-SIS-ICD-B1I-3.0 | https://www.beidou.gov.cn/xt/gfxz/201902/P020190227593621142475.pdf |
| BeiDou B1C | BDS-SIS-ICD-B1C-1.0 | https://en.beidou.gov.cn/SYSTEMS/ICD/201806/P020180608519640359959.pdf |
| BeiDou B2a | BDS-SIS-ICD-B2a-1.0 | https://www.beidou.gov.cn/xt/gfxz/201812/P020181227529449178798.pdf |
| GLONASS FDMA | ICD GLONASS 5.1 English public mirror | https://kb.unavco.org/article/english-version-of-icd-glonass-version-5-1-727.html |
| SignalSim reference implementation | Local upstream tables and bit packing reference | `/home/danik/Projects/SignalSim/src/CNav2Bit.cpp`, `/home/danik/Projects/SignalSim/src/BCNav1Bit.cpp`, `/home/danik/Projects/SignalSim/src/BCNavBit.cpp` |

## Conformance matrix

| Constellation | Signal / nav type | ICD section/table | Checked fields | Test / fixture | Result |
| --- | --- | --- | --- | --- | --- |
| GPS | L1 C/A LNAV | IS-GPS-200N 20.3.3.1-20.3.3.2 | TLM preamble `0x8b`, HOW next TOW, subframe ID | `gps_nav_headers_have_bit_exact_icd_sync_fields` | pass fixture |
| GPS | L2C CNAV | IS-GPS-200N 3.3.3, CNAV message layout | raw preamble/SVID/message-type headers for types 10 and 11, encoded binary output | `gps_nav_headers_have_bit_exact_icd_sync_fields` | pass fixture |
| GPS | L5 CNAV | IS-GPS-705J 3.2.2-3.3.3 | encoded 600-symbol frame, convolution/interleaver prefix and CRC/FEC suffix vectors | `gps_nav_headers_have_bit_exact_icd_sync_fields` | pass fixture |
| GPS | L1C CNAV-2 | IS-GPS-800J 3.2.3, 3.5 | TOI BCH(51,8), CRC24Q, SignalSim/ICD LDPC generator tables, subframe 2/3 split, 38x46 interleaver, ephemeris/UTC/ISC field head vectors | `gps_nav_headers_have_bit_exact_icd_sync_fields` | pass fixture, remaining subframe 3 page coverage gap |
| Galileo | E1/E5b I/NAV | Galileo OS SIS ICD 4.3 | even/odd page sync pattern `0101100000` | `galileo_nav_sync_words_are_bit_exact` | pass fixture |
| Galileo | E5a F/NAV | Galileo OS SIS ICD 4.2/4.3 | 12-bit sync pattern `0xb70` | `galileo_nav_sync_words_are_bit_exact` | pass fixture |
| BeiDou | B1I/B2I/B3I D1/D2 | BDS-SIS-ICD-B1I-3.0 section 5 | D1/D2 11-bit preamble `0x712` | `beidou_nav_headers_are_bit_exact_for_supported_message_families` | pass fixture |
| BeiDou | B1C BCNAV1 | BDS-SIS-ICD-B1C-1.0 | BCH-coded SVID header for SVID 38, 1200/528-symbol subframe 2/3 deinterleave, LDPC(200,100)/LDPC(88,44) parity vectors | `beidou_nav_headers_are_bit_exact_for_supported_message_families` | pass fixture |
| BeiDou | B2a BCNAV2 | BDS-SIS-ICD-B2a-1.0 | 24-bit preamble `0xe24de8` | `beidou_nav_headers_are_bit_exact_for_supported_message_families` | pass fixture |
| BeiDou | B2b/B3I BCNAV3 | BeiDou ICD family / RINEX 4.02 A.8.3 | 16-bit preamble `0xeb90`, PRN, reserved header bits | `beidou_nav_headers_are_bit_exact_for_supported_message_families` | pass fixture |
| GLONASS | GNav FDMA | GLONASS ICD 5.1 | idle bit, zero padding, 30-bit time marker `0x3e375096` | `glonass_gnav_time_marker_and_string_padding_are_bit_exact` | pass fixture |
| All implemented | Carrier center frequencies | ICD frequency-plan sections and local signal constants | GPS/BDS/Galileo matrix rows, GLONASS G1/G2/G3 | `signal_center_frequency_matrix_matches_icd_fixtures` | pass fixture |
| IF output | IQ8 | local output contract plus SDR signed I/Q convention | signed interleaved byte count and nonzero short file | `short_iq8_generation_writes_interleaved_iq_bytes` | pass fixture |
| IF output | IQ4 | local output contract | one packed byte per sample, nonzero short file | `short_iq4_generation_writes_one_packed_byte_per_sample` | pass fixture |

## Findings

### F-GLO-G3-FREQ

- Severity: decode-breaking for generated GLONASS G3/L3OC IF output.
- Status: fixed after report review on 2026-04-26.
- ICD source: RINEX 4.02 section 5.4.4 and appendix A.8.3 include GLONASS
  L3OC/CDMA message records; the repo also declares `FREQ_GLO_G3 =
  1_202_025_000.0`.
- Expected value or layout: GLONASS G3 generation should use the G3 center
  frequency instead of zero when deriving IF offset.
- Previous repo output: `constants::SIGNAL_CENTER_FREQ[Glonass][2]` was `0.0`;
  `ifdatagen.rs` had a private matrix with the same zero slot, so G3 IF offset
  could be computed from zero.
- Current repo output: both frequency matrices now use `FREQ_GLO_G3` for
  GLONASS signal slot 2.
- Affected symbols: `src/constants.rs::SIGNAL_CENTER_FREQ`,
  `src/ifdatagen.rs::SIGNAL_CENTER_FREQ`, GLONASS branch in
  `IFDataGen::create_satellite_signals`.
- Reproducer: `cargo test --test icd_conformance_fixtures
  signal_center_frequency_matrix_matches_icd_fixtures`.

### F-GPS-CNAV2-PARTIAL

- Severity: unsupported/spec-drift for full GPS L1C CNAV-2 bit conformance.
- Status: LDPC/framing and core field packing fixed on 2026-04-26; subframe-2
  ephemeris packing now confirmed by a round-trip decoder added on 2026-06-01
  (commit `d354887`, test `cnav2_eph_round_trips_via_subframe2_decoder`).
  Residual subframe-3 variable-page coverage gap remains (see below).
- ICD source: IS-GPS-800J sections 3.2.3.1-3.2.3.5 and 3.5.
- Expected value or layout: full CNAV-2 CRC, LDPC, interleaving, and message
  field mapping should match the L1C message definition.
- Previous repo output: `src/cnav2bit.rs::CNav2Bit::get_frame_data` emitted a
  selected pseudo-subframe with `0x8b` preamble/SVID/TOW fields, which is not
  the L1C subframe 1 format. Dead alternate CNAV-2 methods also contained
  placeholder payload branches and a placeholder parity fill. The first report
  review improved framing but still used a local systematic LDPC placeholder.
- Current repo output: `get_frame_data` now emits a full 1800-symbol L1C frame:
  52-symbol TOI BCH(51,8) subframe 1, CRC24Q-protected subframe 2 and
  subframe 3 streams, SignalSim/IS-GPS-800 LDPC generator matrices, and the
  38x46 block interleaver. Subframe 2 ephemeris/clock/ISC packing and
  subframe 3 UTC/ionosphere/ISC packing now follow the upstream CNav2Bit
  reference instead of dense local placeholder words.
- Remaining gap: subframe 3 currently transmits the ionosphere/UTC page only.
  Reduced almanac and MIDI almanac page scheduling are still not implemented,
  so the report does not claim complete CNAV-2 variable-page coverage.
- Affected symbols: `src/cnav2bit.rs::CNav2Bit::get_frame_data`,
  `src/cnav2_ldpc_tables.rs`, `src/navbit.rs::NavBit::crc24q_encode`.
- Reproducer: `cargo test --test icd_conformance_fixtures
  gps_nav_headers_have_bit_exact_icd_sync_fields`.

### F-B1C-LDPC-INTERLEAVE-RISK

- Severity: spec-drift risk for full BeiDou B1C BCNAV1 conformance.
- Status: fixed on 2026-04-26.
- ICD source: BDS-SIS-ICD-B1C-1.0.
- Expected value or layout: complete B1C subframe 2/3 LDPC and interleaving
  should match the ICD symbol layout.
- Previous repo output: the fixture only verified the BCH-coded subframe 1 SVID
  header. The Rust B1C generator matrix strings were also truncated
  (`10000` and `1936` required matrix symbols were reduced to `1500` and `44`),
  and `src/ldpc.rs` used a polynomial GF(64) multiply that did not match the
  BeiDou/SignalSim exponent-table representation.
- Current repo output: the fixture now also deinterleaves the post-header B1C
  payload into the expected 1200-symbol subframe 2 and 528-symbol subframe 3
  blocks and checks fixed LDPC parity symbol vectors for both LDPC(200,100) and
  LDPC(88,44). `src/ldpc.rs` now uses the same `e2v/v2e` GF(64) table method as
  the SignalSim BCNavBit reference.
- Remaining gap: no known B1C LDPC/interleaver gap remains in this pass.
- Affected symbols: `src/bcnav1bit.rs::BCNav1Bit::get_frame_data`,
  `src/ldpc.rs`.
- Reproducer: `cargo test --test icd_conformance_fixtures
  beidou_nav_headers_are_bit_exact_for_supported_message_families`.

### F-GAL-INAV-VITERBI-CRC

- Severity: decode/regression gap for Galileo I/NAV end-to-end verification.
- Status: **resolved on 2026-06-01** (commit `3c565b1`). The fault was in the
  test harness, not the encoder: the Viterbi/deinterleaver fixture mis-ordered
  the even/odd page halves and tail handling, so it reported `0/15 pages CRC OK`
  against a correctly generated stream. The encoder was confirmed correct.
- ICD source: Galileo OS SIS ICD v2.1 sections 4.1, 4.3, and 5.1.
- Expected value or layout: the Viterbi/deinterleaver test should decode
  generated I/NAV pages and verify CRC24Q against the reference page data.
- Previous observed output: `cargo test -- --ignored
  test_galileo_inav_viterbi_decode` failed in `tests/signal_quality.rs` with
  `Summary: 0/15 pages CRC OK` and `DATA MISMATCH even=false odd=true` lines.
- Current observed output: `test_galileo_inav_viterbi_decode` and
  `test_galileo_inav_all_svs_crc` both pass and are no longer `#[ignore]`; they
  run in the default suite (`cargo test --test signal_quality`).
- Affected symbols: `tests/signal_quality.rs` (harness), `src/inavbit.rs`.
- Reproducer: `cargo test --test signal_quality galileo_inav`.

## June 2026 conformance pass (May 31 – June 1)

After the 2026-04-26 sync/header fixtures, a follow-up audit and ICD pass
reworked the ephemeris encoders themselves and added decoders to verify them by
round-trip against the RINEX data. The findings below are all fixed and covered
by the round-trip tests in the next section.

### F-FNAV-E5A-EXPONENT (fixed, `e9b3b5b`)

- The private `unscale_int`/`unscale_uint` helpers in the F/NAV (E5a) encoder
  inverted the scale-factor exponent, which silently zeroed the entire E5a
  ephemeris payload. Galileo OS SIS ICD F/NAV field scaling restored; covered by
  `fnav_eph_round_trips_via_decoder`.

### F-D1D2-EXPONENT (fixed, `6c9127c`)

- The same inverted-exponent bug in `unscale_double` zeroed the entire BeiDou
  legacy D1/D2 (B1I/B2I/B3I) ephemeris. Fixed per BDS-SIS-ICD-B1I-3.0 field
  scaling; covered by `d1_eph_round_trips_via_decoder`.

### F-GLO-G3-L3OC-CODE (fixed, `d3f1658`)

- The GLONASS G3/L3OC ranging code was a degenerate placeholder. Replaced with
  the official ICD truncated Kasami sequence. This complements the earlier
  `F-GLO-G3-FREQ` carrier-frequency fix.

### F-GLO-GNAV-EPHEMERIS (fixed, `2e978de`)

- The GLONASS G-NAV ephemeris encoder was rewritten per the official ICD
  (GLONASS ICD 5.1), correcting field widths, scale factors, and units
  (audit items H15/H16/H17).

### F-BCNAV23-EPHEMERIS (fixed, `c17eb24`, `b4905d9`, `9594bd1`)

- BeiDou B-CNAV2 (B2a) and B-CNAV3 (B2b) ephemeris were a stub / mis-packed.
  Both now encode via the verified B-CNAV1 encoder; B-CNAV3 MT10 framing
  (`append_word`, IODE-in-EphI) corrected. Covered by
  `bcnav2_ephemeris_round_trips_via_b1c_decoder`,
  `bcnav3_ephemeris_round_trips_via_b1c_decoder`, and
  `bcnav3_mt10_frame_round_trips`.

### F-SCALE-MISC (fixed, `953c1c8`, `cdedbc5`, `ef2d765`, `24095ff`)

- F/NAV almanac `sqrtA` scale, BDS D2 `af2` width, and B-CNAV1 `Δn0-dot`/`TGD`
  scaling corrected; GPS L5 CNAV now reuses the verified `CNavBit` encoder; IQ4
  quantization fixed (was collapsing to ~1 effective bit); low-severity audit
  fixes (i32 week overflow, `extract_bits` UB shift, `NavBit` CRC empty-input
  panic).

## Ephemeris round-trip coverage (2026-06-02)

Each encoder now has a decoder that reverses it; the round-trip re-derives the
ephemeris from the encoded bitstream and compares against the source RINEX
values. All pass in the default `cargo test` run.

| Constellation | Nav type | Round-trip test |
| --- | --- | --- |
| GPS | LNAV | `lnavbit::tests::test_lnav_ephemeris_round_trip` |
| GPS | L2C/L5 CNAV | `nav_decode::cnav_roundtrip_tests::cnav_eph_round_trip_matches_l2c_encoder` |
| GPS | L1C CNAV-2 | `nav_decode::cnav_roundtrip_tests::cnav2_eph_round_trips_via_subframe2_decoder` |
| Galileo | I/NAV | `inavbit::tests::test_inav_ephemeris_round_trip` |
| Galileo | F/NAV (E5a) | `nav_decode::fnav_roundtrip_tests::fnav_eph_round_trips_via_decoder` |
| BeiDou | D1/D2 (B1I/B2I/B3I) | `nav_decode::d1d2_roundtrip_tests::d1_eph_round_trips_via_decoder` |
| BeiDou | B-CNAV1 (B1C) | `bcnavbit::tests::test_bcnav_ephemeris_round_trip` |
| BeiDou | B-CNAV2 (B2a) | `bcnav2bit::append_word_tests::bcnav2_ephemeris_round_trips_via_b1c_decoder` |
| BeiDou | B-CNAV3 (B2b) | `bcnav3bit::gf6_tests::bcnav3_ephemeris_round_trips_via_b1c_decoder` |

The standalone `cargo run --bin nav_diagnostics -- --rinex <file>` tool runs the
same encode→decode→compare loop across every SV in a RINEX file for manual
inspection.

## Verification commands run

Fast conformance layer:

```bash
cargo test --test icd_conformance_fixtures
cargo test --test if_output_smoke
cargo test
```

Result: passed on 2026-04-26. `cargo test` included the new fixtures and kept
the existing heavy tests ignored by their test attributes.

Acceleration and feature checks from the verification plan:

```bash
cargo check --all-targets
cargo test --features avx512 --test signal_assembly_baseline
cargo test --features gpu --test acceleration_baseline
```

Result: passed on 2026-04-26. The GPU feature result is a compile/fallback
check on this host, not proof of NVIDIA CUDA acceleration.

Short release smoke for generated files:

```bash
cargo test --release --test if_output_smoke
```

The smoke coverage uses `tests/if_output_smoke.rs` instead of checking
generated binaries into Git. It writes 8-sample IQ8 and IQ4 files under the
system temp directory and asserts deterministic byte sizes plus nonzero content.

Result: passed on 2026-04-26.

Galileo I/NAV Viterbi/CRC decode (was the open `F-GAL-INAV-VITERBI-CRC`
finding, now part of the default suite):

```bash
cargo test --test signal_quality galileo_inav
```

Result: **passed on 2026-06-02** — `test_galileo_inav_viterbi_decode` and
`test_galileo_inav_all_svs_crc` both green after the harness fix in `3c565b1`.
Only two heavy 60s/200ms integration tests remain `#[ignore]` in
`tests/signal_quality.rs` (`test_gps_lnav_end_to_end_decode`,
`test_code_phase_continuity`); these are excluded from the fast suite by design.

Ephemeris round-trip and full suite (2026-06-02):

```bash
cargo test
```

Result: passed — 65 library unit tests (including every round-trip decoder in
the coverage table above) and all integration suites green, 0 failures.

Formatting:

```bash
rustfmt src/cnav2bit.rs src/cnav2_ldpc_tables.rs src/ldpc.rs src/navbit.rs src/lib.rs src/bcnav1bit.rs tests/icd_conformance_fixtures.rs
cargo fmt --all -- --check
```

Result: touched Rust files were formatted. Full `cargo fmt --all -- --check`
still reports broader pre-existing formatting drift in unrelated binaries/tests,
so this pass did not apply a repo-wide formatting-only diff.
