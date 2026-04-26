# GNSS ICD conformance report

Date checked: 2026-04-26

This report tracks the current bit, carrier, modulation, and IF-output
conformance checks against the public ICD/source set listed below. The tests are
small, hand-derived fixtures. Fixed mismatches are marked in the findings; any
remaining spec gaps are recorded explicitly instead of being hidden by sanity
checks.

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
- Status: LDPC/framing and core field packing fixed on 2026-04-26; residual
  page-coverage gap remains.
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
- Status: still open after targeted ignored-test run on 2026-04-26.
- ICD source: Galileo OS SIS ICD v2.1 sections 4.1, 4.3, and 5.1.
- Expected value or layout: the ignored Viterbi/deinterleaver test should decode
  generated I/NAV pages and verify CRC24Q against the reference page data.
- Current observed output: `cargo test -- --ignored
  test_galileo_inav_viterbi_decode` fails in `tests/signal_quality.rs`. For
  Galileo E27, tail bits are reported OK, but all 15 sampled pages fail CRC.
  The output reports `Summary: 0/15 pages CRC OK`; several lines also show
  `DATA MISMATCH even=false odd=true`, which points to a generated data or
  decode ordering mismatch rather than a pure tail-bit issue.
- Affected symbols: `src/inavbit.rs`, `tests/signal_quality.rs`.
- Reproducer: `cargo test -- --ignored test_galileo_inav_viterbi_decode`.

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

Ignored heavy decode probe:

```bash
cargo test -- --ignored test_galileo_inav_viterbi_decode
```

Result: failed on 2026-04-26 as recorded in
`F-GAL-INAV-VITERBI-CRC`. The run decoded Galileo E27, reported valid tail
bits, but ended with `Summary: 0/15 pages CRC OK`; heavy 30s/60s IQ tests remain
ignored and were not added to the default fast suite.

Formatting:

```bash
rustfmt src/cnav2bit.rs src/cnav2_ldpc_tables.rs src/ldpc.rs src/navbit.rs src/lib.rs src/bcnav1bit.rs tests/icd_conformance_fixtures.rs
cargo fmt --all -- --check
```

Result: touched Rust files were formatted. Full `cargo fmt --all -- --check`
still reports broader pre-existing formatting drift in unrelated binaries/tests,
so this pass did not apply a repo-wide formatting-only diff.
