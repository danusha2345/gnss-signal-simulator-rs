# GNSS verification sources and phase matrix

Date checked: 2026-04-25

This document pins the internet sources used as the verification baseline for
parser, navigation message, spreading-code, carrier, and IF-generation work.
Prefer these sources over older comments or local notes when a behavior differs.

## Primary sources

| Area | Source | URL | Local scope |
| --- | --- | --- | --- |
| RINEX navigation/observation exchange | IGS/RTCM RINEX 4.02 | https://files.igs.org/pub/data/format/rinex_4.02.pdf | `json_interpreter.rs`, `navdata.rs`, parser fixtures |
| RINEX governance and version notes | IGS RINEX WG documents/formats | https://igs.org/wg/rinex/ | RINEX version coverage and mixed-GNSS assumptions |
| GPS L1 C/A, L2C, LNAV/CNAV | NAVCEN IS-GPS-200N | https://www.navcen.uscg.gov/sites/default/files/pdf/gps/IS-GPS-200N.pdf | `lnavbit.rs`, `cnavbit.rs`, `prngenerate.rs`, time/orbit tests |
| GPS L1C | NAVCEN IS-GPS-800J | https://www.navcen.uscg.gov/sites/default/files/pdf/gps/IS-GPS-800J.pdf | L1C code and data/pilot generation checks |
| GPS L5 | NAVCEN IS-GPS-705J | https://www.navcen.uscg.gov/sites/default/files/pdf/gps/IS-GPS-705J.pdf | `l5cnavbit.rs`, L5 data/pilot and CNAV checks |
| Galileo E1/E5/E6 OS | Galileo OS SIS ICD v2.1 | https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.1.pdf | `inavbit.rs`, `fnavbit.rs`, Galileo PRN and FEC/CRC tests |
| Galileo release page | European GNSS Service Centre OS SIS ICD v2.1 notice | https://www.gsc-europa.eu/news/galileo-open-service-signal-in-space-interface-control-document-version-21-now-available-for | Confirms current Galileo OS SIS ICD and covered topics |
| BeiDou RNSS documents | BeiDou official document index | https://en.beidou.gov.cn/SYSTEMS/Officialdocument/ | Source index for B1C, B2a, B1I, B3I, B2b ICDs |
| BeiDou service documents | BDS Application Service Architecture / document index | https://en.beidou.gov.cn/SYSTEMS/Officialdocument/202001/P020200116328218237495.pdf | Confirms BDS ICD family and issue dates |
| BeiDou B1I | BDS SIS ICD Open Service Signal B1I v3.0 | https://www.beidou.gov.cn/xt/gfxz/201902/P020190227593621142475.pdf | `d1d2navbit.rs`, B1I timing, BCH/interleaving, BDT handling |
| BeiDou B1C | BDS SIS ICD Open Service Signal B1C v1.0 | https://www.beidou.gov.cn/xt/gfxz/201712/P020171226741342013031.pdf | `bcnav1bit.rs`, B1C PRN and BCNAV1 checks |
| BeiDou B2a | BDS SIS ICD Open Service Signal B2a v1.0 | https://www.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf | `bcnav2bit.rs`, B2a PRN and BCNAV2 checks |
| GLONASS FDMA L1/L2 | GLONASS ICD 5.1 English public mirror | https://www.unavco.org/help/glossary/docs/ICD_GLONASS_5.1_%282008%29_en.pdf | `gnavbit.rs`, `coordinate.rs`, `satellite_param.rs` |
| GLONASS document pointer | UNAVCO knowledge-base entry for ICD 5.1 | https://kb.unavco.org/article/english-version-of-icd-glonass-version-5-1-727.html | Traceability for the English mirror |

## Verification phases

1. Parser and data ingestion
   - RINEX 2/3/4 mixed NAV headers, `TIME SYSTEM CORR`, `IONOSPHERIC CORR`,
     leap seconds, GLONASS slot/frequency lines, and malformed record handling.
   - JSON config parsing for time, trajectory, output, power, and delay options.
   - Commit target: `test(parser): verify rinex and json spec coverage`.

2. Time, orbit, and satellite parameters
   - GPS time, Galileo GST, BeiDou BDT, GLONASS time, leap-second handling,
     BDT-to-GPS offset, clock correction, group delay, Doppler sign, and
     ionosphere/troposphere delay ranges.
   - Commit target: `test(orbit): verify gnss timing and propagation`.

3. PRN and spreading-code generation
   - GPS L1 C/A, L2C, L5, L1C; Galileo E1/E5/E6; BeiDou B1I/B1C/B2a/B3I/B2b;
     GLONASS ranging code length, chip wrapping, cross-correlation sanity, and
     data/pilot separation.
   - Commit target: `test(prn): verify spreading code conformance`.

4. Navigation message formation
   - GPS LNAV/CNAV/CNAV2, Galileo I/NAV/F/NAV, BeiDou D1/D2 and BCNAV1/2/3,
     GLONASS GNav string/frame/superframe timing, parity/BCH/CRC/FEC, and
     round-trip decode where decoders exist.
   - Commit target split by constellation when the diff grows too large.

5. Carrier and IF generation
   - Signal center frequencies, IF mixing, data/pilot component phasing, carrier
     phase continuity, code phase continuity, Doppler update, secondary/NH code
     timing, IQ4/IQ8 quantization, clipping, and scalar/parallel equivalence.
   - Commit target: `test(signal): verify carrier and if generation`.

6. End-to-end generated signal checks
   - Short deterministic scenarios for GPS-only, BDS B1I, Galileo E1, GLONASS G1,
     and mixed GPS/BDS/Galileo. Check output size, finite samples, spectrum peak
     placement, and local decode/acquisition where available.
   - Commit target: `test(e2e): verify generated signal smoke scenarios`.

7. Performance and determinism
   - Stable hashes for short generated IF blocks, scalar/SIMD/parallel comparisons,
     and timing envelopes for hot loops where deterministic enough for CI/local use.
   - Commit target: `perf: add deterministic generation checks`.

## Current known risks to track

- Existing `cargo clippy -- -D warnings` is not clean because of pre-existing
  warnings; keep normal test commits focused and do not hide those warnings.
- GLONASS CDMA signals appear in newer RINEX documents, while current generator
  coverage is mostly FDMA plus existing local G3 behavior. Treat CDMA coverage as
  a separate implementation question.
- Generated IF files, GNSS-SDR outputs, Android build outputs, photos, and local
  telemetry are intentionally ignored by `.gitignore`; test fixtures should be
  small and committed under `tests/` or a dedicated fixture path.
