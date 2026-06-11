# GNSS verification report

Date: 2026-04-25; follow-up re-verification 2026-06-02; L1-band signal pass 2026-06-11.

This report summarizes the phased verification pass from RINEX/config parsing
through navigation message and IF-generation setup. Source references are pinned
in `docs/verification_sources.md`.

> **2026-06-02 follow-up.** A May 31 – June 1 audit and ICD-conformance pass
> reworked the ephemeris encoders and added round-trip decoders for every
> navigation message. Full `cargo test` now passes (65 library unit tests + all
> integration suites, 0 failures). The Galileo I/NAV decode gap noted below is
> resolved. Details in `docs/gnss_icd_conformance_report.md` ("June 2026
> conformance pass" and "Ephemeris round-trip coverage").

> **2026-06-11 L1-band signal pass.** Galileo E1 and BeiDou B1 narrowband
> generation verified end-to-end (IQ statistics, quantization, channel
> quadrature on generated files). PRN codes cross-checked bit-exact against
> independent references: B1I vs gnss-sdr (63/63 PRN), B1C data/pilot/secondary
> vs PocketSDR (63/63 PRN). The generator's two-level CBOC/QMBOC approximations
> were replaced with true multi-level ICD waveforms (E1 CBOC(6,1,1/11) on both
> channels, B1C QMBOC(6,1,4/33) quadrature pilot component); acquisition
> verified before/after with no regression (E1 10/10, z 50–97) and improved B1C
> detection (12/12, z 105–153). `verify_signal_enhanced.py` gained a B1I
> acquisition mode, B1C pilot-channel correlation, and per-system Doppler steps
> (50 Hz for 10 ms coherent codes) — the coarse 250 Hz grid, not the generator,
> caused the historic partial B1C detection. The stale `signal_assembly_baseline`
> expectation (E1 pilot on I) was updated to the hardware-confirmed Q layout.
> Hardware re-test after the waveform change is still pending.

## Phase commits

| Phase | Commit | Scope |
| --- | --- | --- |
| 1 | `8a5e093` | Ignore local generated artifacts and keep `.env` out of future commits. |
| 2 | `71de546`, `fa82805` | Add GNSS/RINEX source matrix and extend RINEX parser baseline coverage. |
| 3 | `4431593` | Verify GNSS time offsets, week/day wrapping, clock corrections, GPS/GLONASS orbit propagation, and physical delay ranges. |
| 4 | `4c13f9e` | Verify PRN matrix, data/pilot separation, cached/block IF generation, GLONASS FDMA carrier effect, and fix an NH-code shift overflow in `SatelliteSignal`. |
| 5 | `d610efa` | Verify navigation message structure: GPS LNAV preamble/TOW/subframe, Galileo I/NAV sync, GLONASS marker/string invariants, BeiDou BCNAV preambles, and `NavData` enum dispatch. |
| 6 | `e5083e8` | Verify all presets, IF-generator config loading, edge signal bits for GPS L2P and GLONASS G3, and CLI/check paths. |

## Verification commands

Targeted checks run during the phases:

```bash
cargo test --test rinex_parser_baseline
cargo test --test time_system_baseline --test orbit_geometry_baseline
cargo test --test prn_generation_baseline --test signal_assembly_baseline
cargo test --test nav_message_formation
cargo test --test if_generator_preset_baseline
cargo check --bin gnss_pilot
cargo run --quiet --bin gnss_rust -- --help
```

Full regression command run after each code/test phase:

```bash
timeout 240 cargo test
```

Latest result: full `cargo test` passed. Existing compiler warnings remain, and
the existing heavy/known-baseline tests remain ignored by their test attributes.

The newer ICD-focused conformance pass is tracked separately in
`docs/gnss_icd_conformance_report.md`. It covers GPS CNAV-2 LDPC/framing,
BeiDou B1C LDPC/interleaving, GPS L5 CNAV vectors, IQ4/IQ8 smoke checks, and the
Galileo I/NAV Viterbi/CRC decode (the formerly open gap was a test-harness bug,
fixed in commit `3c565b1`; the test now passes in the default suite).

## Issues found and fixed

- `SatelliteSignal::get_satellite_signal` could panic for long or zero NH-code
  layouts because `1 << bit_pos` overflowed when `bit_pos >= 32`. The fix now
  only applies the NH mask when the code is non-zero and the bit position fits
  in the 32-bit field.
- `IFDataGen::load_config` did not enable `GEN_L2P` for `GPS/L2P` presets or
  `GEN_G3` for `GLONASS/G3` presets. The mapping now covers both edge signals,
  and a regression test loads `presets/gps_l2p.json` and `presets/glo_g3.json`.

## Coverage notes

- Parser coverage now checks mixed RINEX headers, ionosphere/time fields, limited
  per-constellation loading, and a many-line BeiDou ionosphere header case.
- Signal coverage now spans GPS L1CA/L1C/L2C/L2P/L5, BeiDou B1I/B1C/B2I/B3I/B2a/B2b,
  Galileo E1/E5a/E5b/E6, and GLONASS G1/G3 at PRN or IF-sample level.
- Preset coverage validates every JSON file under `presets/`, checks output paths
  stay under ignored `generated_files/`, verifies ephemeris file references, and
  ensures each enabled signal maps to a known generation bit.

## Remaining risk

- `cargo clippy -- -D warnings` is still not clean because of pre-existing
  warnings outside this pass.
- Heavy end-to-end acquisition/decode tests are still ignored to avoid generating
  large IF files during normal regression runs.
- The source matrix includes newer GLONASS/RINEX CDMA references, but this pass
  only verifies the generator behavior currently implemented in the repository.
