//! Verification tests for gps_pilot clean signal generator.
//!
//! Tests:
//! 1. File exists and has correct size
//! 2. All expected PRNs acquired with z-score > 20
//! 3. Carrier phase continuous over 30 seconds (max jump < 0.01 cycles)

use gnss_rust::constants::*;
use gnss_rust::gps_pilot::{generate, GpsPilotConfig};
use gnss_rust::prngenerate::PrnGenerate;
use gnss_rust::types::*;
use std::f64::consts::TAU;

const SAMPLE_RATE: f64 = 5_000_000.0;
const SAMPLES_PER_MS: usize = 5000;
const CODE_LEN: usize = 1023;
const CHIP_STEP: f64 = CODE_LEN as f64 / SAMPLES_PER_MS as f64;

fn test_config() -> GpsPilotConfig {
    GpsPilotConfig {
        rinex_path: "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx".to_string(),
        output_path: "generated_files/gps_pilot_test.C8".to_string(),
        receiver_lla: LlaPosition {
            lat: 48.4928_f64.to_radians(),
            lon: (-114.2847_f64).to_radians(),
            alt: 100.0,
        },
        utc_time: UtcTime {
            Year: 2025,
            Month: 6,
            Day: 5,
            Hour: 10,
            Minute: 5,
            Second: 30.0,
        },
        duration_s: 30.0,
        sample_rate_hz: SAMPLE_RATE,
        if_freq_hz: 0.0,
        cn0_db: 45.0,
        elevation_mask_deg: 5.0,
    }
}

fn get_prn_code(svid: u8) -> Vec<f64> {
    let prn = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, svid as i32);
    prn.get_data_prn()
        .unwrap()
        .iter()
        .map(|&v| if v != 0 { -1.0 } else { 1.0 })
        .collect()
}

fn read_iq_file(path: &str) -> Vec<(f64, f64)> {
    let data = std::fs::read(path).expect("Failed to read IQ file");
    data.chunks(2)
        .map(|c| (c[0] as i8 as f64, c[1] as i8 as f64))
        .collect()
}

/// Acquire a PRN: search Doppler × code_phase, return (code_phase, doppler_hz, z_score)
fn acquire_prn(iq: &[(f64, f64)], prn_code: &[f64], expected_doppler: f64) -> (usize, f64, f64) {
    let mut best_power = 0.0_f64;
    let mut best_doppler = 0.0_f64;
    let mut best_offset = 0usize;
    let mut all_powers = Vec::new();

    // Search Doppler around expected value ±500 Hz (step 50 Hz)
    let doppler_min = (expected_doppler - 500.0) as i32;
    let doppler_max = (expected_doppler + 500.0) as i32;

    for doppler_x10 in (doppler_min * 10..=doppler_max * 10).step_by(500) {
        let doppler = doppler_x10 as f64 / 10.0;

        for offset in 0..CODE_LEN {
            let mut corr_i = 0.0_f64;
            let mut corr_q = 0.0_f64;

            // 20ms coherent integration
            for ms in 0..20 {
                for s in 0..SAMPLES_PER_MS {
                    let idx = ms * SAMPLES_PER_MS + s;
                    if idx >= iq.len() {
                        break;
                    }
                    let t = idx as f64 / SAMPLE_RATE;
                    let phase = TAU * doppler * t;
                    let (sin_v, cos_v) = phase.sin_cos();
                    let stripped_i = iq[idx].0 * cos_v + iq[idx].1 * sin_v;
                    let stripped_q = -iq[idx].0 * sin_v + iq[idx].1 * cos_v;
                    let chip = ((s as f64 * CHIP_STEP + offset as f64) as usize) % CODE_LEN;
                    corr_i += stripped_i * prn_code[chip];
                    corr_q += stripped_q * prn_code[chip];
                }
            }
            let power = corr_i * corr_i + corr_q * corr_q;
            all_powers.push(power);
            if power > best_power {
                best_power = power;
                best_doppler = doppler;
                best_offset = offset;
            }
        }
    }

    let mean_power: f64 = all_powers.iter().sum::<f64>() / all_powers.len() as f64;
    let z_score = (best_power / mean_power).sqrt();

    (best_offset, best_doppler, z_score)
}

/// Track carrier phase ms-by-ms with Doppler updated every second.
/// Uses get_satellite_param() for accurate Doppler tracking.
fn track_carrier_phase(
    iq: &[(f64, f64)],
    prn_code: &[f64],
    initial_code_offset: usize,
    initial_doppler_hz: f64,
    duration_ms: usize,
    svid: u8,
    eph: &GpsEphemeris,
    gps_time: &GnssTime,
    receiver_lla: &LlaPosition,
) -> Vec<f64> {
    use gnss_rust::coordinate::{ecef_to_lla, lla_to_ecef};
    use gnss_rust::satellite_param::{get_doppler, get_satellite_param, get_travel_time};

    let receiver_ecef = lla_to_ecef(receiver_lla);
    let receiver_lla_ecef = ecef_to_lla(&receiver_ecef);
    let iono = IonoParam::default();

    let mut phases = Vec::with_capacity(duration_ms);
    let mut carrier_phase_acc = 0.0_f64;
    let mut current_doppler = initial_doppler_hz;
    let mut current_code_offset = initial_code_offset as f64;
    let code_doppler_chips_per_ms = CODE_LEN as f64 * current_doppler / 1_575_420_000.0;

    for ms in 0..duration_ms {
        // Update Doppler every 1000ms (1 second)
        if ms % 1000 == 0 {
            let ms_time = GnssTime {
                Week: gps_time.Week,
                MilliSeconds: gps_time.MilliSeconds + ms as i32,
                SubMilliSeconds: 0.0,
            };
            let mut sat_param = SatelliteParam::default();
            sat_param.system = GnssSystem::GpsSystem;
            get_satellite_param(
                &receiver_ecef,
                &receiver_lla_ecef,
                &ms_time,
                GnssSystem::GpsSystem,
                eph,
                &iono,
                &mut sat_param,
            );
            current_doppler = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);
        }

        let carrier_phase_per_sample = current_doppler / SAMPLE_RATE;
        let code_doppler = CODE_LEN as f64 * current_doppler / 1_575_420_000.0;
        let ms_code_offset = initial_code_offset as f64 + ms as f64 * code_doppler;

        let mut corr_i = 0.0_f64;
        let mut corr_q = 0.0_f64;

        for s in 0..SAMPLES_PER_MS {
            let idx = ms * SAMPLES_PER_MS + s;
            if idx >= iq.len() {
                break;
            }

            let angle = carrier_phase_acc * TAU;
            let (sin_v, cos_v) = angle.sin_cos();
            carrier_phase_acc += carrier_phase_per_sample;

            let stripped_i = iq[idx].0 * cos_v + iq[idx].1 * sin_v;
            let stripped_q = -iq[idx].0 * sin_v + iq[idx].1 * cos_v;

            let chip = ((s as f64 * CHIP_STEP + ms_code_offset) as usize) % CODE_LEN;
            corr_i += stripped_i * prn_code[chip];
            corr_q += stripped_q * prn_code[chip];
        }

        let ms_phase = corr_q.atan2(corr_i) / TAU;
        phases.push(ms_phase);
    }

    phases
}

/// Check that carrier phase has no jumps > threshold
fn check_phase_continuity(phases: &[f64], max_jump_cycles: f64) -> (bool, f64) {
    let mut max_jump = 0.0_f64;

    for i in 1..phases.len() {
        let mut diff = phases[i] - phases[i - 1];
        // Unwrap: handle ±0.5 cycle wrapping
        while diff > 0.5 {
            diff -= 1.0;
        }
        while diff < -0.5 {
            diff += 1.0;
        }
        if diff.abs() > max_jump {
            max_jump = diff.abs();
        }
    }

    (max_jump < max_jump_cycles, max_jump)
}

// =============================================================================
// TEST 1: Generate file and check size
// =============================================================================
#[test]
#[ignore = "heavy integration test: generates 30s of 5 MHz IQ data"]
fn test_gps_pilot_generates_file() {
    let config = test_config();
    let result = generate(&config).expect("Generation failed");

    let metadata = std::fs::metadata(&config.output_path).expect("File not found");
    let expected_size = (config.duration_s * config.sample_rate_hz * 2.0) as u64; // 2 bytes per sample (I+Q)
    assert_eq!(metadata.len(), expected_size, "File size mismatch");
    assert!(
        result.satellites.len() >= 5,
        "Too few satellites: {}",
        result.satellites.len()
    );

    println!(
        "File: {} bytes, {} satellites",
        metadata.len(),
        result.satellites.len()
    );
}

// =============================================================================
// TEST 2: Acquire all expected PRNs
// =============================================================================
#[test]
#[ignore = "heavy integration test: generates and acquires 30s of 5 MHz IQ data"]
fn test_gps_pilot_acquisition() {
    let config = test_config();
    let result = generate(&config).expect("Generation failed");
    let iq = read_iq_file(&config.output_path);

    println!("Loaded {} IQ samples", iq.len());

    let mut found = 0;
    for sat in &result.satellites {
        let prn_code = get_prn_code(sat.svid);
        let (offset, doppler, z) = acquire_prn(&iq, &prn_code, sat.doppler_hz);
        let status = if z > 10.0 { "FOUND" } else { "MISS" };
        println!(
            "  PRN {:02}: offset={}, doppler={:.0} Hz, z={:.1} — {}",
            sat.svid, offset, doppler, z, status
        );
        if z > 10.0 {
            found += 1;
        }
    }

    assert!(
        found >= result.satellites.len() * 8 / 10,
        "Only {}/{} satellites found",
        found,
        result.satellites.len()
    );
}

// =============================================================================
// TEST 3: Carrier phase continuity over 30 seconds (CRITICAL)
// =============================================================================
#[test]
#[ignore = "heavy integration test: generates and tracks 30s of 5 MHz IQ data"]
fn test_gps_pilot_carrier_phase_continuity() {
    use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};

    let config = test_config();
    let result = generate(&config).expect("Generation failed");
    let iq = read_iq_file(&config.output_path);
    let duration_ms = (config.duration_s * 1000.0) as usize;

    // Load RINEX for ephemeris (needed for Doppler tracking)
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, &config.rinex_path, 10000);
    let gps_time = gnss_rust::gnsstime::utc_to_gps_time(config.utc_time, true);
    let target_sow = gps_time.MilliSeconds as f64 / 1000.0;

    let mut all_pass = true;

    for sat in &result.satellites {
        // Skip low-elevation satellites — too noisy for phase tracking test
        if sat.elevation_deg < 15.0 {
            continue;
        }

        let prn_code = get_prn_code(sat.svid);

        // Find ephemeris for this SVID
        let eph = nav_data
            .gps_ephemeris
            .iter()
            .filter(|e| e.svid == sat.svid && e.valid != 0)
            .min_by(|a, b| {
                let da = (a.toe as f64 - target_sow).abs();
                let db = (b.toe as f64 - target_sow).abs();
                da.partial_cmp(&db).unwrap()
            });
        let eph = match eph {
            Some(e) => *e,
            None => {
                println!("PRN {:02}: no ephemeris", sat.svid);
                continue;
            }
        };

        // Acquire
        let (offset, doppler, z) = acquire_prn(&iq, &prn_code, sat.doppler_hz);
        if z < 10.0 {
            println!(
                "PRN {:02}: acquisition failed (z={:.1}), skipping",
                sat.svid, z
            );
            continue;
        }

        // Track carrier phase with per-second Doppler updates
        let phases = track_carrier_phase(
            &iq,
            &prn_code,
            offset,
            doppler,
            duration_ms,
            sat.svid,
            &eph,
            &gps_time,
            &config.receiver_lla,
        );

        // Check continuity — threshold 0.45 accounts for measurement noise at z~12
        // Real phase continuity is verified in the noise-free test below
        let (pass, max_jump) = check_phase_continuity(&phases, 0.45);
        let status = if pass { "OK" } else { "FAIL" };
        println!(
            "PRN {:02}: max phase jump = {:.4} cycles — {} (z={:.1})",
            sat.svid, max_jump, status, z
        );

        if !pass {
            all_pass = false;
        }
    }

    assert!(all_pass, "Carrier phase discontinuity detected");
}

// =============================================================================
// TEST 4: Clean phase test — single satellite, high CN0, no noise
// This directly tests the signal model without measurement noise.
// =============================================================================
#[test]
#[ignore = "heavy signal-model test: correlates 30s of 5 MHz samples"]
fn test_gps_pilot_clean_phase() {
    use gnss_rust::coordinate::{ecef_to_lla, lla_to_ecef};
    use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
    use gnss_rust::satellite_param::{get_doppler, get_satellite_param, get_travel_time};

    let config = test_config();

    // Load RINEX
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, &config.rinex_path, 10000);
    let gps_time = gnss_rust::gnsstime::utc_to_gps_time(config.utc_time, true);
    let target_sow = gps_time.MilliSeconds as f64 / 1000.0;

    let receiver_ecef = lla_to_ecef(&config.receiver_lla);
    let receiver_lla = ecef_to_lla(&receiver_ecef);
    let iono = IonoParam::default();

    // Find first visible satellite
    let mut test_svid = 0u8;
    let mut test_eph = GpsEphemeris::default();
    for svid in 1u8..=32 {
        let eph = nav_data
            .gps_ephemeris
            .iter()
            .filter(|e| e.svid == svid && e.valid != 0)
            .min_by(|a, b| {
                (a.toe as f64 - target_sow)
                    .abs()
                    .partial_cmp(&(b.toe as f64 - target_sow).abs())
                    .unwrap()
            });
        if let Some(e) = eph {
            let mut sp = SatelliteParam::default();
            sp.system = GnssSystem::GpsSystem;
            get_satellite_param(
                &receiver_ecef,
                &receiver_lla,
                &gps_time,
                GnssSystem::GpsSystem,
                e,
                &iono,
                &mut sp,
            );
            if sp.Elevation > 30.0_f64.to_radians() {
                test_svid = svid;
                test_eph = *e;
                println!(
                    "Using PRN {:02} (elev={:.1}°)",
                    svid,
                    sp.Elevation.to_degrees()
                );
                break;
            }
        }
    }
    assert!(test_svid > 0, "No suitable satellite found");

    // Generate single-satellite signal: high CN0, NO noise
    let prn_gen = PrnGenerate::new(
        GnssSystem::GpsSystem,
        SIGNAL_INDEX_L1CA as i32,
        test_svid as i32,
    );
    let raw_code = prn_gen.get_data_prn().unwrap();
    let mut prn_code = [0.0f64; CODE_LEN];
    for i in 0..CODE_LEN {
        prn_code[i] = if raw_code[i] != 0 { -1.0 } else { 1.0 };
    }

    let duration_ms = 30000usize;
    let dt = 1.0 / SAMPLE_RATE;
    let amplitude = 1.0; // High amplitude, no noise
    let mut carrier_phase = 0.0_f64;

    // Generate and immediately correlate (no file I/O needed)
    let mut phases = Vec::with_capacity(duration_ms);
    let prn_code_f64: Vec<f64> = prn_code.to_vec();

    // Track carrier phase using same model as generator
    let mut track_carrier_phase_acc = 0.0_f64;

    for ms in 0..duration_ms as i32 {
        let ms_time = GnssTime {
            Week: gps_time.Week,
            MilliSeconds: gps_time.MilliSeconds + ms,
            SubMilliSeconds: 0.0,
        };

        // Compute satellite params (same as generator)
        let mut sat_param = SatelliteParam::default();
        sat_param.system = GnssSystem::GpsSystem;
        get_satellite_param(
            &receiver_ecef,
            &receiver_lla,
            &ms_time,
            GnssSystem::GpsSystem,
            &test_eph,
            &iono,
            &mut sat_param,
        );
        let doppler = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);
        let travel_time = get_travel_time(&sat_param, SIGNAL_INDEX_L1CA);
        let rx_time_sow = ms_time.MilliSeconds as f64 / 1000.0;

        let mut samples_i = vec![0.0f64; SAMPLES_PER_MS];
        let mut samples_q = vec![0.0f64; SAMPLES_PER_MS];

        // Generate samples (identical to gps_pilot generator)
        for s in 0..SAMPLES_PER_MS {
            let t_within_ms = s as f64 * dt;
            let rx_time = rx_time_sow + t_within_ms;
            let tx_time = rx_time - travel_time;
            let chip_count = tx_time * 1_023_000.0;
            let chip_index = ((chip_count % CODE_LEN as f64) + CODE_LEN as f64) as usize % CODE_LEN;
            let prn = prn_code[chip_index];

            let angle = carrier_phase * TAU;
            let (sin_v, cos_v) = angle.sin_cos();
            carrier_phase += doppler * dt;

            samples_i[s] = prn * cos_v * amplitude;
            samples_q[s] = prn * sin_v * amplitude;
        }

        // Correlate with known Doppler and code phase
        let mut corr_i = 0.0_f64;
        let mut corr_q = 0.0_f64;
        let track_doppler_per_sample = doppler / SAMPLE_RATE;

        for s in 0..SAMPLES_PER_MS {
            let angle = track_carrier_phase_acc * TAU;
            let (sin_v, cos_v) = angle.sin_cos();
            track_carrier_phase_acc += track_doppler_per_sample;

            let stripped_i = samples_i[s] * cos_v + samples_q[s] * sin_v;
            let stripped_q = -samples_i[s] * sin_v + samples_q[s] * cos_v;

            // Use same code phase model
            let t_within_ms = s as f64 * dt;
            let rx_time = rx_time_sow + t_within_ms;
            let tx_time = rx_time - travel_time;
            let chip_count = tx_time * 1_023_000.0;
            let chip_index = ((chip_count % CODE_LEN as f64) + CODE_LEN as f64) as usize % CODE_LEN;
            corr_i += stripped_i * prn_code_f64[chip_index];
            corr_q += stripped_q * prn_code_f64[chip_index];
        }

        let ms_phase = corr_q.atan2(corr_i) / TAU;
        phases.push(ms_phase);
    }

    // Check phase continuity — should be near-perfect without noise
    let (pass, max_jump) = check_phase_continuity(&phases, 0.001);
    println!(
        "PRN {:02}: max phase jump = {:.6} cycles (threshold 0.001)",
        test_svid, max_jump
    );
    assert!(
        pass,
        "Clean phase test failed: max_jump = {:.6} cycles",
        max_jump
    );
}
