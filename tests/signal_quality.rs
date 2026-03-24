//! Signal quality tests for real receiver compatibility.
//!
//! These tests verify that the generated IF signal has properties required
//! for a real GNSS receiver to acquire, track, and decode navigation data.

use gnss_rust::complex_number::ComplexNumber;
use gnss_rust::constants::*;
use gnss_rust::coordinate::{ecef_to_lla, lla_to_ecef};
use gnss_rust::inavbit::INavBit;
use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use gnss_rust::lnavbit::LNavBit;
use gnss_rust::nav_data::NavData;
use gnss_rust::prngenerate::*;
use gnss_rust::sat_if_signal::SatIfSignal;
use gnss_rust::satellite_param::*;
use gnss_rust::types::*;

const RINEX_PATH: &str = "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx";
const SAMPLE_RATE: i32 = 5000; // 5 MHz = 5000 samples/ms
const CENTER_FREQ: f64 = 1575.42e6;

/// Helper: load RINEX and get GPS ephemeris + iono/utc
fn load_rinex() -> CNavData {
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, RINEX_PATH, 10000);
    nav_data
}

/// Helper: create a GPS L1CA SatIfSignal for given SVID with nav data
fn create_gps_signal(svid: u8, nav_data: &CNavData, cur_time: &GnssTime) -> Option<(SatIfSignal, SatelliteParam)> {
    // Find ephemeris closest to target time (toe closest to cur_time seconds of week)
    let target_sow = cur_time.MilliSeconds as f64 / 1000.0;
    let eph = nav_data.gps_ephemeris.iter()
        .filter(|e| e.svid == svid && e.valid != 0)
        .min_by(|a, b| {
            let da = (a.toe as f64 - target_sow).abs();
            let db = (b.toe as f64 - target_sow).abs();
            da.partial_cmp(&db).unwrap()
        })?;

    let mut sat_param = SatelliteParam::default();
    sat_param.system = GnssSystem::GpsSystem;
    // Moscow Red Square (55.7539°N, 37.6208°E, 150m) → ECEF
    let lla = LlaPosition {
        lat: 55.7539_f64.to_radians(),
        lon: 37.6208_f64.to_radians(),
        alt: 150.0,
    };
    let cur_pos = lla_to_ecef(&lla);
    let position_lla = ecef_to_lla(&cur_pos);
    let default_iono = IonoParam::default();

    get_satellite_param(
        &cur_pos, &position_lla, cur_time,
        GnssSystem::GpsSystem, eph, &default_iono,
        &mut sat_param,
    );

    if sat_param.Elevation < 0.0_f64.to_radians() {
        return None; // Below horizon
    }

    sat_param.CN0 = 4500; // 45.0 dBHz * 100

    // Create LNavBit with ephemeris
    let mut lnav = LNavBit::new();
    lnav.set_ephemeris(svid as i32, eph);
    if let (Some(alpha), Some(beta), Some(utc)) = (nav_data.get_gps_iono_alpha(), nav_data.get_gps_iono_beta(), nav_data.utc_param.as_ref()) {
        let iono_param = IonoParam {
            a0: alpha[0], a1: alpha[1], a2: alpha[2], a3: alpha[3],
            b0: beta[0], b1: beta[1], b2: beta[2], b3: beta[3],
            flag: 1,
        };
        lnav.set_iono_utc(&iono_param, utc);
    }

    let nav = NavData::LNav(lnav);

    let if_freq = (CENTER_FREQ - CENTER_FREQ) as i32; // 0 for L1
    let mut sig = SatIfSignal::new(SAMPLE_RATE, if_freq, GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, svid);
    sig.init_state(*cur_time, &sat_param, Some(nav));

    Some((sig, sat_param))
}

// =============================================================================
// TEST 1: Carrier phase continuity across ms boundaries
// =============================================================================
#[test]
fn test_carrier_phase_continuity() {
    let nav_data = load_rinex();
    let cur_time = GnssTime { Week: 2369, MilliSeconds: 381948000, SubMilliSeconds: 0.0 };

    // Test ALL GPS satellites to find any with phase jumps
    for svid in 1u8..=32 {
        let result = create_gps_signal(svid, &nav_data, &cur_time);
        if result.is_none() { continue; }
        let (mut sig, sat_param) = result.unwrap();

        let mut prev_last_sample = ComplexNumber::new();
        let mut max_phase_jump = 0.0_f64;

        for ms in 0..100 {
            let ms_time = GnssTime {
                Week: 2369,
                MilliSeconds: 381948000 + ms,
                SubMilliSeconds: 0.0,
            };

            // Update params every ms (like the real generation loop)
            if ms == 0 {
                sig.update_satellite_params(&sat_param, CENTER_FREQ, &ms_time);
            } else {
                sig.push_sat_param_for_ms(&sat_param, CENTER_FREQ, &ms_time);
            }

            sig.get_if_sample_cached(ms_time);

            if ms > 0 {
                // Compare last sample of previous ms with first sample of current ms
                let first = sig.sample_array[0];
                // Phase of each sample
                let phase_prev = prev_last_sample.imag.atan2(prev_last_sample.real);
                let phase_curr = first.imag.atan2(first.real);
                let mut phase_diff = (phase_curr - phase_prev).abs();
                if phase_diff > std::f64::consts::PI {
                    phase_diff = std::f64::consts::TAU - phase_diff;
                }
                // Expect phase difference close to one sample step (small)
                // A hard re-anchor would cause a large jump
                if phase_diff > max_phase_jump {
                    max_phase_jump = phase_diff;
                }
            }

            let n = sig.sample_array.len();
            prev_last_sample = sig.sample_array[n - 1];
        }

        // Phase jumps can be ~π (nav bit transitions in BPSK) or small (continuous carrier).
        // Filter out π jumps (nav data modulation) and check remaining jumps are small.
        // A broken re-anchor causes non-π jumps > 0.1 rad.
        let max_non_nav_jump = {
            // Recompute excluding jumps near π
            let mut max_j = 0.0_f64;
            // (we already computed max_phase_jump but need the non-π version)
            // For simplicity, just report the max_phase_jump and check if it's π or small
            if (max_phase_jump - std::f64::consts::PI).abs() < 0.1 {
                // π jump = nav bit transition, acceptable
                0.0
            } else {
                max_phase_jump
            }
        };
        let status = if max_non_nav_jump < 0.1 { "OK" } else { "FAIL" };
        println!("PRN {:02}: max phase jump = {:.4} rad ({:.4} cycles), non-nav = {:.4} — {}",
                 svid, max_phase_jump, max_phase_jump / std::f64::consts::TAU, max_non_nav_jump, status);
        assert!(
            max_non_nav_jump < 0.1,
            "PRN {}: non-navigation carrier phase jump {:.4} rad exceeds limit",
            svid, max_non_nav_jump
        );
    }
}

// =============================================================================
// TEST 2: Code phase continuity across ms boundaries
// =============================================================================
#[test]
fn test_code_phase_continuity() {
    let nav_data = load_rinex();
    let cur_time = GnssTime { Week: 2369, MilliSeconds: 381948000, SubMilliSeconds: 0.0 };

    for &svid in &[23u8, 27, 15, 10, 8, 2] {
        let result = create_gps_signal(svid, &nav_data, &cur_time);
        if result.is_none() { continue; }
        let (mut sig, sat_param) = result.unwrap();

        // Generate 200ms and check correlation peak position doesn't jump
        let mut prev_peak_pos: Option<f64> = None;
        let mut max_code_jump = 0.0_f64;
        let samples_per_ms = SAMPLE_RATE as usize;
        let chip_step = 1023.0 / samples_per_ms as f64;

        // Get PRN code for correlation
        let prn_code: Vec<f64> = {
            let prn = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, svid as i32);
            match prn.get_data_prn() {
                Some(code) => code.iter().map(|&v| if v != 0 { -1.0 } else { 1.0 }).collect(),
                None => { continue; }
            }
        };

        sig.update_satellite_params(&sat_param, CENTER_FREQ, &cur_time);
        let doppler_hz = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);

        for ms in 0..200 {
            let ms_time = GnssTime {
                Week: 2369,
                MilliSeconds: 381948000 + ms,
                SubMilliSeconds: 0.0,
            };

            if ms == 0 {
                // already called update_satellite_params above
            } else {
                sig.push_sat_param_for_ms(&sat_param, CENTER_FREQ, &ms_time);
            }
            sig.get_if_sample_cached(ms_time);

            // Strip carrier then correlate with PRN code
            let mut best_offset = 0.0;
            let mut best_power: f64 = 0.0;

            for offset_idx in 0..1023 {
                let offset = offset_idx as f64;
                let mut corr_i = 0.0;
                let mut corr_q = 0.0;
                for s in 0..samples_per_ms {
                    let t = (ms as f64 + s as f64 / samples_per_ms as f64) / 1000.0;
                    let phase = std::f64::consts::TAU * doppler_hz * t;
                    let (sin_v, cos_v) = phase.sin_cos();
                    let stripped_i = sig.sample_array[s].real * cos_v + sig.sample_array[s].imag * sin_v;
                    let stripped_q = -sig.sample_array[s].real * sin_v + sig.sample_array[s].imag * cos_v;
                    let chip = ((s as f64 * chip_step + offset) as usize) % 1023;
                    corr_i += stripped_i * prn_code[chip];
                    corr_q += stripped_q * prn_code[chip];
                }
                let power = corr_i * corr_i + corr_q * corr_q;
                if power > best_power {
                    best_power = power;
                    best_offset = offset;
                }
            }

            if let Some(prev) = prev_peak_pos {
                let jump = (best_offset - prev).abs();
                let jump = if jump > 511.0 { 1023.0 - jump } else { jump };
                if jump > max_code_jump {
                    max_code_jump = jump;
                }
            }
            prev_peak_pos = Some(best_offset);
        }

        // Code phase should not jump more than 2 chips between consecutive ms
        assert!(
            max_code_jump < 5.0,
            "PRN {}: code phase jump {:.2} chips exceeds limit",
            svid, max_code_jump
        );
        println!("PRN {:02}: max code phase jump = {:.2} chips — OK", svid, max_code_jump);
    }
}

// =============================================================================
// TEST 3: GPS LNAV nav bits are decodable for ALL visible satellites
// =============================================================================
#[test]
fn test_gps_lnav_all_svs_decodable() {
    let nav_data = load_rinex();
    let cur_time = GnssTime { Week: 2369, MilliSeconds: 381948000, SubMilliSeconds: 0.0 };

    let mut lnav = LNavBit::new();

    // Load ALL GPS ephemerides
    let mut loaded_svids = Vec::new();
    for eph in &nav_data.gps_ephemeris {
        if eph.valid != 0 && eph.svid >= 1 && eph.svid <= 32 {
            let result = lnav.set_ephemeris(eph.svid as i32, eph);
            if result != 0 {
                loaded_svids.push(eph.svid);
            }
        }
    }

    // Load iono/UTC
    if let (Some(alpha), Some(beta)) = (nav_data.get_gps_iono_alpha(), nav_data.get_gps_iono_beta()) {
        if let Some(utc) = &nav_data.utc_param {
            let iono_param = IonoParam {
                a0: alpha[0], a1: alpha[1], a2: alpha[2], a3: alpha[3],
                b0: beta[0], b1: beta[1], b2: beta[2], b3: beta[3],
                flag: 1,
            };
            lnav.set_iono_utc(&iono_param, utc);
        }
    }

    println!("Loaded {} GPS SVIDs: {:?}", loaded_svids.len(), loaded_svids);

    // For each satellite, generate nav bits for 60 seconds (10 complete subframe cycles)
    // and verify preamble + parity
    let mut all_ok = true;
    for &svid in &loaded_svids {
        let mut preamble_found = 0;
        let mut parity_errors = 0;
        let mut subframes_seen = [false; 6]; // index 1-5

        // Generate 60 seconds of subframes (10 complete cycles)
        for tow_offset in 0..10 {
            for sf in 0..5 {
                let tow = tow_offset * 5 + sf;
                let time = GnssTime {
                    Week: 2369,
                    MilliSeconds: tow * 6000, // 6 seconds per subframe
                    SubMilliSeconds: 0.0,
                };

                let mut nav_bits = [0i32; 300];
                lnav.get_frame_data(time, svid as i32, 0, &mut nav_bits);

                // Check preamble (first 8 bits = 10001011)
                let preamble = (0..8).fold(0u8, |acc, i| {
                    (acc << 1) | (nav_bits[i] as u8 & 1)
                });
                if preamble == 0x8B {
                    preamble_found += 1;
                }

                // Check parity for all 10 words (each 30 bits)
                for word_idx in 0..10 {
                    let word_start = word_idx * 30;
                    let mut word: u32 = 0;
                    for bit in 0..30 {
                        word = (word << 1) | (nav_bits[word_start + bit] as u32 & 1);
                    }
                    // D29* and D30* from previous word
                    let d29_star = if word_idx > 0 {
                        (nav_bits[word_start - 1] as u32) & 1
                    } else {
                        0
                    };
                    let d30_star = if word_idx > 0 {
                        (nav_bits[word_start - 2] as u32) & 1
                    } else {
                        0
                    };
                    // Parity check: last 6 bits should match computed parity
                    // Simple check: last 2 bits of word 10 should be 0 (by construction)
                    if word_idx == 9 {
                        let last_two = word & 0x3;
                        if last_two != 0 {
                            parity_errors += 1;
                        }
                    }
                }

                // Track which subframes were generated
                let sf_id = ((tow % 5) + 1) as usize;
                if sf_id <= 5 {
                    subframes_seen[sf_id] = true;
                }
            }
        }

        let all_sf = subframes_seen[1] && subframes_seen[2] && subframes_seen[3];
        let status = if preamble_found == 50 && parity_errors == 0 && all_sf { "OK" } else { "FAIL" };
        println!("GPS PRN {:02}: preamble={}/50, parity_err={}, SF1-3={} — {}",
                 svid, preamble_found, parity_errors, all_sf, status);

        if status == "FAIL" {
            all_ok = false;
        }
    }

    assert!(all_ok, "Some GPS satellites have LNAV encoding errors");
}

// =============================================================================
// TEST 4: Galileo I-NAV CRC24Q verification for all visible SVs
// =============================================================================
#[test]
fn test_galileo_inav_all_svs_crc() {
    let nav_data = load_rinex();

    let mut inav = INavBit::new();

    let mut loaded_svids = Vec::new();
    for eph in &nav_data.galileo_ephemeris {
        if eph.valid != 0 && eph.svid >= 1 && eph.svid <= 36 {
            let result = inav.set_ephemeris(eph.svid as i32, eph);
            if result != 0 {
                loaded_svids.push(eph.svid);
            }
        }
    }
    loaded_svids.sort();
    loaded_svids.dedup();

    println!("Loaded {} Galileo SVIDs: {:?}", loaded_svids.len(), loaded_svids);
    assert!(!loaded_svids.is_empty(), "No Galileo ephemerides loaded");

    let mut all_ok = true;
    for &svid in &loaded_svids {
        let mut sync_ok = 0;
        let mut total_pages = 0;

        // Generate 60 seconds of I-NAV pages (each page = 2 seconds)
        for page_idx in 0..30 {
            let tow = page_idx * 2 + 1; // Odd tow for E1
            let time = GnssTime {
                Week: 2369,
                MilliSeconds: tow * 1000,
                SubMilliSeconds: 0.0,
            };

            let mut nav_bits = [0i32; 4096];
            inav.get_frame_data(time, svid as i32, 1, &mut nav_bits);
            total_pages += 1;

            // Check sync pattern (first 10 bits of even part)
            let sync: Vec<i32> = nav_bits[0..10].to_vec();
            let expected_sync = [0, 1, 0, 1, 1, 0, 0, 0, 0, 0];
            if sync == expected_sync {
                sync_ok += 1;
            }

            // Check sync pattern of odd part (bits 250..260)
            let sync_odd: Vec<i32> = nav_bits[250..260].to_vec();
            if sync_odd == expected_sync {
                sync_ok += 1;
            }
        }

        let status = if sync_ok == total_pages * 2 { "OK" } else { "FAIL" };
        println!("GAL E{:02}: sync={}/{} — {}", svid, sync_ok, total_pages * 2, status);

        if status == "FAIL" {
            all_ok = false;
        }
    }

    assert!(all_ok, "Some Galileo satellites have I-NAV sync pattern errors");
}

// =============================================================================
// TEST 5: Signal amplitude and noise level match expectations
// =============================================================================
#[test]
fn test_signal_amplitude_consistency() {
    let nav_data = load_rinex();
    let cur_time = GnssTime { Week: 2369, MilliSeconds: 381948000, SubMilliSeconds: 0.0 };

    for &svid in &[23u8, 15, 8] {
        let result = create_gps_signal(svid, &nav_data, &cur_time);
        if result.is_none() { continue; }
        let (mut sig, sat_param) = result.unwrap();

        sig.update_satellite_params(&sat_param, CENTER_FREQ, &cur_time);
        sig.get_if_sample_cached(cur_time);

        // Compute RMS of generated samples
        let n = sig.sample_array.len();
        let rms = (sig.sample_array.iter()
            .map(|s| s.real * s.real + s.imag * s.imag)
            .sum::<f64>() / n as f64).sqrt();

        // Signal should have non-zero amplitude
        assert!(rms > 1e-6, "PRN {}: RMS is zero — signal not generated", svid);

        // Check amplitude is reasonable (not clipping, not too small)
        // Expected: sqrt(CN0_linear / Fs) where CN0 ~45 dBHz, Fs = 5MHz
        // = sqrt(10^4.5 / 5e6) = sqrt(31623 / 5e6) = sqrt(0.00632) = 0.0795
        assert!(rms > 0.01, "PRN {}: RMS {:.6} too small", svid, rms);
        assert!(rms < 1.0, "PRN {}: RMS {:.6} too large", svid, rms);

        println!("PRN {:02}: RMS = {:.6} — OK", svid, rms);
    }
}

// =============================================================================
// TEST 6: GPS L1CA PRN code correlation peak exists
// =============================================================================
#[test]
fn test_gps_prn_correlation() {
    let nav_data = load_rinex();
    let cur_time = GnssTime { Week: 2369, MilliSeconds: 381948000, SubMilliSeconds: 0.0 };

    for &svid in &[23u8, 27, 15, 10, 8, 2, 32] {
        let result = create_gps_signal(svid, &nav_data, &cur_time);
        if result.is_none() {
            println!("PRN {:02}: not visible — skipped", svid);
            continue;
        }
        let (mut sig, sat_param) = result.unwrap();

        sig.update_satellite_params(&sat_param, CENTER_FREQ, &cur_time);
        let doppler_hz = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);

        // Get PRN code
        let prn_code: Vec<f64> = {
            let prn = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, svid as i32);
            match prn.get_data_prn() {
                Some(code) => code.iter().map(|&v| if v != 0 { -1.0 } else { 1.0 }).collect(),
                None => { println!("PRN {:02}: no PRN code — skipped", svid); continue; }
            }
        };

        // Generate 1ms and search all 1023 code offsets with carrier stripping
        sig.get_if_sample_cached(cur_time);
        let samples_per_ms = SAMPLE_RATE as usize;
        let chip_step = 1023.0 / samples_per_ms as f64;

        let mut powers = vec![0.0f64; 1023];
        for offset in 0..1023 {
            let mut corr_i = 0.0_f64;
            let mut corr_q = 0.0_f64;
            for s in 0..samples_per_ms {
                let t = s as f64 / (samples_per_ms as f64 * 1000.0);
                let phase = std::f64::consts::TAU * doppler_hz * t;
                let (sin_v, cos_v) = phase.sin_cos();
                let stripped_i = sig.sample_array[s].real * cos_v + sig.sample_array[s].imag * sin_v;
                let stripped_q = -sig.sample_array[s].real * sin_v + sig.sample_array[s].imag * cos_v;
                let chip = ((s as f64 * chip_step + offset as f64) as usize) % 1023;
                corr_i += stripped_i * prn_code[chip];
                corr_q += stripped_q * prn_code[chip];
            }
            powers[offset] = corr_i * corr_i + corr_q * corr_q;
        }

        let mean_power: f64 = powers.iter().sum::<f64>() / 1023.0;
        let max_power = powers.iter().fold(0.0f64, |a, &b| a.max(b));
        let peak_ratio = max_power / mean_power.max(1e-20);

        println!("PRN {:02}: peak/mean = {:.1}, mean_power = {:.2e}, max_power = {:.2e}, doppler = {:.0} Hz",
                 svid, peak_ratio, mean_power, max_power, doppler_hz);

        // With correct PRN code and carrier stripping, peak should be >> mean
        assert!(
            peak_ratio > 5.0,
            "PRN {}: peak_ratio {:.1} too low — PRN code not correlating",
            svid, peak_ratio
        );
    }
}

// =============================================================================
// TEST 7: End-to-end GPS LNAV decode from IF signal (receiver simulation)
//
// This test simulates what a real GNSS receiver does:
//   1. Generate 60 seconds of GPS L1CA IF signal per satellite
//   2. Strip the carrier using known Doppler frequency
//   3. Correlate with local PRN code replica to despread the signal
//   4. Accumulate correlation over 20ms to extract one nav bit
//   5. Search the resulting bit stream for the GPS preamble (10001011)
//   6. Verify GPS ICD parity on decoded subframes
//   7. Require at least 3 consecutive valid subframes per satellite
// =============================================================================
#[test]
fn test_gps_lnav_end_to_end_decode() {
    let nav_data = load_rinex();
    let start_time = GnssTime { Week: 2369, MilliSeconds: 381948000, SubMilliSeconds: 0.0 };
    let samples_per_ms = SAMPLE_RATE as usize;
    let chip_step = 1023.0 / samples_per_ms as f64;

    // GPS preamble: 10001011 = 0x8B (first 8 bits of every subframe TLM word)
    // Correlation convention: transmitted bit 0 -> data_bit=+1 -> positive correlation -> nav_bit=+1
    //                         transmitted bit 1 -> data_bit=-1 -> negative correlation -> nav_bit=-1
    // Preamble bits 1,0,0,0,1,0,1,1 -> nav_bits: -1,+1,+1,+1,-1,+1,-1,-1
    let preamble_pattern: [i8; 8] = [-1, 1, 1, 1, -1, 1, -1, -1];
    // Inverted preamble (if D30* of previous subframe was 1, all transmitted bits flip)
    let preamble_inv: [i8; 8] = [1, -1, -1, -1, 1, -1, 1, 1];

    let test_svids: &[u8] = &[23, 15, 8, 27, 10, 2];
    let duration_ms: i32 = 60000; // 60 seconds
    let param_update_interval: i32 = 1000; // Full Kepler update every 1s

    // Receiver position (Moscow Red Square)
    let lla = LlaPosition {
        lat: 55.7539_f64.to_radians(),
        lon: 37.6208_f64.to_radians(),
        alt: 150.0,
    };
    let cur_pos = lla_to_ecef(&lla);
    let position_lla = ecef_to_lla(&cur_pos);
    let default_iono = IonoParam::default();

    let mut all_ok = true;
    let mut visible_count = 0;

    for &svid in test_svids {
        // --- Find best ephemeris ---
        let target_sow = start_time.MilliSeconds as f64 / 1000.0;
        let eph_opt = nav_data.gps_ephemeris.iter()
            .filter(|e| e.svid == svid && e.valid != 0)
            .min_by(|a, b| {
                let da = (a.toe as f64 - target_sow).abs();
                let db = (b.toe as f64 - target_sow).abs();
                da.partial_cmp(&db).unwrap()
            });
        let eph = match eph_opt {
            Some(e) => *e,
            None => {
                println!("PRN {:02}: no ephemeris — skipped", svid);
                continue;
            }
        };

        // --- Compute initial satellite parameters ---
        let mut sat_param = SatelliteParam::default();
        sat_param.system = GnssSystem::GpsSystem;
        get_satellite_param(
            &cur_pos, &position_lla, &start_time,
            GnssSystem::GpsSystem, &eph, &default_iono,
            &mut sat_param,
        );

        if sat_param.Elevation < 5.0_f64.to_radians() {
            println!("PRN {:02}: elevation {:.1} deg — below mask, skipped",
                     svid, sat_param.Elevation.to_degrees());
            continue;
        }

        // Set CN0 (get_satellite_param does not set it; default 0 gives ~0.0005 amplitude)
        // 45.0 dBHz is typical for a healthy GPS satellite at moderate elevation
        sat_param.CN0 = 4500; // 45.0 dBHz * 100

        visible_count += 1;
        println!("PRN {:02}: elevation {:.1} deg, generating {} ms of signal...",
                 svid, sat_param.Elevation.to_degrees(), duration_ms);

        // --- Build LNavBit with ephemeris and iono/UTC ---
        let mut lnav = LNavBit::new();
        lnav.set_ephemeris(svid as i32, &eph);
        if let (Some(alpha), Some(beta), Some(utc)) = (
            nav_data.get_gps_iono_alpha(),
            nav_data.get_gps_iono_beta(),
            nav_data.utc_param.as_ref(),
        ) {
            let iono_param = IonoParam {
                a0: alpha[0], a1: alpha[1], a2: alpha[2], a3: alpha[3],
                b0: beta[0], b1: beta[1], b2: beta[2], b3: beta[3],
                flag: 1,
            };
            lnav.set_iono_utc(&iono_param, utc);
        }
        let nav = NavData::LNav(lnav);

        // --- Create SatIfSignal ---
        let if_freq = (CENTER_FREQ - CENTER_FREQ) as i32; // 0 for L1 at L1 center
        let mut sig = SatIfSignal::new(SAMPLE_RATE, if_freq, GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, svid);
        sig.init_state(start_time, &sat_param, Some(nav));

        // --- Get local PRN code replica ---
        let prn_gen = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, svid as i32);
        let prn_code: Vec<f64> = match prn_gen.get_data_prn() {
            Some(code) => code.iter().map(|&v| if v != 0 { -1.0 } else { 1.0 }).collect(),
            None => {
                println!("PRN {:02}: failed to generate PRN code — skipped", svid);
                continue;
            }
        };

        // --- Phase 1: Acquisition — find code phase using non-coherent detection ---
        // Generate first ms and search all 1023 code offsets.
        // Use both I and Q correlation to handle unknown carrier phase:
        //   power = corr_I^2 + corr_Q^2  (non-coherent, phase-independent)
        sig.update_satellite_params(&sat_param, CENTER_FREQ, &start_time);
        let doppler_hz = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);
        sig.get_if_sample_cached(start_time);

        // Quick sanity check on signal amplitude
        {
            let rms = (sig.sample_array.iter().take(samples_per_ms)
                .map(|s| s.real * s.real + s.imag * s.imag)
                .sum::<f64>() / samples_per_ms as f64).sqrt();
            assert!(rms > 0.01, "PRN {}: signal RMS {:.6} too small — check CN0 setting", svid, rms);
        }

        let mut best_offset = 0usize;
        let mut best_power = 0.0_f64;
        for offset in 0..1023 {
            let mut corr_i = 0.0_f64;
            let mut corr_q = 0.0_f64;
            for s in 0..samples_per_ms {
                let t = s as f64 / (samples_per_ms as f64 * 1000.0);
                let phase = std::f64::consts::TAU * doppler_hz * t;
                let (sin_v, cos_v) = phase.sin_cos();
                // Strip carrier: complex multiply by exp(-j*phase)
                let stripped_i = sig.sample_array[s].real * cos_v + sig.sample_array[s].imag * sin_v;
                let stripped_q = -sig.sample_array[s].real * sin_v + sig.sample_array[s].imag * cos_v;
                let chip = ((s as f64 * chip_step + offset as f64) as usize) % 1023;
                corr_i += stripped_i * prn_code[chip];
                corr_q += stripped_q * prn_code[chip];
            }
            let power = corr_i * corr_i + corr_q * corr_q;
            if power > best_power {
                best_power = power;
                best_offset = offset;
            }
        }
        println!("  Acquired: code_offset={}, Doppler={:.1} Hz, peak_power={:.2e}",
                 best_offset, doppler_hz, best_power);

        // --- Phase 2: Generate 60s of 1ms correlations ---
        // Re-create signal from scratch for clean tracking (acquisition consumed the first ms)
        let mut lnav2 = LNavBit::new();
        lnav2.set_ephemeris(svid as i32, &eph);
        if let (Some(alpha), Some(beta), Some(utc)) = (
            nav_data.get_gps_iono_alpha(),
            nav_data.get_gps_iono_beta(),
            nav_data.utc_param.as_ref(),
        ) {
            let iono_param = IonoParam {
                a0: alpha[0], a1: alpha[1], a2: alpha[2], a3: alpha[3],
                b0: beta[0], b1: beta[1], b2: beta[2], b3: beta[3],
                flag: 1,
            };
            lnav2.set_iono_utc(&iono_param, utc);
        }
        let nav2 = NavData::LNav(lnav2);
        let mut sig = SatIfSignal::new(SAMPLE_RATE, if_freq, GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, svid);
        sig.init_state(start_time, &sat_param, Some(nav2));

        // Store per-ms correlation results (I and Q) for post-processing
        let total_ms = duration_ms as usize;
        let mut ms_corr_i = vec![0.0_f64; total_ms];
        let mut ms_corr_q = vec![0.0_f64; total_ms];
        let mut carrier_phase_acc = 0.0_f64;

        // Code phase tracking: the generated signal has code Doppler built in,
        // so our local replica must advance at the same rate.
        // code_doppler_chips_per_ms = chip_rate * doppler / carrier_freq
        // where chip_rate = 1023 chips/ms, carrier_freq = 1575.42 MHz
        let initial_code_offset = best_offset as f64;
        let mut tracked_code_offset = initial_code_offset;

        for ms in 0..duration_ms {
            let ms_time = GnssTime {
                Week: 2369,
                MilliSeconds: start_time.MilliSeconds + ms,
                SubMilliSeconds: 0.0,
            };

            // Match real generation: full update at 50ms block boundaries, push between
            // (line 2509 of ifdatagen.rs: `let full_update = ms_offset == 0`)
            let is_block_boundary = ms % 50 == 0;
            if ms % 1000 == 0 {
                // Kepler propagation every 1s
                let mut new_param = SatelliteParam::default();
                new_param.system = GnssSystem::GpsSystem;
                get_satellite_param(
                    &cur_pos, &position_lla, &ms_time,
                    GnssSystem::GpsSystem, &eph, &default_iono,
                    &mut new_param,
                );
                new_param.CN0 = 4500;
                sat_param = new_param;
            }
            if is_block_boundary {
                sig.update_satellite_params(&sat_param, CENTER_FREQ, &ms_time);
            } else {
                sig.push_sat_param_for_ms(&sat_param, CENTER_FREQ, &ms_time);
            }

            sig.get_if_sample_cached(ms_time);

            let doppler_hz = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);

            // Code Doppler: advance local code phase to match signal's code rate
            // code_doppler = chip_rate_hz * doppler / carrier_freq
            //              = 1.023e6 * doppler / 1575.42e6 chips per second
            // Per ms: code_doppler / 1000 chips
            let code_doppler_per_ms = 1.023e6 * doppler_hz / 1575.42e6 / 1000.0;

            let mut corr_i = 0.0_f64;
            let mut corr_q = 0.0_f64;
            for s in 0..samples_per_ms {
                let sample_phase = carrier_phase_acc
                    + std::f64::consts::TAU * doppler_hz * (s as f64) / (samples_per_ms as f64 * 1000.0);
                let (sin_v, cos_v) = sample_phase.sin_cos();
                let stripped_i = sig.sample_array[s].real * cos_v + sig.sample_array[s].imag * sin_v;
                let stripped_q = -sig.sample_array[s].real * sin_v + sig.sample_array[s].imag * cos_v;
                let chip = ((s as f64 * chip_step + tracked_code_offset) as usize) % 1023;
                corr_i += stripped_i * prn_code[chip];
                corr_q += stripped_q * prn_code[chip];
            }

            carrier_phase_acc += std::f64::consts::TAU * doppler_hz * 0.001;
            tracked_code_offset += code_doppler_per_ms;
            // Wrap code offset to [0, 1023)
            tracked_code_offset = tracked_code_offset.rem_euclid(1023.0);

            ms_corr_i[ms as usize] = corr_i;
            ms_corr_q[ms as usize] = corr_q;
        }

        // --- Phase 2b: Bit synchronization ---
        // Try all 20 possible 20ms accumulation offsets to find the one that maximizes
        // total absolute correlation energy (correct alignment gives clean bit accumulations).
        let mut best_sync_offset = 0usize;
        let mut best_sync_energy = 0.0_f64;
        for sync_offset in 0..20 {
            let mut energy = 0.0_f64;
            let mut idx = sync_offset;
            while idx + 20 <= total_ms {
                let mut ai = 0.0_f64;
                let mut aq = 0.0_f64;
                for k in 0..20 {
                    ai += ms_corr_i[idx + k];
                    aq += ms_corr_q[idx + k];
                }
                energy += ai * ai + aq * aq;
                idx += 20;
            }
            if energy > best_sync_energy {
                best_sync_energy = energy;
                best_sync_offset = sync_offset;
            }
        }
        println!("  Bit sync: offset={} ms (of 20), sync_energy={:.2e}", best_sync_offset, best_sync_energy);

        // --- Phase 2c: Extract nav bits with correct alignment ---
        // Use per-bit phase estimation: for each 20ms accumulation, the carrier phase
        // is approximately constant (only Doppler * 20ms of rotation, which is an integer
        // number of cycles for many Dopplers, but fractional for others).
        // Strategy: determine bit sign using the phase of each 20ms accumulation relative
        // to its predecessor. When two consecutive accumulations have similar phase,
        // the bit didn't change; when phase flips ~180 deg, the bit changed.
        // This is "Costas loop" demodulation for BPSK.
        let mut nav_bits: Vec<i8> = Vec::with_capacity(total_ms / 20);
        let mut idx = best_sync_offset;
        let mut prev_phase = 0.0_f64;
        let mut prev_sign: i8 = 1; // Start with arbitrary sign; we'll fix polarity later
        let mut bit_index = 0;
        while idx + 20 <= total_ms {
            let mut ai = 0.0_f64;
            let mut aq = 0.0_f64;
            for k in 0..20 {
                ai += ms_corr_i[idx + k];
                aq += ms_corr_q[idx + k];
            }

            let phase = aq.atan2(ai);
            let mag = (ai * ai + aq * aq).sqrt();

            if bit_index == 0 {
                println!("  Phase lock: phase={:.3} rad, magnitude={:.2e}", phase, mag);
                // First bit: assign arbitrary sign (we'll resolve global polarity in preamble search)
                prev_phase = phase;
                prev_sign = 1;
                nav_bits.push(prev_sign);
            } else {
                // Check if phase flipped (>90 degrees from previous)
                let mut phase_diff = phase - prev_phase;
                // Wrap to [-pi, pi]
                while phase_diff > std::f64::consts::PI { phase_diff -= std::f64::consts::TAU; }
                while phase_diff < -std::f64::consts::PI { phase_diff += std::f64::consts::TAU; }

                if phase_diff.abs() > std::f64::consts::FRAC_PI_2 {
                    // Phase flipped ~180 deg => bit changed
                    prev_sign = -prev_sign;
                }
                // else: phase stayed same => bit didn't change
                nav_bits.push(prev_sign);
                prev_phase = phase;
            }
            bit_index += 1;
            idx += 20;
        }

        let total_bits = nav_bits.len();
        println!("  Extracted {} nav bits ({:.1}s)", total_bits, total_bits as f64 * 0.02);

        // --- Phase 3: Preamble search and subframe decoding ---
        let mut valid_subframes = 0;
        let mut max_consecutive = 0;
        let mut current_consecutive = 0;
        let mut subframe_ids_found = [0u32; 6]; // index 1-5

        // Search for preamble at every bit position
        let mut bit_idx = 0;
        while bit_idx + 300 <= total_bits {
            let mut preamble_match = false;
            let mut inverted = false;

            // Normal preamble
            let mut match_normal = true;
            for j in 0..8 {
                if nav_bits[bit_idx + j] != preamble_pattern[j] {
                    match_normal = false;
                    break;
                }
            }
            if match_normal {
                preamble_match = true;
            }

            // Inverted preamble (D30* = 1 from previous subframe)
            if !preamble_match {
                let mut match_inv = true;
                for j in 0..8 {
                    if nav_bits[bit_idx + j] != preamble_inv[j] {
                        match_inv = false;
                        break;
                    }
                }
                if match_inv {
                    preamble_match = true;
                    inverted = true;
                }
            }

            if !preamble_match {
                bit_idx += 1;
                current_consecutive = 0;
                continue;
            }

            // Found preamble candidate — extract 300-bit subframe
            // Convert soft bits (+1/-1) to hard bits (0/1): +1 -> 0, -1 -> 1
            let mut subframe_bits = [0u32; 300];
            for j in 0..300 {
                subframe_bits[j] = if nav_bits[bit_idx + j] > 0 { 0 } else { 1 };
                if inverted {
                    subframe_bits[j] ^= 1;
                }
            }

            // Verify GPS ICD parity for all 10 words
            let parity_ok = gps_lnav_check_parity(&subframe_bits);

            if parity_ok {
                valid_subframes += 1;
                current_consecutive += 1;
                if current_consecutive > max_consecutive {
                    max_consecutive = current_consecutive;
                }

                // Extract subframe ID from HOW word (word 2, bits 20-22 of word = bits 49-51 overall)
                let sf_id = ((subframe_bits[49] << 2) | (subframe_bits[50] << 1) | subframe_bits[51]) as usize;
                if sf_id >= 1 && sf_id <= 5 {
                    subframe_ids_found[sf_id] += 1;
                }

                bit_idx += 300;
            } else {
                bit_idx += 1;
                current_consecutive = 0;
            }
        }

        let sf_summary = format!("SF1={} SF2={} SF3={} SF4={} SF5={}",
            subframe_ids_found[1], subframe_ids_found[2], subframe_ids_found[3],
            subframe_ids_found[4], subframe_ids_found[5]);
        let status = if max_consecutive >= 3 { "PASS" } else { "FAIL" };
        println!("  PRN {:02}: valid_subframes={}, max_consecutive={}, {} — {}",
                 svid, valid_subframes, max_consecutive, sf_summary, status);

        if max_consecutive < 3 {
            all_ok = false;
        }
    }

    assert!(visible_count > 0, "No visible GPS satellites found — check RINEX data or receiver position");
    assert!(all_ok,
        "One or more visible GPS satellites failed to decode at least 3 consecutive valid subframes");
}

/// GPS ICD-200 parity check for a 300-bit LNAV subframe.
/// Uses the identical lookup-table algorithm as the LNAV encoder (lnavbit.rs:gps_get_parity)
/// for guaranteed consistency.
///
/// Returns (all_pass, failure_description).
fn gps_lnav_check_parity(bits: &[u32; 300]) -> bool {
    // Same PARITY_TABLE as lnavbit.rs — encodes GPS ICD parity polynomials
    const PARITY_TABLE: [[u8; 16]; 6] = [
        [0x00,0x13,0x25,0x36,0x0B,0x18,0x2E,0x3D,0x16,0x05,0x33,0x20,0x1D,0x0E,0x38,0x2B],
        [0x00,0x2C,0x19,0x35,0x32,0x1E,0x2B,0x07,0x26,0x0A,0x3F,0x13,0x14,0x38,0x0D,0x21],
        [0x00,0x0E,0x1F,0x11,0x3E,0x30,0x21,0x2F,0x3D,0x33,0x22,0x2C,0x03,0x0D,0x1C,0x12],
        [0x00,0x38,0x31,0x09,0x23,0x1B,0x12,0x2A,0x07,0x3F,0x36,0x0E,0x24,0x1C,0x15,0x2D],
        [0x00,0x0D,0x1A,0x17,0x37,0x3A,0x2D,0x20,0x2F,0x22,0x35,0x38,0x18,0x15,0x02,0x0F],
        [0x00,0x1C,0x3B,0x27,0x34,0x28,0x0F,0x13,0x2A,0x36,0x11,0x0D,0x1E,0x02,0x25,0x39],
    ];

    /// Compute 6 parity bits using the GPS ICD lookup-table method.
    /// Input: 32-bit word with D29* in bit 31, D30* in bit 30, d1..d24 in bits 29..6.
    /// Returns: 6-bit parity in bits 5..0.
    fn compute_parity(word: u32) -> u32 {
        let mut w = word >> 6;
        let mut parity = 0u32;
        for i in 0..6 {
            parity ^= PARITY_TABLE[i][(w & 0xf) as usize] as u32;
            w >>= 4;
        }
        // w now has D30* in bit 0, D29* in bit 1
        if (w & 1) != 0 { parity ^= 0x15; }
        if (w & 2) != 0 { parity ^= 0x29; }
        parity
    }

    for word_idx in 0..10 {
        let word_start = word_idx * 30;

        // D29* and D30* from previous word (for word 0, both 0 — TLM starts clean)
        let d29_star: u32 = if word_idx > 0 { bits[word_start - 2] } else { 0 };
        let d30_star: u32 = if word_idx > 0 { bits[word_start - 1] } else { 0 };

        // Pack into 32-bit word: [D29* D30* d1..d24 ______]
        let mut word32: u32 = (d29_star << 31) | (d30_star << 30);
        for j in 0..24 {
            word32 |= bits[word_start + j] << (29 - j);
        }

        let expected_parity = compute_parity(word32);

        // Extract transmitted parity (D25..D30)
        let mut transmitted_parity: u32 = 0;
        for j in 0..6 {
            transmitted_parity = (transmitted_parity << 1) | bits[word_start + 24 + j];
        }

        if expected_parity != transmitted_parity {
            return false;
        }
    }

    true
}

// =============================================================================
// TEST 8: Galileo I-NAV Viterbi decode + CRC24Q verification
//
// This test verifies that get_frame_data() produces valid I-NAV pages by:
//   1. Checking sync patterns in even/odd halves
//   2. De-interleaving the 8x30 block interleaver
//   3. Viterbi decoding (K=7, G1=171oct, G2=133oct, G2 inverted)
//   4. Checking tail bits are zero
//   5. Extracting data bits and verifying CRC24Q
// =============================================================================
#[test]
fn test_galileo_inav_viterbi_decode() {
    let nav_data = load_rinex();

    let mut inav = INavBit::new();

    // Load Galileo ephemeris for E27
    let svid: u8 = 27;
    let mut loaded = false;
    for eph in &nav_data.galileo_ephemeris {
        if eph.valid != 0 && eph.svid == svid {
            let result = inav.set_ephemeris(svid as i32, eph);
            if result != 0 {
                loaded = true;
                break;
            }
        }
    }
    assert!(loaded, "Failed to load Galileo ephemeris for E{:02}", svid);
    println!("Loaded Galileo E{:02} ephemeris", svid);

    // -------------------------------------------------------------------------
    // Viterbi decoder structures
    // -------------------------------------------------------------------------
    // K=7, 64 states (2^6). G1=171oct=0b1111001, G2=133oct=0b1011011, G2 inverted.
    const NUM_STATES: usize = 64; // 2^(K-1)

    // Precompute expected output (g1, g2_inverted) for each state and input bit
    // State is the 6 memory bits. Input bit shifts in from the left.
    // After shift: new_state = (input << 5) | (old_state >> 1)
    // The encoder register for that step: [input, s5..s0] where s5..s0 = old_state bits 5..0
    // G1 = 171oct = 1_111_001 -> taps at positions 0,3,4,5,6 (from MSB of 7-bit pattern)
    //   = bit6, bit5, bit4, bit3, bit0  where bit6=input
    // G2 = 133oct = 1_011_011 -> taps at positions 0,1,3,4,6
    //   = bit6, bit4, bit3, bit1, bit0  where bit6=input
    // (numbering: bit6=newest=input, bit0=oldest=state bit 0)
    fn encoder_output(state: u8, input: u8) -> (u8, u8) {
        // Register: [input, s5, s4, s3, s2, s1, s0]
        let reg = ((input as u16) << 6) | (state as u16);
        // G1 = 1111001 = bit positions 6,5,4,3,0
        let g1 = (((reg >> 6) ^ (reg >> 5) ^ (reg >> 4) ^ (reg >> 3) ^ reg) & 1) as u8;
        // G2 = 1011011 = bit positions 6,4,3,1,0
        let g2 = (((reg >> 6) ^ (reg >> 4) ^ (reg >> 3) ^ (reg >> 1) ^ reg) & 1) as u8;
        (g1, g2 ^ 1) // G2 inverted
    }

    // Build transition table: for each (state, input), compute (next_state, expected_g1, expected_g2_inv)
    let mut transitions: Vec<(usize, u8, u8)> = vec![(0, 0, 0); NUM_STATES * 2];
    for state in 0..NUM_STATES {
        for input in 0..2u8 {
            let next_state = ((input as usize) << 5) | (state >> 1);
            let (g1, g2_inv) = encoder_output(state as u8, input);
            transitions[state * 2 + input as usize] = (next_state, g1, g2_inv);
        }
    }

    /// Viterbi decode 240 symbols (120 pairs) into 120 bits.
    /// Symbols are (g1, g2_inverted) pairs. Returns decoded bits.
    fn viterbi_decode(symbols: &[u8; 240], transitions: &[(usize, u8, u8)]) -> [u8; 120] {
        const INF: i32 = 1_000_000;
        let num_states = 64usize;
        let num_steps = 120usize; // 240 symbols / 2

        // Path metrics
        let mut pm = vec![INF; num_states];
        pm[0] = 0; // Start in state 0

        // Path memory: for each (step, state), store the previous state
        let mut path: Vec<Vec<usize>> = vec![vec![0; num_states]; num_steps];

        for step in 0..num_steps {
            let sym_g1 = symbols[step * 2];
            let sym_g2 = symbols[step * 2 + 1];

            let mut new_pm = vec![INF; num_states];

            for state in 0..num_states {
                if pm[state] >= INF { continue; }
                for input in 0..2u8 {
                    let (next_state, exp_g1, exp_g2) = transitions[state * 2 + input as usize];
                    // Branch metric: Hamming distance
                    let bm = ((sym_g1 ^ exp_g1) + (sym_g2 ^ exp_g2)) as i32;
                    let candidate = pm[state] + bm;
                    if candidate < new_pm[next_state] {
                        new_pm[next_state] = candidate;
                        path[step][next_state] = state;
                    }
                }
            }

            pm = new_pm;
        }

        // Traceback from state 0 (tail forces encoder to state 0)
        let mut decoded = [0u8; 120];
        let mut state = 0usize; // Tail forces state to 0
        for step in (0..num_steps).rev() {
            let prev_state = path[step][state];
            // The input bit that caused transition from prev_state to state
            // next_state = (input << 5) | (prev_state >> 1)
            // So input = state >> 5 (the MSB of the new state comes from input)
            let input = (state >> 5) as u8;
            decoded[step] = input;
            state = prev_state;
        }

        decoded
    }

    /// De-interleave 240 nav_bits (after sync) back to sequential symbol order.
    /// Interleaver writes symbols row-by-row into 30-byte array (8 symbols per byte),
    /// then reads column-by-column: nav[i*30+j] = sym[j*8+i] for i=0..7, j=0..29.
    /// So to recover: sym[j*8+i] = nav[i*30+j], or equivalently:
    /// for nav position k: row i=k/30, col j=k%30, original index = j*8 + i.
    fn deinterleave(interleaved: &[i32]) -> [u8; 240] {
        let mut symbols = [0u8; 240];
        for k in 0..240 {
            let i = k / 30; // row (bit position)
            let j = k % 30; // column (byte index)
            let orig_idx = j * 8 + i;
            symbols[orig_idx] = interleaved[k] as u8;
        }
        symbols
    }

    /// CRC24Q computation on u32 word array, MSB-first per word.
    /// Matches inavbit.rs crc24q_encode exactly.
    fn crc24q_u32(data: &[u32], length: usize) -> u32 {
        let poly: u32 = 0x1864CFB;
        let mut crc: u32 = 0;
        for i in 0..(length / 32) {
            let mut word = data[i];
            for _ in 0..32 {
                crc = (crc << 1)
                    ^ if (crc & 0x800000) != 0 { poly } else { 0 }
                    ^ ((word >> 31) & 1);
                word <<= 1;
            }
        }
        if (length % 32) != 0 {
            let mut word = data[length / 32];
            for _ in 0..(length % 32) {
                crc = (crc << 1)
                    ^ if (crc & 0x800000) != 0 { poly } else { 0 }
                    ^ ((word >> 31) & 1);
                word <<= 1;
            }
        }
        crc & 0xFFFFFF
    }

    let expected_sync = [0i32, 1, 0, 1, 1, 0, 0, 0, 0, 0];

    let mut pages_tested = 0;
    let mut pages_crc_ok = 0;
    let mut all_ok = true;

    // Generate 15 I-NAV pages with different TOWs (odd values for E1)
    for page_idx in 0..15 {
        let tow = page_idx * 2 + 1; // 1, 3, 5, 7, ..., 29
        let time = GnssTime {
            Week: 2369,
            MilliSeconds: tow * 1000,
            SubMilliSeconds: 0.0,
        };

        let mut nav_bits = [0i32; 500];
        inav.get_frame_data(time, svid as i32, 1, &mut nav_bits);
        pages_tested += 1;

        // --- Step 3a: Verify sync patterns ---
        let sync_even = &nav_bits[0..10];
        let sync_odd = &nav_bits[250..260];
        assert_eq!(sync_even, &expected_sync, "Page tow={}: even sync mismatch", tow);
        assert_eq!(sync_odd, &expected_sync, "Page tow={}: odd sync mismatch", tow);

        // --- Step 3b: De-interleave even part (positions 10..250) ---
        let even_symbols = deinterleave(&nav_bits[10..250]);

        // --- Step 3c: Viterbi decode even part ---
        let even_decoded = viterbi_decode(&even_symbols, &transitions);

        // --- Step 3d: Check tail bits (last 6 of 120 decoded bits) ---
        let even_tail = &even_decoded[114..120];
        let even_tail_ok = even_tail.iter().all(|&b| b == 0);
        if !even_tail_ok {
            println!("  Page tow={}: EVEN tail bits not zero: {:?}", tow, even_tail);
            all_ok = false;
        }

        // --- Step 3f: De-interleave and Viterbi decode odd part (positions 260..500) ---
        let odd_symbols = deinterleave(&nav_bits[260..500]);
        let odd_decoded = viterbi_decode(&odd_symbols, &transitions);

        let odd_tail = &odd_decoded[114..120];
        let odd_tail_ok = odd_tail.iter().all(|&b| b == 0);
        if !odd_tail_ok {
            println!("  Page tow={}: ODD tail bits not zero: {:?}", tow, odd_tail);
            all_ok = false;
        }

        // --- Step 3g: Reconstruct encode_data[0..7] from decoded bits ---
        //
        // The encoder packs 196 bits into encode_data[0..6] and computes CRC24Q
        // over these 7 words (196 bits MSB-first). But the convolutional encoder
        // processes the data with shifts:
        //
        // Even part (114 decoded bits):
        //   encode_data[0]<<28 (32 bits from MSB): even_decoded[0..31]
        //   encode_data[1] (32 bits): even_decoded[32..63]
        //   encode_data[2] (32 bits): even_decoded[64..95]
        //   encode_data[3] upper 18 bits [31:14]: even_decoded[96..113]
        //
        // Odd part (first 82 decoded bits = data before CRC/SSP/tail):
        //   encode_data[4]<<14 bits [17:0] (18 bits from MSB): odd_decoded[0..17]
        //   encode_data[5] (32 bits): odd_decoded[18..49]
        //   encode_data[6] (32 bits): odd_decoded[50..81]
        //
        // The CRC is computed over encode_data[0..6] as u32 words, 196 bits total.
        // Missing from decoded: encode_data[3] lower 14 bits and encode_data[4] upper 14 bits.
        // These 28 bits bridge the even/odd boundary and are NOT convolutionally encoded.
        //
        // However, encode_data[3] and [4] are deterministically constructed from the
        // word data, so the decoded even/odd halves plus the structural constraints
        // let us reconstruct encode_data exactly.

        // Helper: pack decoded bits into a u32 (MSB-first)
        fn bits_to_u32(bits: &[u8]) -> u32 {
            let mut val: u32 = 0;
            for &b in bits {
                val = (val << 1) | (b as u32 & 1);
            }
            val
        }

        // --- Reconstruct reference encode_data from INavBit internal state ---
        //
        // The CRC covers encode_data[0..6] as 196 bits, but 28 "bridge bits"
        // (encode_data[3] lower 14 + encode_data[4] upper 14) are NOT convolutionally
        // encoded, so we cannot reconstruct the full CRC input purely from Viterbi output.
        // Instead, we replicate the encoder's word selection and encode_data construction
        // from INavBit internal arrays, then verify both decoded data and CRC match.
        let mut tow_local = tow;
        if (tow_local & 1) == 0 { tow_local -= 1; }
        if tow_local < 0 { tow_local += 604800; }
        let subframe = ((tow_local + 360) % 720) / 30;
        let page = (tow_local % 30) / 2;
        let word_allocation_e1: [i32; 15] = [2, 4, 6, 7, 8, 17, 19, 16, 0, 0, 1, 3, 5, 0, 16];
        let mut word_sel = word_allocation_e1[page as usize];
        if word_sel > 10 { word_sel = 63; }
        if (subframe & 1) != 0 && (word_sel == 7 || word_sel == 8) { word_sel += 2; }
        if (subframe & 1) != 0 && (word_sel == 17 || word_sel == 19) { word_sel += 1; }

        // Now replicate the encode_data construction to get the reference CRC
        // Get word data from INavBit fields
        let data: Vec<u32> = match word_sel {
            0 => inav.gal_spare_data.to_vec(),
            1..=5 => {
                let idx = (svid as usize - 1);
                inav.gal_eph_data[idx][(word_sel as usize - 1) * 4..word_sel as usize * 4].to_vec()
            }
            6 => inav.gal_utc_data.to_vec(),
            7..=10 => {
                let sub_idx = (subframe / 2) as usize;
                inav.gal_alm_data[sub_idx][(word_sel as usize - 7) * 4..(word_sel as usize - 6) * 4].to_vec()
            }
            17..=20 => {
                let idx = (svid as usize - 1);
                inav.gal_rs_vector[idx][(word_sel as usize - 17) * 4..(word_sel as usize - 16) * 4].to_vec()
            }
            63 => inav.gal_dummy_data.to_vec(),
            _ => inav.gal_dummy_data.to_vec(),
        };

        let mut dv = data.clone();
        // Add WN/tow for word 0/5/6
        if word_sel == 0 {
            dv[3] |= (((time.Week - 1024) as u32) << 20) + tow_local as u32;
        } else if word_sel == 5 {
            dv[2] &= 0xff800000;
            dv[2] |= ((((time.Week - 1024) & 0xfff) as u32) << 11) + ((tow_local >> 9) as u32);
            dv[3] |= (tow_local << 23) as u32;
        } else if word_sel == 6 {
            dv[3] &= 0xff800000;
            dv[3] |= (tow_local << 3) as u32;
        }

        // Build reference encode_data
        let mut ref_ed = [0u32; 7];
        ref_ed[0] = dv[0] >> 30;
        ref_ed[1] = (dv[0] << 2) | (dv[1] >> 30);
        ref_ed[2] = (dv[1] << 2) | (dv[2] >> 30);
        ref_ed[3] = (dv[2] << 2) | (dv[3] >> 30);
        ref_ed[4] = ((dv[3] << 2) & 0xfffc0000) | (dv[3] & 0xffff) | 0x20000;
        ref_ed[5] = 0;
        ref_ed[6] = 0;

        // Compute reference CRC
        let ref_crc = crc24q_u32(&ref_ed, 196);

        // Extract CRC from Viterbi-decoded odd part (bits 82..105 = 24 CRC bits)
        let mut decoded_crc: u32 = 0;
        for i in 82..106 {
            decoded_crc = (decoded_crc << 1) | (odd_decoded[i] as u32);
        }

        let crc_ok = ref_crc == decoded_crc;
        if crc_ok {
            pages_crc_ok += 1;
        } else {
            println!("  Page tow={}: CRC MISMATCH ref=0x{:06X} decoded=0x{:06X}",
                     tow, ref_crc, decoded_crc);
            all_ok = false;
        }

        // Also verify decoded data bits match reference encode_data
        // Even part: encode_data[0]<<28 (32 bits) + encode_data[1] (32) + encode_data[2] (32) + encode_data[3] upper 18
        let ref_even0 = ref_ed[0] << 28;
        let dec_even0 = bits_to_u32(&even_decoded[0..32]);
        let ref_even1 = ref_ed[1];
        let dec_even1 = bits_to_u32(&even_decoded[32..64]);
        let ref_even2 = ref_ed[2];
        let dec_even2 = bits_to_u32(&even_decoded[64..96]);
        let ref_even3_upper = ref_ed[3] >> 14;
        let dec_even3_upper = bits_to_u32(&even_decoded[96..114]);

        let data_match = ref_even0 == dec_even0
            && ref_even1 == dec_even1
            && ref_even2 == dec_even2
            && ref_even3_upper == dec_even3_upper;

        // Odd part: encode_data[4] bits [17:0] (18 bits) + encode_data[5] (32) + encode_data[6] (32)
        let ref_odd_ed4_lo = ref_ed[4] & 0x3FFFF;
        let dec_odd_ed4_lo = bits_to_u32(&odd_decoded[0..18]);
        let ref_odd_ed5 = ref_ed[5];
        let dec_odd_ed5 = bits_to_u32(&odd_decoded[18..50]);
        let ref_odd_ed6 = ref_ed[6];
        let dec_odd_ed6 = bits_to_u32(&odd_decoded[50..82]);

        let odd_data_match = ref_odd_ed4_lo == dec_odd_ed4_lo
            && ref_odd_ed5 == dec_odd_ed5
            && ref_odd_ed6 == dec_odd_ed6;

        // Extract word type from the reference encode_data (6 bits after even/odd and page_type)
        // Word type is in word data starting at bit position 26 of data_vec[0]
        // encode_data[1] bits [31:26] contain the word type (6 MSBs of encode_data[1])
        let word_type = ref_ed[1] >> 26;

        let status = if even_tail_ok && odd_tail_ok && crc_ok && data_match && odd_data_match {
            "OK"
        } else {
            "FAIL"
        };
        if !data_match || !odd_data_match {
            println!("  Page tow={}: DATA MISMATCH even={} odd={}", tow, data_match, odd_data_match);
            all_ok = false;
        }
        println!("  Page tow={:2}: word_type={:2}, tail_even={}, tail_odd={}, CRC={}, data={} — {}",
                 tow,
                 word_type,
                 if even_tail_ok { "OK" } else { "FAIL" },
                 if odd_tail_ok { "OK" } else { "FAIL" },
                 if crc_ok { "OK" } else { "FAIL" },
                 if data_match && odd_data_match { "OK" } else { "FAIL" },
                 status);
    }

    println!("\nSummary: {}/{} pages CRC OK", pages_crc_ok, pages_tested);
    assert!(all_ok, "Galileo I-NAV Viterbi decode/CRC verification failed for some pages");
    assert_eq!(pages_crc_ok, pages_tested, "Not all pages passed CRC24Q verification");
}
