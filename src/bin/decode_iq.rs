//! GPS L1CA IQ8 file decoder.
//!
//! Reads a .C8 file, acquires GPS satellites, tracks carrier/code,
//! extracts navigation bits, and verifies LNAV parity.
//!
//! Usage: cargo run --release --bin decode_iq [file.C8]

use gnss_rust::constants::SIGNAL_INDEX_L1CA;
use gnss_rust::coordinate::{ecef_to_lla, lla_to_ecef};
use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use gnss_rust::nav_decode::verify_lnav_parity;
use gnss_rust::prngenerate::PrnGenerate;
use gnss_rust::satellite_param::{get_doppler, get_satellite_param, get_travel_time};
use gnss_rust::types::*;
use rayon::prelude::*;
use std::f64::consts::TAU;

const SAMPLE_RATE: f64 = 5_000_000.0;
const SAMPLES_PER_MS: usize = 5000;
const CODE_LEN: usize = 1023;
const CHIP_STEP: f64 = CODE_LEN as f64 / SAMPLES_PER_MS as f64;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let file_path = args
        .get(1)
        .map(|s| s.as_str())
        .unwrap_or("generated_files/gps_pilot.C8");

    println!("GPS L1CA IQ8 Decoder");
    println!("File: {}", file_path);

    let raw = std::fs::read(file_path).expect("Cannot read file");
    let total_samples = raw.len() / 2;
    let duration_s = total_samples as f64 / SAMPLE_RATE;
    let total_ms = total_samples / SAMPLES_PER_MS;

    println!(
        "Samples: {}, Duration: {:.1}s\n",
        total_samples, duration_s
    );

    let iq: Vec<(f64, f64)> = raw
        .chunks(2)
        .map(|c| (c[0] as i8 as f64, c[1] as i8 as f64))
        .collect();

    // Parallel acquisition of all 32 PRNs
    println!("=== ACQUISITION (parallel) ===");
    let start = std::time::Instant::now();

    let mut all_results: Vec<(u8, usize, f64, f64)> = (1u8..=32)
        .into_par_iter()
        .map(|svid| {
            let prn_code = get_prn_code(svid);
            let (offset, doppler, z) = acquire_serial(&iq, &prn_code);
            (svid, offset, doppler, z)
        })
        .collect();
    all_results.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap());

    // Show top 15 results for debugging
    for &(svid, offset, doppler, z) in all_results.iter().take(15) {
        let mark = if z > 8.0 { "<<<" } else { "" };
        println!("  PRN {:02}: offset={:4}, doppler={:+7.0} Hz, z={:.1} {}", svid, offset, doppler, z, mark);
    }

    let results: Vec<(u8, usize, f64, f64)> = all_results.into_iter().filter(|r| r.3 > 2.8).collect();

    println!("Acquisition done in {:.1}s", start.elapsed().as_secs_f64());
    for &(svid, offset, doppler, z) in &results {
        println!(
            "  PRN {:02}: offset={:4}, doppler={:+7.0} Hz, z={:.1}",
            svid, offset, doppler, z
        );
    }
    println!("\nAcquired: {} satellites\n", results.len());

    if results.is_empty() {
        println!("No satellites found.");
        return;
    }

    // Track and decode each satellite
    println!("=== TRACKING & NAV DECODE ===");
    let mut total_decoded = 0;

    for &(svid, code_offset, doppler, _z) in &results {
        let prn_code = get_prn_code(svid);
        let result = track_and_decode(&iq, &prn_code, svid, code_offset, doppler, total_ms);

        if result.subframes_valid >= 3 {
            total_decoded += 1;
        }
    }

    println!("\n=== SUMMARY ===");
    println!("Acquired: {} satellites", results.len());
    println!(
        "Decoded:  {} satellites with ≥3 valid subframes",
        total_decoded
    );
}

fn get_prn_code(svid: u8) -> Vec<f64> {
    let prn = PrnGenerate::new(
        GnssSystem::GpsSystem,
        SIGNAL_INDEX_L1CA as i32,
        svid as i32,
    );
    prn.get_data_prn()
        .unwrap()
        .iter()
        .map(|&v| if v != 0 { -1.0 } else { 1.0 })
        .collect()
}

/// Serial acquisition: for each Doppler bin, strip carrier for 1ms then correlate
/// with all 1023 code offsets. Non-coherent accumulation over 10ms.
fn acquire_serial(iq: &[(f64, f64)], prn_code: &[f64]) -> (usize, f64, f64) {
    let mut best_power = 0.0_f64;
    let mut best_doppler = 0.0_f64;
    let mut best_offset = 0usize;
    let mut noise_sum = 0.0_f64;
    let mut noise_count = 0usize;

    // Doppler search: -5000 to +5000 Hz, step 250 Hz
    for doppler_bin in -20i32..=20 {
        let doppler = doppler_bin as f64 * 250.0;

        // 1ms coherent × 10ms non-coherent (handles nav bit transitions)
        let mut power_per_offset = vec![0.0f64; CODE_LEN];

        for ms in 0..10 {
            // Pre-compute carrier-stripped samples for this ms
            let mut si_arr = vec![0.0f64; SAMPLES_PER_MS];
            let mut sq_arr = vec![0.0f64; SAMPLES_PER_MS];
            for s in 0..SAMPLES_PER_MS {
                let idx = ms * SAMPLES_PER_MS + s;
                if idx >= iq.len() { break; }
                let t = idx as f64 / SAMPLE_RATE;
                let phase = TAU * doppler * t;
                let (sv, cv) = phase.sin_cos();
                si_arr[s] = iq[idx].0 * cv + iq[idx].1 * sv;
                sq_arr[s] = -iq[idx].0 * sv + iq[idx].1 * cv;
            }

            for offset in 0..CODE_LEN {
                let mut ci = 0.0_f64;
                let mut cq = 0.0_f64;
                for s in 0..SAMPLES_PER_MS {
                    let chip = ((s as f64 * CHIP_STEP + offset as f64) as usize) % CODE_LEN;
                    ci += si_arr[s] * prn_code[chip];
                    cq += sq_arr[s] * prn_code[chip];
                }
                power_per_offset[offset] += ci * ci + cq * cq;
            }
        }

        for (offset, &power) in power_per_offset.iter().enumerate() {
            noise_sum += power;
            noise_count += 1;
            if power > best_power {
                best_power = power;
                best_doppler = doppler;
                best_offset = offset;
            }
        }
    }

    let mean = noise_sum / noise_count as f64;
    let z = (best_power / mean).sqrt();
    (best_offset, best_doppler, z)
}

struct DecodeResult {
    subframes_valid: usize,
}

fn track_and_decode(
    iq: &[(f64, f64)],
    prn_code: &[f64],
    svid: u8,
    code_offset: usize,
    _doppler: f64,
    total_ms: usize,
) -> DecodeResult {
    // Load RINEX for exact Doppler per second
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx", 10000);
    let gps_time = gnss_rust::gnsstime::utc_to_gps_time(
        UtcTime { Year: 2025, Month: 6, Day: 5, Hour: 10, Minute: 5, Second: 30.0 },
        true,
    );
    let target_sow = gps_time.MilliSeconds as f64 / 1000.0;
    let eph = nav_data.gps_ephemeris.iter()
        .filter(|e| e.svid == svid && e.valid != 0)
        .min_by(|a, b| (a.toe as f64 - target_sow).abs().partial_cmp(&(b.toe as f64 - target_sow).abs()).unwrap())
        .copied()
        .expect("No ephemeris");

    let receiver_lla = LlaPosition {
        lat: 48.4928_f64.to_radians(),
        lon: (-114.2847_f64).to_radians(),
        alt: 100.0,
    };
    let receiver_ecef = lla_to_ecef(&receiver_lla);
    let receiver_lla_conv = ecef_to_lla(&receiver_ecef);
    let iono = IonoParam::default();

    // 1. Per-ms prompt correlation with orbital Doppler
    let mut ms_prompt = vec![0.0_f64; total_ms];
    let mut carrier_phase = 0.0_f64;
    let mut current_doppler = _doppler;
    let mut tracked_offset = code_offset as f64;

    for ms in 0..total_ms {
        // Update Doppler from ephemeris every 100ms
        if ms % 100 == 0 {
            let ms_time = GnssTime {
                Week: gps_time.Week,
                MilliSeconds: gps_time.MilliSeconds + ms as i32,
                SubMilliSeconds: 0.0,
            };
            let mut sat_param = SatelliteParam::default();
            sat_param.system = GnssSystem::GpsSystem;
            get_satellite_param(
                &receiver_ecef, &receiver_lla_conv, &ms_time,
                GnssSystem::GpsSystem, &eph, &iono, &mut sat_param,
            );
            current_doppler = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);
        }

        let carrier_per_sample = current_doppler / SAMPLE_RATE;
        let code_doppler_per_ms = 1_023_000.0 * current_doppler / 1_575_420_000.0 / 1000.0;

        let mut ci = 0.0_f64;
        for s in 0..SAMPLES_PER_MS {
            let idx = ms * SAMPLES_PER_MS + s;
            if idx >= iq.len() { break; }
            let angle = carrier_phase * TAU;
            let (sv, cv) = angle.sin_cos();
            carrier_phase += carrier_per_sample;
            let si = iq[idx].0 * cv + iq[idx].1 * sv;
            let chip = ((s as f64 * CHIP_STEP + tracked_offset) as usize) % CODE_LEN;
            ci += si * prn_code[chip];
        }
        ms_prompt[ms] = ci;

        tracked_offset += code_doppler_per_ms;
        tracked_offset = tracked_offset.rem_euclid(CODE_LEN as f64);
    }

    // 2. Bit sync
    let mut best_sync = 0usize;
    let mut best_energy = 0.0_f64;
    for sync in 0..20 {
        let mut energy = 0.0;
        let mut idx = sync;
        while idx + 20 <= total_ms {
            let sum: f64 = ms_prompt[idx..idx + 20].iter().sum();
            energy += sum * sum;
            idx += 20;
        }
        if energy > best_energy {
            best_energy = energy;
            best_sync = sync;
        }
    }

    // 3. Extract nav bits
    let mut nav_bits: Vec<i8> = Vec::new();
    let mut idx = best_sync;
    while idx + 20 <= total_ms {
        let sum: f64 = ms_prompt[idx..idx + 20].iter().sum();
        nav_bits.push(if sum > 0.0 { 1 } else { -1 });
        idx += 20;
    }

    // Debug: print first 30 bits and prompt magnitudes
    if nav_bits.len() >= 30 {
        let bits_str: String = nav_bits[..30].iter().map(|&b| if b > 0 { '0' } else { '1' }).collect();
        println!("  First 30 bits: {}", bits_str);
        let mut mags = Vec::new();
        let mut idx_d = best_sync;
        for _ in 0..10 {
            if idx_d + 20 > total_ms { break; }
            let sum: f64 = ms_prompt[idx_d..idx_d + 20].iter().sum();
            mags.push(format!("{:.0}", sum.abs()));
            idx_d += 20;
        }
        println!("  Prompt magnitudes: [{}]", mags.join(", "));
    }

    // 4. Preamble search + parity check using proven verify_lnav_parity
    // Convert i8 nav_bits to i32 format expected by verify_lnav_parity
    // Convention: nav_bit=+1 (positive prompt) → transmitted 0, nav_bit=-1 → transmitted 1
    // verify_lnav_parity expects bits as 0/1 i32 values

    let preamble_normal: [i8; 8] = [-1, 1, 1, 1, -1, 1, -1, -1]; // preamble 10001011
    let preamble_inv: [i8; 8] = [1, -1, -1, -1, 1, -1, 1, 1];

    let mut subframes_valid = 0;
    let mut subframes_total = 0;

    let mut i = 0;
    while i + 300 <= nav_bits.len() {
        let match_n = (0..8).all(|k| nav_bits[i + k] == preamble_normal[k]);
        let match_i = (0..8).all(|k| nav_bits[i + k] == preamble_inv[k]);

        if !match_n && !match_i {
            i += 1;
            continue;
        }

        let inverted = match_i;

        // Convert to i32 bits (0 or 1) for verify_lnav_parity
        let mut nav_bits_i32 = [0i32; 300];
        for k in 0..300 {
            let bit = if inverted { -nav_bits[i + k] } else { nav_bits[i + k] };
            nav_bits_i32[k] = if bit < 0 { 1 } else { 0 };
        }

        let parity_results = verify_lnav_parity(&nav_bits_i32);
        let all_ok = parity_results.iter().all(|(_, ok)| *ok);
        subframes_total += 1;

        if all_ok {
            subframes_valid += 1;
            // Extract subframe info from raw words
            let mut word2 = 0u32;
            for k in 0..30 {
                word2 = (word2 << 1) | (nav_bits_i32[30 + k] as u32 & 1);
            }
            // Un-XOR if D30* of word 1 is set
            let d30_star_w1 = nav_bits_i32[29] as u32 & 1;
            let how_data = if d30_star_w1 == 1 {
                ((word2 >> 6) ^ 0xFFFFFF) & 0xFFFFFF
            } else {
                (word2 >> 6) & 0xFFFFFF
            };
            let sf_id = (how_data >> 2) & 0x7;
            let tow = (how_data >> 7) & 0x1FFFF;
            print!("  SF{} TOW={:6}s", sf_id, tow * 6);
            if sf_id == 1 {
                let mut word3 = 0u32;
                for k in 0..30 {
                    word3 = (word3 << 1) | (nav_bits_i32[60 + k] as u32 & 1);
                }
                let d30_star_w2 = nav_bits_i32[59] as u32 & 1;
                let w3_data = if d30_star_w2 == 1 {
                    ((word3 >> 6) ^ 0xFFFFFF) & 0xFFFFFF
                } else {
                    (word3 >> 6) & 0xFFFFFF
                };
                let wn = (w3_data >> 14) & 0x3FF;
                print!(" WN={}", wn);
            }
            println!(" [OK]");
        } else {
            if subframes_total <= 20 {
                let failed: Vec<usize> = parity_results.iter()
                    .filter(|(_, ok)| !ok)
                    .map(|(idx, _)| *idx)
                    .collect();
                println!("  bit {}: [PARITY FAIL words {:?}]", i, failed);
            }
        }

        i += 300;
    }

    let status = if subframes_valid >= 3 {
        "DECODE OK"
    } else if subframes_valid > 0 {
        "PARTIAL"
    } else {
        "NO DECODE"
    };
    println!(
        "PRN {:02}: {} bits, {}/{} subframes OK — {}\n",
        svid,
        nav_bits.len(),
        subframes_valid,
        subframes_total,
        status
    );

    DecodeResult { subframes_valid }
}

// GPS ICD parity lookup table (same as lnavbit.rs / nav_decode.rs)
const PARITY_TABLE: [[u8; 16]; 6] = [
    [0x00,0x13,0x25,0x36,0x0B,0x18,0x2E,0x3D,0x16,0x05,0x33,0x20,0x1D,0x0E,0x38,0x2B],
    [0x00,0x2C,0x19,0x35,0x32,0x1E,0x2B,0x07,0x26,0x0A,0x3F,0x13,0x14,0x38,0x0D,0x21],
    [0x00,0x0E,0x1F,0x11,0x3E,0x30,0x21,0x2F,0x3D,0x33,0x22,0x2C,0x03,0x0D,0x1C,0x12],
    [0x00,0x38,0x31,0x09,0x23,0x1B,0x12,0x2A,0x07,0x3F,0x36,0x0E,0x24,0x1C,0x15,0x2D],
    [0x00,0x0D,0x1A,0x17,0x37,0x3A,0x2D,0x20,0x2F,0x22,0x35,0x38,0x18,0x15,0x02,0x0F],
    [0x00,0x27,0x0D,0x2A,0x1B,0x3C,0x16,0x31,0x36,0x11,0x3B,0x1C,0x2D,0x0A,0x20,0x07],
];

/// GPS parity computation using proven PARITY_TABLE LUT (from nav_decode.rs)
fn gps_compute_parity(word: u32) -> u32 {
    let mut w = word >> 6;
    let mut parity = 0u32;
    for i in 0..6 {
        parity ^= PARITY_TABLE[i][(w & 0xf) as usize] as u32;
        w >>= 4;
    }
    if (w & 1) != 0 { parity ^= 0x15; }
    if (w & 2) != 0 { parity ^= 0x29; }
    parity
}

fn check_subframe_parity(words: &[u32; 10]) -> bool {
    for word_idx in 0..10 {
        let word_30 = words[word_idx];

        // D29* and D30* from previous word's last 2 bits
        let (d29_star, d30_star) = if word_idx == 0 {
            (0u32, 0u32)
        } else {
            ((words[word_idx - 1] >> 1) & 1, words[word_idx - 1] & 1)
        };

        // Assemble 32-bit: D29*(31) | D30*(30) | 30-bit word
        let full_word = (d29_star << 31) | (d30_star << 30) | (word_30 & 0x3FFFFFFF);
        let received_parity = word_30 & 0x3F;
        let computed_parity = gps_compute_parity(full_word);

        if computed_parity != received_parity {
            return false;
        }
    }
    true
}
