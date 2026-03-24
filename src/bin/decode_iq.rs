//! GPS L1CA IQ8 file decoder.
//!
//! Reads a .C8 file, acquires GPS satellites, tracks carrier/code,
//! extracts navigation bits, and verifies LNAV parity.
//!
//! Usage: cargo run --release --bin decode_iq [file.C8]

use gnss_rust::constants::SIGNAL_INDEX_L1CA;
use gnss_rust::prngenerate::PrnGenerate;
use gnss_rust::types::GnssSystem;
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
    doppler: f64,
    total_ms: usize,
) -> DecodeResult {
    // 1. Per-ms prompt correlation with FLL-assisted Doppler tracking
    let mut ms_prompt = vec![0.0_f64; total_ms];
    let mut carrier_phase = 0.0_f64;
    let mut current_doppler = doppler;
    let mut tracked_offset = code_offset as f64;

    // FLL: estimate Doppler drift from cross-product of consecutive ms correlations
    let mut prev_ci = 0.0_f64;
    let mut prev_cq = 0.0_f64;

    for ms in 0..total_ms {
        let carrier_per_sample = current_doppler / SAMPLE_RATE;
        let code_doppler_per_ms = 1_023_000.0 * current_doppler / 1_575_420_000.0 / 1000.0;

        let mut ci = 0.0_f64;
        let mut cq = 0.0_f64;
        for s in 0..SAMPLES_PER_MS {
            let idx = ms * SAMPLES_PER_MS + s;
            if idx >= iq.len() { break; }
            let angle = carrier_phase * TAU;
            let (sv, cv) = angle.sin_cos();
            carrier_phase += carrier_per_sample;
            let si = iq[idx].0 * cv + iq[idx].1 * sv;
            let sq = -iq[idx].0 * sv + iq[idx].1 * cv;
            let chip = ((s as f64 * CHIP_STEP + tracked_offset) as usize) % CODE_LEN;
            ci += si * prn_code[chip];
            cq += sq * prn_code[chip];
        }
        ms_prompt[ms] = ci;

        // FLL: cross-product frequency discriminator
        if ms > 0 {
            let cross = prev_ci * cq - prev_cq * ci;
            let dot = prev_ci * ci + prev_cq * cq;
            if dot.abs() > 1e-10 {
                let freq_error = cross.atan2(dot) / (TAU * 0.001); // Hz
                current_doppler += freq_error * 0.05; // loop bandwidth ~25 Hz
            }
        }
        prev_ci = ci;
        prev_cq = cq;

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

    // 4. Preamble search + parity check
    let preamble: [i8; 8] = [-1, 1, 1, 1, -1, 1, -1, -1];
    let preamble_inv: [i8; 8] = [1, -1, -1, -1, 1, -1, 1, 1];

    let mut subframes_valid = 0;
    let mut subframes_total = 0;

    let mut i = 0;
    while i + 300 <= nav_bits.len() {
        let match_n = (0..8).all(|k| nav_bits[i + k] == preamble[k]);
        let match_i = (0..8).all(|k| nav_bits[i + k] == preamble_inv[k]);

        if !match_n && !match_i {
            i += 1;
            continue;
        }

        let inverted = match_i;

        // Extract 10 words × 30 bits
        let mut words = [0u32; 10];
        for w in 0..10 {
            let mut word = 0u32;
            for b in 0..30 {
                let bit = if inverted {
                    -nav_bits[i + w * 30 + b]
                } else {
                    nav_bits[i + w * 30 + b]
                };
                if bit < 0 {
                    word |= 1 << (29 - b);
                }
            }
            words[w] = word;
        }

        // Parity check
        let parity_ok = check_subframe_parity(&words);
        subframes_total += 1;

        if parity_ok {
            subframes_valid += 1;
            let sf_id = (words[1] >> 8) & 0x7;
            let tow = (words[1] >> 13) & 0x1FFFF;
            print!("  SF{} TOW={:6}s", sf_id, tow * 6);
            if sf_id == 1 {
                let wn = (words[2] >> 20) & 0x3FF;
                print!(" WN={}", wn);
            }
            println!(" [OK]");
        } else if subframes_total <= 20 {
            println!("  bit {}: [PARITY FAIL]", i);
        }

        // Advance to next subframe (300 bits)
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

fn check_subframe_parity(words: &[u32; 10]) -> bool {
    let mut d29_star = 0u32;
    let mut d30_star = 0u32;

    for w in 0..10 {
        let received = words[w];

        let data_bits = if d30_star == 1 {
            ((received >> 6) ^ 0xFFFFFF) & 0xFFFFFF
        } else {
            (received >> 6) & 0xFFFFFF
        };

        let full = (d29_star << 31) | (d30_star << 30) | (data_bits << 6) | (received & 0x3F);

        if !check_word_parity(full) {
            return false;
        }

        d29_star = (received >> 1) & 1;
        d30_star = received & 1;
    }
    true
}

fn check_word_parity(word: u32) -> bool {
    let d29s = (word >> 31) & 1;
    let d30s = (word >> 30) & 1;
    let d = if d30s == 1 {
        ((word >> 6) ^ 0xFFFFFF) & 0xFFFFFF
    } else {
        (word >> 6) & 0xFFFFFF
    };

    // GPS ICD-200 parity polynomials
    let masks: [u32; 6] = [
        0xBB1F34, 0x5D8F9A, 0xAEC7CD, 0x5763E6, 0x6BB1F3, 0x8B7A89,
    ];

    let received = word & 0x3F;
    let mut computed = 0u32;

    for (i, &mask) in masks.iter().enumerate() {
        let mut bit = (d & mask).count_ones() % 2;
        match i {
            0..=3 => bit ^= d29s,
            4 => bit ^= d30s,
            5 => bit ^= d29s,
            _ => {}
        }
        computed |= bit << (5 - i);
    }

    computed == received
}
