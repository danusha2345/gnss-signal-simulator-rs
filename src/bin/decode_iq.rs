//! GPS L1CA IQ8 file decoder — parallel acquisition, tracking, LNAV decode.
//!
//! Usage: cargo run --release --bin decode_iq [file.C8]

use gnss_rust::constants::SIGNAL_INDEX_L1CA;
use gnss_rust::coordinate::{ecef_to_lla, lla_to_ecef};
use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use gnss_rust::nav_decode::{decode_lnav_stream123, verify_lnav_parity};
use gnss_rust::prngenerate::PrnGenerate;
use gnss_rust::satellite_param::{get_doppler, get_satellite_param};
use gnss_rust::types::*;
use rayon::prelude::*;
use std::f64::consts::TAU;

const SAMPLE_RATE: f64 = 5_000_000.0;
const SAMPLES_PER_MS: usize = 5000;
const CODE_LEN: usize = 1023;
const CHIP_STEP: f64 = CODE_LEN as f64 / SAMPLES_PER_MS as f64;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let file_path = args.get(1).map(|s| s.as_str()).unwrap_or("generated_files/gps_pilot.C8");
    let rinex_path = args.get(2).map(|s| s.as_str())
        .unwrap_or("Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx");

    println!("GPS L1CA IQ8 Decoder");
    println!("File:  {}", file_path);
    println!("RINEX: {}", rinex_path);

    let raw = std::fs::read(file_path).expect("Cannot read file");
    let total_samples = raw.len() / 2;
    let total_ms = total_samples / SAMPLES_PER_MS;
    println!("Duration: {:.1}s ({} ms)\n", total_samples as f64 / SAMPLE_RATE, total_ms);

    let iq: Vec<(f64, f64)> = raw.chunks(2)
        .map(|c| (c[0] as i8 as f64, c[1] as i8 as f64))
        .collect();

    // Load RINEX
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, rinex_path, 10000);
    let gps_time = gnss_rust::gnsstime::utc_to_gps_time(
        UtcTime { Year: 2025, Month: 6, Day: 5, Hour: 10, Minute: 5, Second: 30.0 }, true,
    );
    let target_sow = gps_time.MilliSeconds as f64 / 1000.0;

    let receiver_lla = LlaPosition {
        lat: 55.7539_f64.to_radians(), lon: 37.6208_f64.to_radians(), alt: 150.0,
    };
    let receiver_ecef = lla_to_ecef(&receiver_lla);
    let receiver_lla_conv = ecef_to_lla(&receiver_ecef);
    let iono = IonoParam::default();

    // === PARALLEL ACQUISITION ===
    println!("=== ACQUISITION ===");
    let acq_start = std::time::Instant::now();

    let mut acq_results: Vec<(u8, usize, f64, f64)> = (1u8..=32)
        .into_par_iter()
        .map(|svid| {
            let prn_code = get_prn_code(svid);
            let (offset, doppler, z) = acquire(&iq, &prn_code);
            (svid, offset, doppler, z)
        })
        .collect();
    acq_results.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap());

    let acquired: Vec<_> = acq_results.iter().filter(|r| r.3 > 2.8).cloned().collect();
    println!("{} satellites in {:.1}s:", acquired.len(), acq_start.elapsed().as_secs_f64());
    for &(svid, offset, doppler, z) in &acquired {
        println!("  PRN {:02}: offset={:4}, doppler={:+7.0} Hz, z={:.1}", svid, offset, doppler, z);
    }

    if acquired.is_empty() { println!("\nNo satellites found."); return; }

    // === PARALLEL TRACKING + DECODE ===
    println!("\n=== TRACKING & NAV DECODE ===");
    let track_start = std::time::Instant::now();

    let decode_results: Vec<(u8, DecodeResult)> = acquired.par_iter().map(|&(svid, code_offset, _doppler, _z)| {
        let prn_code = get_prn_code(svid);

        // Find ephemeris
        let eph = nav_data.gps_ephemeris.iter()
            .filter(|e| e.svid == svid && e.valid != 0)
            .min_by(|a, b| (a.toe as f64 - target_sow).abs().partial_cmp(&(b.toe as f64 - target_sow).abs()).unwrap())
            .copied();

        let result = track_and_decode(
            &iq, &prn_code, svid, code_offset, eph,
            &gps_time, &receiver_ecef, &receiver_lla_conv, &iono, total_ms,
        );
        (svid, result)
    }).collect();

    println!("Tracking done in {:.1}s\n", track_start.elapsed().as_secs_f64());

    // === RESULTS ===
    let mut total_decoded = 0;
    for (svid, result) in &decode_results {
        let status = if result.subframes_valid >= 3 { "DECODE OK" } else { "PARTIAL" };
        if result.subframes_valid >= 3 { total_decoded += 1; }
        println!("PRN {:02}: {}/{} subframes, {} bits — {}",
            svid, result.subframes_valid, result.subframes_total, result.nav_bits_count, status);

        // Print decoded ephemeris parameters
        for sf in &result.subframe_info {
            println!("    SF{} TOW={}s{}", sf.sf_id, sf.tow * 6,
                if sf.sf_id == 1 { format!(" WN={}", sf.wn) } else { String::new() });
        }
        for diff in &result.eph_diffs {
            let mark = if diff.ok { "  " } else { "!!" };
            println!("  {} {:12}: RINEX={:>15.6}  decoded={:>15.6}  err={:.2e}",
                mark, diff.name, diff.original, diff.decoded, diff.diff);
        }
    }

    println!("\n=== SUMMARY ===");
    println!("Acquired: {} | Decoded: {} with ≥3 subframes", acquired.len(), total_decoded);
}

fn get_prn_code(svid: u8) -> Vec<f64> {
    let prn = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, svid as i32);
    prn.get_data_prn().unwrap().iter().map(|&v| if v != 0 { -1.0 } else { 1.0 }).collect()
}

fn acquire(iq: &[(f64, f64)], prn_code: &[f64]) -> (usize, f64, f64) {
    let mut best_power = 0.0_f64;
    let mut best_doppler = 0.0;
    let mut best_offset = 0usize;
    let mut noise_sum = 0.0_f64;
    let mut noise_count = 0usize;

    for doppler_bin in -20i32..=20 {
        let doppler = doppler_bin as f64 * 250.0;
        let mut power_per_offset = vec![0.0f64; CODE_LEN];

        for ms in 0..10 {
            let mut si = vec![0.0f64; SAMPLES_PER_MS];
            let mut sq = vec![0.0f64; SAMPLES_PER_MS];
            for s in 0..SAMPLES_PER_MS {
                let idx = ms * SAMPLES_PER_MS + s;
                if idx >= iq.len() { break; }
                let t = idx as f64 / SAMPLE_RATE;
                let phase = TAU * doppler * t;
                let (sv, cv) = phase.sin_cos();
                si[s] = iq[idx].0 * cv + iq[idx].1 * sv;
                sq[s] = -iq[idx].0 * sv + iq[idx].1 * cv;
            }
            for offset in 0..CODE_LEN {
                let mut ci = 0.0_f64;
                let mut cq = 0.0_f64;
                for s in 0..SAMPLES_PER_MS {
                    let chip = ((s as f64 * CHIP_STEP + offset as f64) as usize) % CODE_LEN;
                    ci += si[s] * prn_code[chip];
                    cq += sq[s] * prn_code[chip];
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
    let z = (best_power / (noise_sum / noise_count as f64)).sqrt();
    (best_offset, best_doppler, z)
}

struct SubframeInfo { sf_id: u32, tow: u32, wn: u32 }

struct DecodeResult {
    nav_bits_count: usize,
    subframes_valid: usize,
    subframes_total: usize,
    subframe_info: Vec<SubframeInfo>,
    eph_diffs: Vec<gnss_rust::nav_decode::ParamDiff>,
}

fn track_and_decode(
    iq: &[(f64, f64)], prn_code: &[f64], svid: u8, code_offset: usize,
    eph: Option<GpsEphemeris>,
    gps_time: &GnssTime, receiver_ecef: &KinematicInfo, receiver_lla: &LlaPosition,
    iono: &IonoParam, total_ms: usize,
) -> DecodeResult {
    let eph_val = eph.unwrap_or_default();

    // 1. Per-ms prompt correlation with orbital Doppler (updated every 100ms)
    let mut ms_prompt = vec![0.0_f64; total_ms];
    let mut carrier_phase = 0.0_f64;
    let mut current_doppler = 0.0_f64;
    let mut tracked_offset = code_offset as f64;

    for ms in 0..total_ms {
        if ms % 100 == 0 {
            let ms_time = GnssTime {
                Week: gps_time.Week, MilliSeconds: gps_time.MilliSeconds + ms as i32, SubMilliSeconds: 0.0,
            };
            let mut sp = SatelliteParam::default();
            sp.system = GnssSystem::GpsSystem;
            get_satellite_param(receiver_ecef, receiver_lla, &ms_time, GnssSystem::GpsSystem, &eph_val, iono, &mut sp);
            current_doppler = get_doppler(&sp, SIGNAL_INDEX_L1CA);
        }

        let cps = current_doppler / SAMPLE_RATE;
        let cdpm = 1_023_000.0 * current_doppler / 1_575_420_000.0 / 1000.0;

        let mut ci = 0.0_f64;
        for s in 0..SAMPLES_PER_MS {
            let idx = ms * SAMPLES_PER_MS + s;
            if idx >= iq.len() { break; }
            let (sv, cv) = (carrier_phase * TAU).sin_cos();
            carrier_phase += cps;
            let si = iq[idx].0 * cv + iq[idx].1 * sv;
            let chip = ((s as f64 * CHIP_STEP + tracked_offset) as usize) % CODE_LEN;
            ci += si * prn_code[chip];
        }
        ms_prompt[ms] = ci;
        tracked_offset = (tracked_offset + cdpm).rem_euclid(CODE_LEN as f64);
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
        if energy > best_energy { best_energy = energy; best_sync = sync; }
    }

    // 3. Extract nav bits
    let mut nav_bits: Vec<i8> = Vec::new();
    let mut idx = best_sync;
    while idx + 20 <= total_ms {
        let sum: f64 = ms_prompt[idx..idx + 20].iter().sum();
        nav_bits.push(if sum > 0.0 { 1 } else { -1 });
        idx += 20;
    }

    // 4. Preamble search + parity + ephemeris decode
    let preamble_n: [i8; 8] = [-1, 1, 1, 1, -1, 1, -1, -1];
    let preamble_i: [i8; 8] = [1, -1, -1, -1, 1, -1, 1, 1];

    let mut subframes_valid = 0;
    let mut subframes_total = 0;
    let mut sf_info = Vec::new();
    let mut collected_streams: [[u32; 8]; 3] = [[0; 8]; 3]; // for sf 1,2,3
    let mut collected_mask = [false; 3];

    let mut i = 0;
    while i + 300 <= nav_bits.len() {
        let mn = (0..8).all(|k| nav_bits[i + k] == preamble_n[k]);
        let mi = (0..8).all(|k| nav_bits[i + k] == preamble_i[k]);
        if !mn && !mi { i += 1; continue; }

        let inverted = mi;
        let mut bits_i32 = [0i32; 300];
        for k in 0..300 {
            let bit = if inverted { -nav_bits[i + k] } else { nav_bits[i + k] };
            bits_i32[k] = if bit < 0 { 1 } else { 0 };
        }

        let parity_results = verify_lnav_parity(&bits_i32);
        let all_ok = parity_results.iter().all(|(_, ok)| *ok);
        subframes_total += 1;

        if all_ok {
            subframes_valid += 1;

            // Reconstruct 10 words (30 bits each → u32)
            let mut words = [0u32; 10];
            for w in 0..10 {
                for b in 0..30 {
                    words[w] = (words[w] << 1) | (bits_i32[w * 30 + b] as u32 & 1);
                }
            }

            // Extract D30* corrected data for HOW
            let d30_w1 = words[0] & 1;
            let how_raw = if d30_w1 == 1 { ((words[1] >> 6) ^ 0xFFFFFF) & 0xFFFFFF } else { (words[1] >> 6) & 0xFFFFFF };
            let sf_id = (how_raw >> 2) & 0x7;
            let tow = (how_raw >> 7) & 0x1FFFF;

            let mut wn = 0u32;
            if sf_id == 1 {
                let d30_w2 = words[1] & 1;
                let w3_raw = if d30_w2 == 1 { ((words[2] >> 6) ^ 0xFFFFFF) & 0xFFFFFF } else { (words[2] >> 6) & 0xFFFFFF };
                wn = (w3_raw >> 14) & 0x3FF;
            }
            sf_info.push(SubframeInfo { sf_id, tow, wn });

            // Collect stream words for ephemeris decode (words 3-10 = stream[0..8])
            if sf_id >= 1 && sf_id <= 3 {
                let sf_idx = (sf_id - 1) as usize;
                for w in 2..10 {
                    let prev_d30 = words[w - 1] & 1;
                    let data = if prev_d30 == 1 { ((words[w] >> 6) ^ 0xFFFFFF) & 0xFFFFFF } else { (words[w] >> 6) & 0xFFFFFF };
                    collected_streams[sf_idx][w - 2] = data;
                }
                collected_mask[sf_idx] = true;
            }
        }
        i += 300;
    }

    // 5. Decode ephemeris if we have all 3 subframes
    let mut eph_diffs = Vec::new();
    if collected_mask[0] && collected_mask[1] && collected_mask[2] {
        let decoded = decode_lnav_stream123(&collected_streams);
        if let Some(ref original) = eph {
            eph_diffs = decoded.compare_with_original(original);
        }
    }

    DecodeResult {
        nav_bits_count: nav_bits.len(),
        subframes_valid,
        subframes_total,
        subframe_info: sf_info,
        eph_diffs,
    }
}
