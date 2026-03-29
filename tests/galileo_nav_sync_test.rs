//! Diagnostic test: verify Galileo E1 I-NAV symbol alignment in gnss_pilot signal.
//!
//! Generates a short Galileo E1 signal using gnss_pilot's method,
//! correlates with the data PRN code to strip code+BOC,
//! and checks that the demodulated symbols match the expected I-NAV bits.

use gnss_rust::constants::*;
use gnss_rust::coordinate::{ecef_to_lla, lla_to_ecef};
use gnss_rust::inavbit::INavBit;
use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use gnss_rust::prngenerate::PrnGenerate;
use gnss_rust::satellite_param::{get_doppler, get_satellite_param, get_travel_time};
use gnss_rust::types::*;

const GAL_CHIP_RATE: f64 = 1_023_000.0;
const GAL_SUBCHIP_RATE: f64 = 2_046_000.0;
const GAL_CODE_LEN: usize = 4092;
const GAL_NAV_BIT_MS: i32 = 4;
const GAL_FRAME_MS: i32 = 2000;
const GAL_BITS_PER_FRAME: usize = 500;
const CS25: u32 = 0x9b501c;
const CS25_LEN: i32 = 25;
const AMP_HALF: f64 = std::f64::consts::FRAC_1_SQRT_2;
const SYNC_PATTERN: [i32; 10] = [0, 1, 0, 1, 1, 0, 0, 0, 0, 0];

#[test]
fn test_galileo_inav_symbol_alignment() {
    // Load RINEX
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx", 10000);

    let utc = UtcTime { Year: 2025, Month: 6, Day: 5, Hour: 10, Minute: 5, Second: 30.0 };
    let gps_time = gnss_rust::gnsstime::utc_to_gps_time(utc, true);
    let receiver_lla = LlaPosition {
        lat: 55.7539_f64.to_radians(), lon: 37.6208_f64.to_radians(), alt: 150.0,
    };
    let receiver_ecef = lla_to_ecef(&receiver_lla);
    let receiver_lla_c = ecef_to_lla(&receiver_ecef);
    let iono = IonoParam::default();
    let sample_rate = 5_000_000.0_f64;
    let dt = 1.0 / sample_rate;
    let spm = (sample_rate / 1000.0) as usize; // 5000

    // Pick first visible Galileo satellite
    let svid = 2u8;
    let eph = nav_data.galileo_ephemeris.iter()
        .filter(|e| e.svid == svid && e.valid != 0)
        .next().copied()
        .expect("No Galileo E02 ephemeris");

    let mut sp = SatelliteParam::default();
    sp.system = GnssSystem::GalileoSystem;
    get_satellite_param(&receiver_ecef, &receiver_lla_c, &gps_time, GnssSystem::GalileoSystem, &eph, &iono, &mut sp);
    assert!(sp.Elevation > 5.0_f64.to_radians(), "E02 not visible");

    // Generate PRN codes
    let prn_gen = PrnGenerate::new(GnssSystem::GalileoSystem, SIGNAL_INDEX_E1 as i32, svid as i32);
    let raw_data = prn_gen.get_data_prn().unwrap();
    let data_prn: Vec<f64> = raw_data.iter().map(|&v| if v != 0 { -1.0 } else { 1.0 }).collect();

    // Initialize I-NAV
    let mut inav = INavBit::new();
    inav.set_ephemeris(svid as i32, &eph);

    // Generate 4 seconds of signal (2 full pages) and demodulate
    let duration_ms = 4000;
    let mut current_frame: i32 = -1;
    let mut nav_bits = vec![0i32; GAL_BITS_PER_FRAME];

    // We'll accumulate correlation per 4ms code period to get one symbol
    let samples_per_symbol = spm * 4; // 20000 samples per 4ms symbol
    let total_symbols = duration_ms / 4;
    let mut demod_symbols: Vec<f64> = Vec::with_capacity(total_symbols as usize);
    let mut expected_bits: Vec<i32> = Vec::with_capacity(total_symbols as usize);

    let mut doppler_hz = get_doppler(&sp, SIGNAL_INDEX_E1);
    let mut travel_time_s = get_travel_time(&sp, SIGNAL_INDEX_E1);

    // Track carrier phase for coherent demodulation
    let mut carrier_phase: f64 = 0.0;
    let if_freq = 0.0_f64;

    for sym_idx in 0..total_symbols {
        let sym_start_ms = sym_idx * 4;
        let mut correlation = 0.0_f64;
        let mut expected_bit = -999i32;

        for ms_off in 0..4 {
            let global_ms = sym_start_ms + ms_off;
            let ms_time = GnssTime {
                Week: gps_time.Week,
                MilliSeconds: gps_time.MilliSeconds + global_ms,
                SubMilliSeconds: 0.0,
            };

            // Update params every ms
            let mut sp_local = SatelliteParam::default();
            sp_local.system = GnssSystem::GalileoSystem;
            get_satellite_param(&receiver_ecef, &receiver_lla_c, &ms_time, GnssSystem::GalileoSystem, &eph, &iono, &mut sp_local);
            doppler_hz = get_doppler(&sp_local, SIGNAL_INDEX_E1);
            travel_time_s = get_travel_time(&sp_local, SIGNAL_INDEX_E1);
            let freq = if_freq + doppler_hz;
            let rx_sow = ms_time.MilliSeconds as f64 / 1000.0;

            // Nav bit: integer ms (matching gnss_pilot fix)
            let travel_ms = (travel_time_s * 1000.0) as i32;
            let tx_ms_int = ms_time.MilliSeconds - travel_ms;
            let tx_ms_offset = tx_ms_int + 1000;
            let fn_ = tx_ms_offset / GAL_FRAME_MS;
            if fn_ != current_frame {
                inav.get_frame_data(
                    GnssTime { Week: ms_time.Week, MilliSeconds: tx_ms_int, SubMilliSeconds: 0.0 },
                    svid as i32, 1, &mut nav_bits,
                );
                current_frame = fn_;
            }
            let bi = ((tx_ms_offset % GAL_FRAME_MS) / GAL_NAV_BIT_MS) as usize;
            let nav_bit: f64 = if nav_bits[bi.min(GAL_BITS_PER_FRAME - 1)] != 0 { -1.0 } else { 1.0 };

            // Record expected bit (constant within 4ms symbol)
            if ms_off == 0 {
                expected_bit = nav_bits[bi.min(GAL_BITS_PER_FRAME - 1)];
            }

            for s in 0..spm {
                let rx_time = rx_sow + s as f64 * dt;
                let tx_time = rx_time - travel_time_s;

                // Code phase (same as gnss_pilot)
                let chip_count = tx_time * GAL_CHIP_RATE;
                let logical_chip = ((chip_count % GAL_CODE_LEN as f64) + GAL_CODE_LEN as f64) as usize % GAL_CODE_LEN;
                let subchip = (tx_time * GAL_SUBCHIP_RATE) as i64;

                // BOC(1,1) sign
                let boc11: f64 = if subchip & 1 != 0 { -1.0 } else { 1.0 };

                // Generate signal (data channel only, no pilot, no noise)
                let data_val = -nav_bit * data_prn[logical_chip] * boc11 * AMP_HALF;

                // Carrier
                let angle = carrier_phase * std::f64::consts::TAU;
                let (sv, cv) = angle.sin_cos();
                carrier_phase += freq * dt;

                // Signal = data_val * carrier
                let sig_i = data_val * cv;
                let sig_q = data_val * sv;

                // Correlate: multiply by local replica (PRN * BOC * carrier)
                let local_prn = data_prn[logical_chip];
                let local_boc = boc11;
                let local_i = local_prn * local_boc * cv;
                let local_q = local_prn * local_boc * sv;

                // Dot product
                correlation += sig_i * local_i + sig_q * local_q;
            }
        }

        demod_symbols.push(correlation);
        expected_bits.push(expected_bit);
    }

    // Normalize
    let max_abs = demod_symbols.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    println!("\n=== Galileo E1 I-NAV Symbol Alignment Test (E{:02}) ===", svid);
    println!("Duration: {}ms = {} symbols", duration_ms, total_symbols);
    println!("Max |correlation|: {:.1}", max_abs);

    // Check symbol polarity matches expected nav bits
    let mut mismatches = 0;
    let mut first_mismatch = -1i32;
    for (i, (corr, &expected)) in demod_symbols.iter().zip(expected_bits.iter()).enumerate() {
        // correlation > 0 means nav_bit was 0 (mapped to +1, then -1*+1 = -1, but corr with prn gives...)
        // Actually: data_val = -nav_bit * prn * boc * AMP_HALF
        // Correlation = sum(data_val * prn * boc * carrier² ) ≈ -nav_bit * AMP_HALF * N
        // So corr < 0 means nav_bit > 0 means expected = 1
        // And corr > 0 means nav_bit < 0 means expected = 0... wait
        // nav_bit = if nav_bits[bi] != 0 { -1.0 } else { 1.0 }
        // data_val = -nav_bit * prn * boc * AMP
        // corr ≈ -nav_bit * AMP * N (after removing prn*boc via correlation)
        // if nav_bits[bi] = 0: nav_bit = 1.0, corr ≈ -1*AMP*N < 0
        // if nav_bits[bi] = 1: nav_bit = -1.0, corr ≈ +1*AMP*N > 0
        let demod_bit = if *corr > 0.0 { 1 } else { 0 };
        if demod_bit != expected {
            mismatches += 1;
            if first_mismatch < 0 { first_mismatch = i as i32; }
        }
    }

    println!("Mismatches: {}/{}", mismatches, total_symbols);
    if first_mismatch >= 0 {
        println!("First mismatch at symbol {}", first_mismatch);
    }

    // Print first 40 symbols (should cover sync pattern + some data)
    println!("\nFirst 40 demodulated symbols vs expected:");
    println!("sym | corr          | demod | expected | match");
    println!("----|---------------|-------|----------|------");
    for i in 0..40.min(total_symbols as usize) {
        let demod_bit = if demod_symbols[i] > 0.0 { 1 } else { 0 };
        let ok = if demod_bit == expected_bits[i] { "✓" } else { "✗" };
        println!("{:3} | {:+13.1} | {:5} | {:8} | {}", i, demod_symbols[i], demod_bit, expected_bits[i], ok);
    }

    // Look for sync pattern in demodulated stream
    println!("\n--- Sync pattern search ---");
    let sync_as_bits: Vec<i32> = SYNC_PATTERN.to_vec();
    for start in 0..(total_symbols as usize - 10) {
        let demod_bits: Vec<i32> = (start..start+10)
            .map(|i| if demod_symbols[i] > 0.0 { 1 } else { 0 })
            .collect();
        if demod_bits == sync_as_bits {
            println!("SYNC found at symbol {} (time = {}ms from start)", start, start * 4);
        }
    }

    // Also check inverted sync
    let sync_inv: Vec<i32> = SYNC_PATTERN.iter().map(|&b| 1 - b).collect();
    for start in 0..(total_symbols as usize - 10) {
        let demod_bits: Vec<i32> = (start..start+10)
            .map(|i| if demod_symbols[i] > 0.0 { 1 } else { 0 })
            .collect();
        if demod_bits == sync_inv {
            println!("INVERTED SYNC found at symbol {} (time = {}ms)", start, start * 4);
        }
    }

    // Print what frame boundaries look like
    println!("\n--- Frame boundary analysis ---");
    let target_sow = gps_time.MilliSeconds as f64 / 1000.0;
    let travel = get_travel_time(&sp, SIGNAL_INDEX_E1);
    let tx_start = target_sow - travel;
    let tx_ms_start = (tx_start * 1000.0) as i32;
    let tx_ms_offset_start = tx_ms_start + 1000;
    let bi_start = ((tx_ms_offset_start % GAL_FRAME_MS) / GAL_NAV_BIT_MS) as usize;
    println!("tx_time_start = {:.6}s, tx_ms = {}, tx_ms_offset = {}", tx_start, tx_ms_start, tx_ms_offset_start);
    println!("Starting bit index = {} (of 0..499)", bi_start);
    println!("Symbols until next even sync (bi=0): {}", (500 - bi_start) % 500);
    println!("Symbols until next odd sync (bi=250): {}", (750 - bi_start) % 500);

    assert_eq!(mismatches, 0, "Symbol polarity mismatches detected — nav bit modulation is wrong");
}
