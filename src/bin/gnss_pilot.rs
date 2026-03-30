//! Combined GPS L1CA + Galileo E1 + BeiDou B1C signal generator.
//!
//! Generates all three L1-band systems into a single IQ8 file with shared noise and AGC.

use gnss_rust::bcnav1bit::BCNav1Bit;
use gnss_rust::constants::*;
use gnss_rust::coordinate::{ecef_to_lla, lla_to_ecef};
use gnss_rust::inavbit::INavBit;
use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use gnss_rust::lnavbit::LNavBit;
use gnss_rust::pilotbit::get_pilot_bits;
use gnss_rust::prngenerate::PrnGenerate;
use gnss_rust::satellite_param::{get_doppler, get_satellite_param, get_travel_time};
use gnss_rust::types::*;
use rand::Rng;
use rayon::prelude::*;
use std::io::Write;

// GPS constants
const GPS_CHIP_RATE: f64 = 1_023_000.0;
const GPS_CODE_LEN: usize = 1023;

// Galileo E1 constants
const GAL_CHIP_RATE: f64 = 1_023_000.0;
const GAL_SUBCHIP_RATE: f64 = 2_046_000.0;
const GAL_CODE_LEN: usize = 4092;
const GAL_NAV_BIT_MS: i32 = 4;
const GAL_FRAME_MS: i32 = 2000;
const GAL_BITS_PER_FRAME: usize = 500;
const CS25: u32 = 0x9b501c;
const CS25_LEN: i32 = 25;
const AMP_HALF: f64 = std::f64::consts::FRAC_1_SQRT_2;

// BeiDou B1C constants
const BDS_CHIP_RATE: f64 = 1_023_000.0;
const BDS_SUBCHIP_RATE: f64 = 2_046_000.0;
const BDS_CODE_LEN: usize = 10230;
const BDS_NAV_BIT_MS: i32 = 10;
const BDS_FRAME_MS: i32 = 18000;
const BDS_BITS_PER_FRAME: usize = 1800;
const BDS_SEC_CODE_LEN: i32 = 1800;
const AMP_1_4: f64 = 0.25;
const AMP_29_44: f64 = 0.811844396954584; // sqrt(29.0/44.0)

enum SatChannel {
    Gps {
        svid: u8,
        eph: GpsEphemeris,
        prn_code: [f64; GPS_CODE_LEN],
        carrier_phase: f64,
        doppler_hz: f64,
        travel_time_s: f64,
        amplitude: f64,
        lnav: LNavBit,
        nav_bits: [i32; 300],
        current_frame: i32,
    },
    Galileo {
        svid: u8,
        eph: GpsEphemeris,
        data_prn: Vec<f64>,
        pilot_prn: Vec<f64>,
        carrier_phase: f64,
        doppler_hz: f64,
        travel_time_s: f64,
        amplitude: f64,
        inav: INavBit,
        nav_bits: Vec<i32>,
        current_frame: i32,
    },
    BeiDou {
        svid: u8,
        eph: GpsEphemeris,
        data_prn: Vec<f64>,
        pilot_prn: Vec<f64>,
        pilot_secondary: Vec<u32>,
        carrier_phase: f64,
        doppler_hz: f64,
        travel_time_s: f64,
        amplitude: f64,
        bcnav1: BCNav1Bit,
        nav_bits: Vec<i32>,
        current_frame: i32,
    },
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let rinex = args.get(1).map(|s| s.as_str())
        .unwrap_or("Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx");
    let duration: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(300.0);
    let output = args.get(3).map(|s| s.as_str()).unwrap_or("generated_files/gnss_pilot.C8");

    let receiver_lla = LlaPosition {
        lat: 55.7539_f64.to_radians(), lon: 37.6208_f64.to_radians(), alt: 150.0,
    };
    let utc_time = UtcTime { Year: 2025, Month: 6, Day: 5, Hour: 10, Minute: 5, Second: 30.0 };
    let sample_rate: f64 = 5_000_000.0;
    let cn0_db: f64 = 55.0;
    let if_freq: f64 = 0.0;
    let el_mask: f64 = 5.0_f64.to_radians();

    // Load RINEX
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, rinex, 50000);

    let gps_time = gnss_rust::gnsstime::utc_to_gps_time(utc_time, true);
    let target_sow = gps_time.MilliSeconds as f64 / 1000.0;
    let receiver_ecef = lla_to_ecef(&receiver_lla);
    let receiver_lla_c = ecef_to_lla(&receiver_ecef);
    let iono = IonoParam::default();
    let amplitude = (10.0_f64.powf(cn0_db / 10.0) / sample_rate).sqrt();

    let mut channels: Vec<SatChannel> = Vec::new();
    let mut gps_count = 0u32;
    let mut gal_count = 0u32;
    let mut bds_count = 0u32;

    // === GPS satellites ===
    for svid in 1u8..=32 {
        let eph = match nav_data.gps_ephemeris.iter()
            .filter(|e| e.svid == svid && e.valid != 0)
            .min_by(|a, b| (a.toe as f64 - target_sow).abs().partial_cmp(&(b.toe as f64 - target_sow).abs()).unwrap())
            .copied() { Some(e) => e, None => continue };

        let mut sp = SatelliteParam::default();
        sp.system = GnssSystem::GpsSystem;
        get_satellite_param(&receiver_ecef, &receiver_lla_c, &gps_time, GnssSystem::GpsSystem, &eph, &iono, &mut sp);
        if sp.Elevation < el_mask { continue; }

        let prn_gen = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, svid as i32);
        let raw = match prn_gen.get_data_prn() { Some(c) => c, None => continue };
        let mut prn_code = [0.0f64; GPS_CODE_LEN];
        for i in 0..GPS_CODE_LEN { prn_code[i] = if raw[i] != 0 { -1.0 } else { 1.0 }; }

        let mut lnav = LNavBit::new();
        lnav.set_ephemeris(svid as i32, &eph);
        if let (Some(a), Some(b), Some(u)) = (nav_data.get_gps_iono_alpha(), nav_data.get_gps_iono_beta(), nav_data.utc_param.as_ref()) {
            lnav.set_iono_utc(&IonoParam { a0: a[0], a1: a[1], a2: a[2], a3: a[3], b0: b[0], b1: b[1], b2: b[2], b3: b[3], flag: 1 }, u);
        }

        println!("  GPS {:02}: el={:.1}\u{00b0} az={:.1}\u{00b0} dop={:.0}", svid, sp.Elevation.to_degrees(), sp.Azimuth.to_degrees(), get_doppler(&sp, SIGNAL_INDEX_L1CA));
        channels.push(SatChannel::Gps {
            svid, eph, prn_code, carrier_phase: 0.0,
            doppler_hz: get_doppler(&sp, SIGNAL_INDEX_L1CA),
            travel_time_s: get_travel_time(&sp, SIGNAL_INDEX_L1CA),
            amplitude, lnav, nav_bits: [0; 300], current_frame: -1,
        });
        gps_count += 1;
    }

    // === Galileo satellites ===
    for svid in 1u8..=36 {
        let eph = match nav_data.galileo_ephemeris.iter()
            .filter(|e| e.svid == svid && e.valid != 0)
            .min_by(|a, b| (a.toe as f64 - target_sow).abs().partial_cmp(&(b.toe as f64 - target_sow).abs()).unwrap())
            .copied() { Some(e) => e, None => continue };

        let mut sp = SatelliteParam::default();
        sp.system = GnssSystem::GalileoSystem;
        get_satellite_param(&receiver_ecef, &receiver_lla_c, &gps_time, GnssSystem::GalileoSystem, &eph, &iono, &mut sp);
        if sp.Elevation < el_mask { continue; }

        let prn_gen = PrnGenerate::new(GnssSystem::GalileoSystem, SIGNAL_INDEX_E1 as i32, svid as i32);
        let data_prn: Vec<f64> = match prn_gen.get_data_prn() { Some(c) => c, None => continue }
            .iter().map(|&v| if v != 0 { -1.0 } else { 1.0 }).collect();
        let pilot_prn: Vec<f64> = match prn_gen.get_pilot_prn() { Some(c) => c, None => continue }
            .iter().map(|&v| if v != 0 { -1.0 } else { 1.0 }).collect();

        let mut inav = INavBit::new();
        inav.set_ephemeris(svid as i32, &eph);

        println!("  GAL {:02}: el={:.1}\u{00b0} az={:.1}\u{00b0} dop={:.0}", svid, sp.Elevation.to_degrees(), sp.Azimuth.to_degrees(), get_doppler(&sp, SIGNAL_INDEX_E1));
        channels.push(SatChannel::Galileo {
            svid, eph, data_prn, pilot_prn, carrier_phase: 0.0,
            doppler_hz: get_doppler(&sp, SIGNAL_INDEX_E1),
            travel_time_s: get_travel_time(&sp, SIGNAL_INDEX_E1),
            amplitude, inav, nav_bits: vec![0; GAL_BITS_PER_FRAME], current_frame: -1,
        });
        gal_count += 1;
    }

    // === BeiDou B1C satellites (BDS-3 only, SVID 19-58) ===
    // BDS-2 (SVID 1-18, 59-63) does not transmit B1C
    for svid in 19u8..=58 {
        let bds_eph = match nav_data.beidou_ephemeris.iter()
            .filter(|e| e.svid == svid && e.valid != 0)
            .min_by(|a, b| (a.toe as f64 - target_sow).abs().partial_cmp(&(b.toe as f64 - target_sow).abs()).unwrap())
            { Some(e) => e, None => continue };

        let gps_eph = bds_eph.to_gps_ephemeris();
        let mut sp = SatelliteParam::default();
        sp.system = GnssSystem::BdsSystem;
        get_satellite_param(&receiver_ecef, &receiver_lla_c, &gps_time, GnssSystem::BdsSystem, &gps_eph, &iono, &mut sp);
        if sp.Elevation < el_mask { continue; }

        let prn_gen = PrnGenerate::new(GnssSystem::BdsSystem, SIGNAL_INDEX_B1C as i32, svid as i32);
        let data_prn: Vec<f64> = match prn_gen.get_data_prn() { Some(c) => c, None => continue }
            .iter().map(|&v| if v != 0 { -1.0 } else { 1.0 }).collect();
        let pilot_prn: Vec<f64> = match prn_gen.get_pilot_prn() { Some(c) => c, None => continue }
            .iter().map(|&v| if v != 0 { -1.0 } else { 1.0 }).collect();

        let pilot_secondary = match get_pilot_bits(GnssSystem::BdsSystem, SIGNAL_INDEX_B1C, svid as i32) {
            Some((words, _len)) => words.to_vec(),
            None => continue,
        };

        let mut bcnav1 = BCNav1Bit::new();
        bcnav1.set_ephemeris(svid as i32, &gps_eph);

        println!("  BDS {:02}: el={:.1}\u{00b0} az={:.1}\u{00b0} dop={:.0}", svid, sp.Elevation.to_degrees(), sp.Azimuth.to_degrees(), get_doppler(&sp, SIGNAL_INDEX_B1C));
        channels.push(SatChannel::BeiDou {
            svid, eph: gps_eph, data_prn, pilot_prn, pilot_secondary, carrier_phase: 0.0,
            doppler_hz: get_doppler(&sp, SIGNAL_INDEX_B1C),
            travel_time_s: get_travel_time(&sp, SIGNAL_INDEX_B1C),
            amplitude, bcnav1, nav_bits: vec![0; BDS_BITS_PER_FRAME], current_frame: -1,
        });
        bds_count += 1;
    }

    println!("[gnss_pilot] {} GPS + {} Galileo + {} BeiDou = {} total satellites", gps_count, gal_count, bds_count, channels.len());

    // Diagnostic: dump first Galileo I-NAV frame
    for ch in &mut channels {
        if let SatChannel::Galileo { svid, inav, nav_bits, .. } = ch {
            let test_time = GnssTime { Week: gps_time.Week, MilliSeconds: gps_time.MilliSeconds, SubMilliSeconds: 0.0 };
            inav.get_frame_data(test_time, *svid as i32, 1, nav_bits);
            let nonzero = nav_bits.iter().filter(|&&x| x != 0).count();
            println!("[DIAG] E{:02} I-NAV: {}/{} non-zero bits, first 20: {:?}",
                svid, nonzero, nav_bits.len(), &nav_bits[..20]);
            break;
        }
    }

    // === Generation ===
    let total_ms = (duration * 1000.0) as i32;
    let spm = (sample_rate / 1000.0) as usize;
    let dt = 1.0 / sample_rate;
    let noise_sigma = 0.5;
    let total_sig_rms = (channels.len() as f64).sqrt() * amplitude;
    let total_rms = (total_sig_rms * total_sig_rms + noise_sigma * noise_sigma).sqrt();
    let agc_gain = 0.25 / total_rms;

    println!("[gnss_pilot] Generating {:.0}s, AGC={:.4}", duration, agc_gain);

    let mut file = std::io::BufWriter::with_capacity(1 << 20, std::fs::File::create(output).unwrap());
    let start = std::time::Instant::now();

    for chunk_start in (0..total_ms).step_by(1000) {
        let chunk_end = (chunk_start + 1000).min(total_ms);
        let chunk_dur = (chunk_end - chunk_start) as usize;
        let chunk_samples = chunk_dur * spm;

        let sat_bufs: Vec<Vec<(f32, f32)>> = channels.par_iter_mut().map(|ch| {
            let mut buf = vec![(0.0f32, 0.0f32); chunk_samples];

            match ch {
                SatChannel::Gps { svid, eph, prn_code, carrier_phase, doppler_hz, travel_time_s, amplitude: amp, lnav, nav_bits, current_frame } => {
                    for ms_off in 0..chunk_dur {
                        let global_ms = chunk_start + ms_off as i32;
                        let ms_time = GnssTime { Week: gps_time.Week, MilliSeconds: gps_time.MilliSeconds + global_ms, SubMilliSeconds: 0.0 };
                        let mut sp = SatelliteParam::default();
                        sp.system = GnssSystem::GpsSystem;
                        let re = lla_to_ecef(&receiver_lla);
                        let rl = ecef_to_lla(&re);
                        get_satellite_param(&re, &rl, &ms_time, GnssSystem::GpsSystem, eph, &iono, &mut sp);
                        *doppler_hz = get_doppler(&sp, SIGNAL_INDEX_L1CA);
                        *travel_time_s = get_travel_time(&sp, SIGNAL_INDEX_L1CA);
                        let freq = if_freq + *doppler_hz;
                        let rx_sow = ms_time.MilliSeconds as f64 / 1000.0;
                        let base = ms_off * spm;

                        for s in 0..spm {
                            let rx_time = rx_sow + s as f64 * dt;
                            let tx_time = rx_time - *travel_time_s;
                            let chip_count = tx_time * GPS_CHIP_RATE;
                            let chip_idx = ((chip_count % GPS_CODE_LEN as f64) + GPS_CODE_LEN as f64) as usize % GPS_CODE_LEN;
                            let prn = prn_code[chip_idx];

                            let tx_ms = (tx_time * 1000.0) as i32;
                            let fn_ = tx_ms / 6000;
                            if fn_ != *current_frame {
                                lnav.get_frame_data(GnssTime { Week: ms_time.Week, MilliSeconds: tx_ms, SubMilliSeconds: 0.0 }, *svid as i32, 0, nav_bits);
                                *current_frame = fn_;
                            }
                            let bi = ((tx_ms % 6000) / 20) as usize;
                            let nav_bit: f64 = if nav_bits[bi.min(299)] != 0 { -1.0 } else { 1.0 };

                            let angle = *carrier_phase * std::f64::consts::TAU;
                            let (sv, cv) = angle.sin_cos();
                            *carrier_phase += freq * dt;

                            let sig = prn * nav_bit * *amp;
                            buf[base + s] = ((sig * cv) as f32, (sig * sv) as f32);
                        }
                    }
                },
                SatChannel::Galileo { svid, eph, data_prn, pilot_prn, carrier_phase, doppler_hz, travel_time_s, amplitude: amp, inav, nav_bits, current_frame } => {
                    for ms_off in 0..chunk_dur {
                        let global_ms = chunk_start + ms_off as i32;
                        let ms_time = GnssTime { Week: gps_time.Week, MilliSeconds: gps_time.MilliSeconds + global_ms, SubMilliSeconds: 0.0 };
                        let mut sp = SatelliteParam::default();
                        sp.system = GnssSystem::GalileoSystem;
                        let re = lla_to_ecef(&receiver_lla);
                        let rl = ecef_to_lla(&re);
                        get_satellite_param(&re, &rl, &ms_time, GnssSystem::GalileoSystem, eph, &iono, &mut sp);
                        *doppler_hz = get_doppler(&sp, SIGNAL_INDEX_E1);
                        *travel_time_s = get_travel_time(&sp, SIGNAL_INDEX_E1);
                        let freq = if_freq + *doppler_hz;
                        let rx_sow = ms_time.MilliSeconds as f64 / 1000.0;
                        let base = ms_off * spm;

                        // Nav bit timing: use integer ms (matching C++ satellite_signal.rs)
                        // tx_ms = receiver_ms - floor(travel_time_ms)
                        // NOT floor((rx_time - travel_time) * 1000) which differs by ±1ms
                        let travel_ms = (*travel_time_s * 1000.0) as i32;
                        let tx_ms_int = ms_time.MilliSeconds - travel_ms;
                        let tx_ms_offset_int = tx_ms_int + 1000;
                        let fn_ = tx_ms_offset_int / GAL_FRAME_MS;
                        if fn_ != *current_frame {
                            inav.get_frame_data(GnssTime { Week: ms_time.Week, MilliSeconds: tx_ms_int, SubMilliSeconds: 0.0 }, *svid as i32, 1, nav_bits);
                            *current_frame = fn_;
                        }
                        let bi = ((tx_ms_offset_int % GAL_FRAME_MS) / GAL_NAV_BIT_MS) as usize;
                        let nav_bit: f64 = if nav_bits[bi.min(GAL_BITS_PER_FRAME - 1)] != 0 { -1.0 } else { 1.0 };

                        let code_period_int = tx_ms_int / GAL_NAV_BIT_MS;
                        let sec_idx = code_period_int.rem_euclid(CS25_LEN) as u32;
                        let sec_bit: f64 = if (CS25 >> sec_idx) & 1 != 0 { -1.0 } else { 1.0 };

                        for s in 0..spm {
                            let rx_time = rx_sow + s as f64 * dt;
                            let tx_time = rx_time - *travel_time_s;

                            let chip_count = tx_time * GAL_CHIP_RATE;
                            let logical_chip = ((chip_count % GAL_CODE_LEN as f64) + GAL_CODE_LEN as f64) as usize % GAL_CODE_LEN;
                            let subchip = (tx_time * GAL_SUBCHIP_RATE) as i64;
                            let lchip_i64 = (tx_time * GAL_CHIP_RATE) as i64;

                            // BOC(1,1) sign
                            let boc11: f64 = if subchip & 1 != 0 { -1.0 } else { 1.0 };

                            // CBOC for pilot
                            let chip_in_code = lchip_i64.rem_euclid(GAL_CODE_LEN as i64);
                            let cboc: f64 = if chip_in_code % 11 == 0 {
                                let p = subchip.rem_euclid(12);
                                if p >= 6 { -1.0 } else { 1.0 }
                            } else { boc11 };

                            let data_val = -nav_bit * data_prn[logical_chip] * boc11 * AMP_HALF;
                            let pilot_val = sec_bit * pilot_prn[logical_chip] * cboc * AMP_HALF;
                            let sig_i = (data_val + pilot_val) * *amp;

                            let angle = *carrier_phase * std::f64::consts::TAU;
                            let (sv, cv) = angle.sin_cos();
                            *carrier_phase += freq * dt;

                            buf[base + s] = ((sig_i * cv) as f32, (sig_i * sv) as f32);
                        }
                    }
                },
                SatChannel::BeiDou { svid, eph, data_prn, pilot_prn, pilot_secondary, carrier_phase, doppler_hz, travel_time_s, amplitude: amp, bcnav1, nav_bits, current_frame } => {
                    for ms_off in 0..chunk_dur {
                        let global_ms = chunk_start + ms_off as i32;
                        let ms_time = GnssTime { Week: gps_time.Week, MilliSeconds: gps_time.MilliSeconds + global_ms, SubMilliSeconds: 0.0 };
                        let mut sp = SatelliteParam::default();
                        sp.system = GnssSystem::BdsSystem;
                        let re = lla_to_ecef(&receiver_lla);
                        let rl = ecef_to_lla(&re);
                        get_satellite_param(&re, &rl, &ms_time, GnssSystem::BdsSystem, eph, &iono, &mut sp);
                        *doppler_hz = get_doppler(&sp, SIGNAL_INDEX_B1C);
                        *travel_time_s = get_travel_time(&sp, SIGNAL_INDEX_B1C);
                        let freq = if_freq + *doppler_hz;
                        let rx_sow = ms_time.MilliSeconds as f64 / 1000.0;
                        let base = ms_off * spm;

                        // Nav bit timing (integer ms, same approach as Galileo)
                        let travel_ms = (*travel_time_s * 1000.0) as i32;
                        let tx_ms_int = ms_time.MilliSeconds - travel_ms;
                        let fn_ = tx_ms_int / BDS_FRAME_MS;
                        if fn_ != *current_frame {
                            bcnav1.get_frame_data(GnssTime { Week: ms_time.Week, MilliSeconds: tx_ms_int, SubMilliSeconds: 0.0 }, *svid as i32, 0, nav_bits);
                            *current_frame = fn_;
                        }
                        let bi = ((tx_ms_int % BDS_FRAME_MS).max(0) / BDS_NAV_BIT_MS) as usize;
                        let nav_bit: f64 = if nav_bits[bi.min(BDS_BITS_PER_FRAME - 1)] != 0 { -1.0 } else { 1.0 };

                        // Pilot secondary code (1800 bits packed in 57 u32 words, MSB first)
                        let sec_idx = (tx_ms_int / BDS_NAV_BIT_MS).rem_euclid(BDS_SEC_CODE_LEN) as usize;
                        let sec_word = pilot_secondary[sec_idx / 32];
                        let sec_bit_val = (sec_word >> (31 - (sec_idx % 32))) & 1;
                        let sec_bit: f64 = if sec_bit_val != 0 { -1.0 } else { 1.0 };

                        // QMBOC symbol position (matching sat_if_signal.rs)
                        // BOC(6,1) applied at symbol positions 1,5,7,30 within each 330ms (33×10ms) window
                        let symbol_pos = (tx_ms_int % 330) / 10;
                        let is_boc6_symbol = symbol_pos == 1 || symbol_pos == 5 || symbol_pos == 7 || symbol_pos == 30;

                        for s in 0..spm {
                            let rx_time = rx_sow + s as f64 * dt;
                            let tx_time = rx_time - *travel_time_s;

                            let chip_count = tx_time * BDS_CHIP_RATE;
                            let logical_chip = ((chip_count % BDS_CODE_LEN as f64) + BDS_CODE_LEN as f64) as usize % BDS_CODE_LEN;
                            let subchip = (tx_time * BDS_SUBCHIP_RATE) as i64;

                            // BOC(1,1) sign for data channel
                            let boc11: f64 = if subchip & 1 != 0 { -1.0 } else { 1.0 };

                            // QMBOC for pilot: BOC(6,1) at specific symbol positions, BOC(1,1) otherwise
                            let qmboc: f64 = if is_boc6_symbol {
                                let p = subchip.rem_euclid(12);
                                if p >= 6 { -1.0 } else { 1.0 }
                            } else { boc11 };

                            // Data on Q channel, pilot on I channel (BDS-SIS-ICD)
                            let data_val = -nav_bit * data_prn[logical_chip] * boc11 * AMP_1_4;
                            let pilot_val = sec_bit * pilot_prn[logical_chip] * qmboc * AMP_29_44;
                            let sig_i = pilot_val * *amp;
                            let sig_q = data_val * *amp;

                            let angle = *carrier_phase * std::f64::consts::TAU;
                            let (sv, cv) = angle.sin_cos();
                            *carrier_phase += freq * dt;

                            // Complex carrier modulation: (I + jQ)(cos + jsin)
                            buf[base + s] = (((sig_i * cv - sig_q * sv) as f32), ((sig_i * sv + sig_q * cv) as f32));
                        }
                    }
                },
            }
            buf
        }).collect();

        // Sum all satellites + noise + AGC + quantize
        let mut rng = rand::thread_rng();
        let mut out = vec![0u8; chunk_samples * 2];
        for s in 0..chunk_samples {
            let mut i_sum = 0.0_f64;
            let mut q_sum = 0.0_f64;
            for b in &sat_bufs {
                i_sum += b[s].0 as f64;
                q_sum += b[s].1 as f64;
            }
            // Gaussian noise (Box-Muller)
            let u1: f64 = rng.gen::<f64>().max(1e-30);
            let u2: f64 = rng.gen::<f64>();
            let r = noise_sigma * (-2.0 * u1.ln()).sqrt();
            let theta = std::f64::consts::TAU * u2;
            i_sum += r * theta.cos();
            q_sum += r * theta.sin();

            out[s * 2] = (i_sum * agc_gain * 127.0).round().clamp(-128.0, 127.0) as i8 as u8;
            out[s * 2 + 1] = (q_sum * agc_gain * 127.0).round().clamp(-128.0, 127.0) as i8 as u8;
        }
        file.write_all(&out).unwrap();

        if chunk_start % 10000 == 0 || chunk_end == total_ms {
            print!("\r[gnss_pilot] {:.0}%  ", chunk_end as f64 / total_ms as f64 * 100.0);
            std::io::stdout().flush().ok();
        }
    }

    file.flush().unwrap();
    let total_samples = total_ms as usize * spm;
    println!("\r[gnss_pilot] Done! {} GPS + {} GAL + {} BDS, {} samples, {:.1}s → {}",
        gps_count, gal_count, bds_count, total_samples, start.elapsed().as_secs_f64(), output);
}
