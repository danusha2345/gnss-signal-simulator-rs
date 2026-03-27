//! Clean Galileo E1 signal generator.
//!
//! Stateless code phase, continuous carrier phase accumulation.
//! CBOC(6,1,1/11) modulation, I-NAV navigation data, CS25 pilot secondary code.
//! Nav bits aligned to transmit time (code epoch) for real receiver compatibility.

use crate::constants::*;
use crate::coordinate::{ecef_to_lla, lla_to_ecef};
use crate::inavbit::INavBit;
use crate::json_interpreter::{read_nav_file_limited, CNavData};
use crate::prngenerate::PrnGenerate;
use crate::satellite_param::{get_doppler, get_satellite_param, get_travel_time};
use crate::types::*;
use rand::Rng;
use rayon::prelude::*;
use std::io::Write;

const CHIP_RATE: f64 = 1_023_000.0; // Logical chips/second
const SUBCHIP_RATE: f64 = 2_046_000.0; // BOC(1,1) subchips/second
const CODE_LEN: usize = 4092; // Chips per code period (4ms)
const NAV_BIT_PERIOD_MS: i32 = 4; // 4ms per I-NAV symbol
const FRAME_MS: i32 = 2000; // 2-second I-NAV frame
const NAV_BITS_PER_FRAME: usize = 500; // bits per frame
const SECONDARY_CODE: u32 = 0x0E64C2E0; // CS25 reversed for bit extraction
const SECONDARY_CODE_RAW: u32 = 0x9b501c; // CS25 as stored in pilotbit.rs
const SECONDARY_CODE_LEN: i32 = 25;
const AMPLITUDE_HALF: f64 = std::f64::consts::FRAC_1_SQRT_2; // 1/sqrt(2)

pub struct GalileoPilotConfig {
    pub rinex_path: String,
    pub output_path: String,
    pub receiver_lla: LlaPosition,
    pub utc_time: UtcTime,
    pub duration_s: f64,
    pub sample_rate_hz: f64,
    pub if_freq_hz: f64,
    pub cn0_db: f64,
    pub elevation_mask_deg: f64,
}

pub struct SatelliteInfo {
    pub svid: u8,
    pub elevation_deg: f64,
    pub azimuth_deg: f64,
    pub doppler_hz: f64,
}

pub struct GenerationResult {
    pub satellites: Vec<SatelliteInfo>,
    pub total_samples: usize,
}

struct SatelliteChannel {
    svid: u8,
    eph: GpsEphemeris,
    data_prn: Vec<f64>,  // 4092 chips, bipolar
    pilot_prn: Vec<f64>, // 4092 chips, bipolar
    carrier_phase: f64,
    doppler_hz: f64,
    travel_time_s: f64,
    amplitude: f64,
    inav: INavBit,
    nav_bits: Vec<i32>,
    current_frame: i32,
}

fn find_best_galileo_ephemeris(
    nav_data: &CNavData,
    svid: u8,
    target_sow: f64,
) -> Option<GpsEphemeris> {
    nav_data
        .galileo_ephemeris
        .iter()
        .filter(|e| e.svid == svid && e.valid != 0)
        .min_by(|a, b| {
            let da = (a.toe as f64 - target_sow).abs();
            let db = (b.toe as f64 - target_sow).abs();
            da.partial_cmp(&db).unwrap()
        })
        .copied()
}

/// Extract CS25 secondary code bit (0x9b501c, 25 bits, MSB first)
fn get_secondary_code_bit(code_period_index: i32) -> f64 {
    let idx = code_period_index.rem_euclid(SECONDARY_CODE_LEN) as u32;
    // CS25: bit 0 is MSB (bit 24 of u32)
    if (SECONDARY_CODE_RAW >> (24 - idx)) & 1 != 0 {
        -1.0
    } else {
        1.0
    }
}

/// BOC(1,1) sign: flip on odd subchips
fn boc11_sign(subchip_count: i64) -> f64 {
    if subchip_count & 1 != 0 {
        -1.0
    } else {
        1.0
    }
}

/// CBOC sign for pilot: BOC(6,1) on every 11th chip, BOC(1,1) elsewhere
fn cboc_sign(subchip_count: i64, logical_chip: i64) -> f64 {
    let chip_in_code = logical_chip.rem_euclid(CODE_LEN as i64);
    if chip_in_code % 11 == 0 {
        // BOC(6,1): 12-subchip cycle, flip at phase >= 6
        let boc6_phase = subchip_count.rem_euclid(12);
        if boc6_phase >= 6 { -1.0 } else { 1.0 }
    } else {
        // BOC(1,1): flip on odd subchip
        boc11_sign(subchip_count)
    }
}

fn gaussian_pair(rng: &mut impl Rng, sigma: f64) -> (f64, f64) {
    let u1: f64 = rng.gen::<f64>().max(1e-30);
    let u2: f64 = rng.gen::<f64>();
    let r = sigma * (-2.0 * u1.ln()).sqrt();
    let theta = std::f64::consts::TAU * u2;
    (r * theta.cos(), r * theta.sin())
}

pub fn generate(
    config: &GalileoPilotConfig,
) -> Result<GenerationResult, Box<dyn std::error::Error>> {
    // 1. Load RINEX
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, &config.rinex_path, 10000);

    // 2. Convert time (use GPS time directly, not GST)
    let gps_time = crate::gnsstime::utc_to_gps_time(config.utc_time, true);
    let target_sow = gps_time.MilliSeconds as f64 / 1000.0;

    // 3. Find visible Galileo satellites
    let receiver_ecef = lla_to_ecef(&config.receiver_lla);
    let receiver_lla = ecef_to_lla(&receiver_ecef);
    let iono = IonoParam::default();

    let amplitude = (10.0_f64.powf(config.cn0_db / 10.0) / config.sample_rate_hz).sqrt();

    let mut channels: Vec<SatelliteChannel> = Vec::new();
    let mut sat_infos: Vec<SatelliteInfo> = Vec::new();

    for svid in 1u8..=36 {
        let eph = match find_best_galileo_ephemeris(&nav_data, svid, target_sow) {
            Some(e) => e,
            None => continue,
        };

        let mut sat_param = SatelliteParam::default();
        sat_param.system = GnssSystem::GalileoSystem;
        get_satellite_param(
            &receiver_ecef,
            &receiver_lla,
            &gps_time,
            GnssSystem::GalileoSystem,
            &eph,
            &iono,
            &mut sat_param,
        );

        if sat_param.Elevation < config.elevation_mask_deg.to_radians() {
            continue;
        }

        // Generate PRN codes (4092 chips each)
        let prn_gen = PrnGenerate::new(
            GnssSystem::GalileoSystem,
            SIGNAL_INDEX_E1 as i32,
            svid as i32,
        );
        let raw_data = match prn_gen.get_data_prn() {
            Some(c) => c.to_vec(),
            None => continue,
        };
        let raw_pilot = match prn_gen.get_pilot_prn() {
            Some(c) => c.to_vec(),
            None => continue,
        };

        let data_prn: Vec<f64> = raw_data
            .iter()
            .map(|&v| if v != 0 { -1.0 } else { 1.0 })
            .collect();
        let pilot_prn: Vec<f64> = raw_pilot
            .iter()
            .map(|&v| if v != 0 { -1.0 } else { 1.0 })
            .collect();

        let doppler = get_doppler(&sat_param, SIGNAL_INDEX_E1);
        let travel_time = get_travel_time(&sat_param, SIGNAL_INDEX_E1);

        // Initialize I-NAV encoder
        let mut inav = INavBit::new();
        inav.set_ephemeris(svid as i32, &eph);

        sat_infos.push(SatelliteInfo {
            svid,
            elevation_deg: sat_param.Elevation.to_degrees(),
            azimuth_deg: sat_param.Azimuth.to_degrees(),
            doppler_hz: doppler,
        });

        channels.push(SatelliteChannel {
            svid,
            eph,
            data_prn,
            pilot_prn,
            carrier_phase: 0.0,
            doppler_hz: doppler,
            travel_time_s: travel_time,
            amplitude,
            inav,
            nav_bits: vec![0i32; NAV_BITS_PER_FRAME],
            current_frame: -1,
        });
    }

    println!("[galileo_pilot] {} visible Galileo satellites", channels.len());
    for info in &sat_infos {
        println!(
            "  E{:02}: el={:.1}\u{00b0} az={:.1}\u{00b0} doppler={:.0} Hz",
            info.svid, info.elevation_deg, info.azimuth_deg, info.doppler_hz
        );
    }

    // 4. Generate signal in 1-second chunks
    let total_ms = (config.duration_s * 1000.0) as i32;
    let samples_per_ms = (config.sample_rate_hz / 1000.0) as usize;
    let total_samples = total_ms as usize * samples_per_ms;
    let dt = 1.0 / config.sample_rate_hz;

    let noise_sigma = 0.5;
    let total_sig_rms = (channels.len() as f64).sqrt() * amplitude;
    let total_rms = (total_sig_rms * total_sig_rms + noise_sigma * noise_sigma).sqrt();
    let agc_gain = 0.25 / total_rms;

    println!(
        "[galileo_pilot] Generating {:.1}s, {} samples/ms, AGC gain={:.4}",
        config.duration_s, samples_per_ms, agc_gain
    );

    let mut file = std::io::BufWriter::with_capacity(
        1 << 20,
        std::fs::File::create(&config.output_path)?,
    );

    let chunk_ms = 1000;

    for chunk_start in (0..total_ms).step_by(chunk_ms as usize) {
        let chunk_end = (chunk_start + chunk_ms).min(total_ms);
        let chunk_duration = (chunk_end - chunk_start) as usize;
        let chunk_samples = chunk_duration * samples_per_ms;

        let sat_buffers: Vec<Vec<(f32, f32)>> = channels
            .par_iter_mut()
            .map(|ch| {
                let mut buf = vec![(0.0f32, 0.0f32); chunk_samples];

                for ms_offset in 0..chunk_duration {
                    let global_ms = chunk_start + ms_offset as i32;

                    let ms_time = GnssTime {
                        Week: gps_time.Week,
                        MilliSeconds: gps_time.MilliSeconds + global_ms,
                        SubMilliSeconds: 0.0,
                    };

                    // Update satellite params every ms
                    let mut sat_param = SatelliteParam::default();
                    sat_param.system = GnssSystem::GalileoSystem;
                    let receiver_ecef_local = lla_to_ecef(&config.receiver_lla);
                    let receiver_lla_local = ecef_to_lla(&receiver_ecef_local);
                    get_satellite_param(
                        &receiver_ecef_local,
                        &receiver_lla_local,
                        &ms_time,
                        GnssSystem::GalileoSystem,
                        &ch.eph,
                        &iono,
                        &mut sat_param,
                    );

                    ch.doppler_hz = get_doppler(&sat_param, SIGNAL_INDEX_E1);
                    ch.travel_time_s = get_travel_time(&sat_param, SIGNAL_INDEX_E1);

                    let freq = config.if_freq_hz + ch.doppler_hz;
                    let rx_time_sow =
                        ms_time.MilliSeconds as f64 / 1000.0 + ms_time.SubMilliSeconds;

                    let base_idx = ms_offset * samples_per_ms;

                    for s in 0..samples_per_ms {
                        let t_within_ms = s as f64 * dt;

                        // Transmit time (stateless)
                        let rx_time = rx_time_sow + t_within_ms;
                        let tx_time = rx_time - ch.travel_time_s;

                        // Logical chip index (4092 chips at 1.023 MHz)
                        let chip_count = tx_time * CHIP_RATE;
                        let logical_chip =
                            ((chip_count % CODE_LEN as f64) + CODE_LEN as f64) as usize
                                % CODE_LEN;

                        // Subchip for BOC modulation (2.046 MHz)
                        let subchip_count = (tx_time * SUBCHIP_RATE) as i64;
                        let logical_chip_i64 = (tx_time * CHIP_RATE) as i64;

                        // Nav bit: aligned to transmit time code epoch (4ms periods)
                        let tx_ms = (tx_time * 1000.0) as i32;
                        let frame_number = tx_ms / FRAME_MS;
                        if frame_number != ch.current_frame {
                            let tx_time_gnss = GnssTime {
                                Week: ms_time.Week,
                                MilliSeconds: tx_ms,
                                SubMilliSeconds: 0.0,
                            };
                            ch.inav.get_frame_data(
                                tx_time_gnss,
                                ch.svid as i32,
                                1, // param=1 for E1 I-NAV
                                &mut ch.nav_bits,
                            );
                            ch.current_frame = frame_number;
                        }
                        let bit_index =
                            ((tx_ms % FRAME_MS) / NAV_BIT_PERIOD_MS) as usize;
                        let nav_bit: f64 =
                            if ch.nav_bits[bit_index.min(NAV_BITS_PER_FRAME - 1)] != 0 {
                                -1.0
                            } else {
                                1.0
                            };

                        // Secondary code: 1 bit per 4ms code period
                        let code_period = tx_ms / NAV_BIT_PERIOD_MS;
                        let sec_bit = get_secondary_code_bit(code_period);

                        // Data channel: -nav_bit * data_prn * BOC(1,1) * 1/sqrt(2)
                        let data_val = -nav_bit
                            * ch.data_prn[logical_chip]
                            * boc11_sign(subchip_count)
                            * AMPLITUDE_HALF;

                        // Pilot channel: sec_bit * pilot_prn * CBOC * 1/sqrt(2)
                        let pilot_val = sec_bit
                            * ch.pilot_prn[logical_chip]
                            * cboc_sign(subchip_count, logical_chip_i64)
                            * AMPLITUDE_HALF;

                        // Combined signal on I channel only
                        let signal_i = (data_val + pilot_val) * ch.amplitude;

                        // Carrier phase: continuous accumulation
                        let angle = ch.carrier_phase * std::f64::consts::TAU;
                        let (sin_v, cos_v) = angle.sin_cos();
                        ch.carrier_phase += freq * dt;

                        buf[base_idx + s] = (
                            (signal_i * cos_v) as f32,
                            (signal_i * sin_v) as f32,
                        );
                    }
                }
                buf
            })
            .collect();

        // Sum + noise + quantize + write
        let mut rng = rand::thread_rng();
        let mut output_buf = vec![0u8; chunk_samples * 2];

        for s in 0..chunk_samples {
            let mut i_sum = 0.0_f64;
            let mut q_sum = 0.0_f64;
            for buf in &sat_buffers {
                i_sum += buf[s].0 as f64;
                q_sum += buf[s].1 as f64;
            }
            let (ni, nq) = gaussian_pair(&mut rng, noise_sigma);
            i_sum += ni;
            q_sum += nq;

            let i8_val = (i_sum * agc_gain * 127.0).round().clamp(-128.0, 127.0) as i8;
            let q8_val = (q_sum * agc_gain * 127.0).round().clamp(-128.0, 127.0) as i8;
            output_buf[s * 2] = i8_val as u8;
            output_buf[s * 2 + 1] = q8_val as u8;
        }

        file.write_all(&output_buf)?;

        let progress = chunk_end as f64 / total_ms as f64 * 100.0;
        print!("\r[galileo_pilot] {:.0}%  ", progress);
        std::io::stdout().flush().ok();
    }

    file.flush()?;
    println!(
        "\r[galileo_pilot] Done! {} samples written to {}",
        total_samples, config.output_path
    );

    Ok(GenerationResult {
        satellites: sat_infos,
        total_samples,
    })
}
