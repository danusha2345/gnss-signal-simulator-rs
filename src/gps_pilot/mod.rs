//! Clean GPS L1CA signal generator.
//!
//! Stateless code phase, continuous carrier phase accumulation.
//! GPS LNAV navigation data modulation (±1 every 20ms).

use crate::constants::*;
use crate::coordinate::{ecef_to_lla, lla_to_ecef};
use crate::json_interpreter::{read_nav_file_limited, CNavData};
use crate::lnavbit::LNavBit;
use crate::prngenerate::PrnGenerate;
use crate::satellite_param::{get_doppler, get_satellite_param, get_travel_time};
use crate::types::*;
use rand::Rng;
use rayon::prelude::*;
use std::io::Write;

const CHIP_RATE: f64 = 1_023_000.0; // GPS L1CA chips/second
const CODE_LEN: usize = 1023;
const FREQ_L1: f64 = 1_575_420_000.0; // Hz

pub struct GpsPilotConfig {
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
    prn_code: [f64; CODE_LEN],
    carrier_phase: f64,
    doppler_hz: f64,
    travel_time_s: f64,
    amplitude: f64,
    lnav: LNavBit,
    nav_bits: [i32; 300],
    current_frame: i32,
}

fn find_best_ephemeris(nav_data: &CNavData, svid: u8, target_sow: f64) -> Option<GpsEphemeris> {
    nav_data
        .gps_ephemeris
        .iter()
        .filter(|e| e.svid == svid && e.valid != 0)
        .min_by(|a, b| {
            let da = (a.toe as f64 - target_sow).abs();
            let db = (b.toe as f64 - target_sow).abs();
            da.partial_cmp(&db).unwrap()
        })
        .copied()
}

fn gaussian_pair(rng: &mut impl Rng, sigma: f64) -> (f64, f64) {
    let u1: f64 = rng.gen::<f64>().max(1e-30);
    let u2: f64 = rng.gen::<f64>();
    let r = sigma * (-2.0 * u1.ln()).sqrt();
    let theta = std::f64::consts::TAU * u2;
    (r * theta.cos(), r * theta.sin())
}

pub fn generate(config: &GpsPilotConfig) -> Result<GenerationResult, Box<dyn std::error::Error>> {
    // 1. Load RINEX
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, &config.rinex_path, 10000);

    // 2. Convert time
    let gps_time = crate::gnsstime::utc_to_gps_time(config.utc_time, true);
    let target_sow = gps_time.MilliSeconds as f64 / 1000.0;

    // 3. Find visible satellites
    let receiver_ecef = lla_to_ecef(&config.receiver_lla);
    let receiver_lla = ecef_to_lla(&receiver_ecef);
    let iono = IonoParam::default();

    let amplitude = (10.0_f64.powf(config.cn0_db / 10.0) / config.sample_rate_hz).sqrt();

    let mut channels: Vec<SatelliteChannel> = Vec::new();
    let mut sat_infos: Vec<SatelliteInfo> = Vec::new();

    for svid in 1u8..=32 {
        let eph = match find_best_ephemeris(&nav_data, svid, target_sow) {
            Some(e) => e,
            None => continue,
        };

        let mut sat_param = SatelliteParam::default();
        sat_param.system = GnssSystem::GpsSystem;
        get_satellite_param(
            &receiver_ecef,
            &receiver_lla,
            &gps_time,
            GnssSystem::GpsSystem,
            &eph,
            &iono,
            &mut sat_param,
        );

        if sat_param.Elevation < config.elevation_mask_deg.to_radians() {
            continue;
        }

        // Generate PRN code
        let prn_gen = PrnGenerate::new(
            GnssSystem::GpsSystem,
            SIGNAL_INDEX_L1CA as i32,
            svid as i32,
        );
        let raw_code = match prn_gen.get_data_prn() {
            Some(c) => c,
            None => continue,
        };
        let mut prn_code = [0.0f64; CODE_LEN];
        for i in 0..CODE_LEN {
            prn_code[i] = if raw_code[i] != 0 { -1.0 } else { 1.0 };
        }

        let doppler = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);
        let travel_time = get_travel_time(&sat_param, SIGNAL_INDEX_L1CA);

        // Initialize LNAV encoder with ephemeris + iono/UTC
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

        sat_infos.push(SatelliteInfo {
            svid,
            elevation_deg: sat_param.Elevation.to_degrees(),
            azimuth_deg: sat_param.Azimuth.to_degrees(),
            doppler_hz: doppler,
        });

        channels.push(SatelliteChannel {
            svid,
            eph,
            prn_code,
            carrier_phase: 0.0,
            doppler_hz: doppler,
            travel_time_s: travel_time,
            amplitude,
            lnav,
            nav_bits: [0i32; 300],
            current_frame: -1,
        });
    }

    println!(
        "[gps_pilot] {} visible GPS satellites",
        channels.len()
    );
    for info in &sat_infos {
        println!(
            "  PRN {:02}: el={:.1}° az={:.1}° doppler={:.0} Hz",
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
        "[gps_pilot] Generating {:.1}s, {} samples/ms, AGC gain={:.4}",
        config.duration_s, samples_per_ms, agc_gain
    );

    let mut file = std::io::BufWriter::with_capacity(
        1 << 20,
        std::fs::File::create(&config.output_path)?,
    );

    let chunk_ms = 1000; // 1 second chunks

    for chunk_start in (0..total_ms).step_by(chunk_ms as usize) {
        let chunk_end = (chunk_start + chunk_ms).min(total_ms);
        let chunk_duration = (chunk_end - chunk_start) as usize;
        let chunk_samples = chunk_duration * samples_per_ms;

        // Generate per-satellite buffers in parallel
        let sat_buffers: Vec<Vec<(f32, f32)>> = channels
            .par_iter_mut()
            .map(|ch| {
                let mut buf = vec![(0.0f32, 0.0f32); chunk_samples];

                for ms_offset in 0..chunk_duration {
                    let global_ms = chunk_start + ms_offset as i32;

                    // Update satellite params every ms
                    let ms_time = GnssTime {
                        Week: gps_time.Week,
                        MilliSeconds: gps_time.MilliSeconds + global_ms,
                        SubMilliSeconds: 0.0,
                    };

                    let mut sat_param = SatelliteParam::default();
                    sat_param.system = GnssSystem::GpsSystem;
                    let receiver_ecef_local = lla_to_ecef(&config.receiver_lla);
                    let receiver_lla_local = ecef_to_lla(&receiver_ecef_local);
                    get_satellite_param(
                        &receiver_ecef_local,
                        &receiver_lla_local,
                        &ms_time,
                        GnssSystem::GpsSystem,
                        &ch.eph,
                        &iono,
                        &mut sat_param,
                    );

                    ch.doppler_hz = get_doppler(&sat_param, SIGNAL_INDEX_L1CA);
                    ch.travel_time_s = get_travel_time(&sat_param, SIGNAL_INDEX_L1CA);

                    let freq = config.if_freq_hz + ch.doppler_hz;
                    let rx_time_sow =
                        ms_time.MilliSeconds as f64 / 1000.0 + ms_time.SubMilliSeconds;

                    let base_idx = ms_offset * samples_per_ms;

                    for s in 0..samples_per_ms {
                        let t_within_ms = s as f64 * dt;

                        // Code phase: stateless from transmit time
                        let rx_time = rx_time_sow + t_within_ms;
                        let tx_time = rx_time - ch.travel_time_s;
                        let chip_count = tx_time * CHIP_RATE;
                        let chip_index =
                            ((chip_count % CODE_LEN as f64) + CODE_LEN as f64) as usize
                                % CODE_LEN;
                        let prn = ch.prn_code[chip_index];

                        // Nav bit: aligned to transmit time (code epoch), not receiver time
                        let tx_ms_sample = (tx_time * 1000.0) as i32;
                        let frame_number = tx_ms_sample / 6000;
                        if frame_number != ch.current_frame {
                            let tx_time_gnss = GnssTime {
                                Week: ms_time.Week,
                                MilliSeconds: tx_ms_sample,
                                SubMilliSeconds: 0.0,
                            };
                            ch.lnav.get_frame_data(
                                tx_time_gnss, ch.svid as i32, 0, &mut ch.nav_bits,
                            );
                            ch.current_frame = frame_number;
                        }
                        let bit_index = ((tx_ms_sample % 6000) / 20) as usize;
                        let nav_bit: f64 =
                            if ch.nav_bits[bit_index.min(299)] != 0 { -1.0 } else { 1.0 };

                        // Carrier phase: continuous accumulation
                        let angle = ch.carrier_phase * std::f64::consts::TAU;
                        let (sin_v, cos_v) = angle.sin_cos();
                        ch.carrier_phase += freq * dt;

                        buf[base_idx + s] = (
                            (prn * nav_bit * cos_v * ch.amplitude) as f32,
                            (prn * nav_bit * sin_v * ch.amplitude) as f32,
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
        print!("\r[gps_pilot] {:.0}%  ", progress);
        std::io::stdout().flush().ok();
    }

    file.flush()?;
    println!("\r[gps_pilot] Done! {} samples written to {}", total_samples, config.output_path);

    Ok(GenerationResult {
        satellites: sat_infos,
        total_samples,
    })
}
