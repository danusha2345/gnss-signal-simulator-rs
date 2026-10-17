//! Проверки реального тракта генерации на границах миллисекунд и блоков.
use gnss_rust::constants::{SIGNAL_INDEX_E1, SIGNAL_INDEX_L1CA};
use gnss_rust::coordinate::ecef_to_lla;
use gnss_rust::nav_data::NavData;
use gnss_rust::sat_if_signal::SatIfSignal;
use gnss_rust::satellite_param::{get_doppler, get_satellite_param};
use gnss_rust::types::{GnssSystem, KinematicInfo, SatelliteParam};
use gnss_rust::IFDataGen;

const PRESET: &str = "presets/gps_gal_circle100m_10kmh_4m2_300s.json";

#[test]
fn circular_signal_keeps_carrier_continuous_across_parameter_updates() {
    let mut generator = IFDataGen::new();
    generator.load_config(PRESET).unwrap();
    for (system, signal, svid) in [
        (GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA, 1u8),
        (GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA, 13),
        (GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA, 30),
        (GnssSystem::GalileoSystem, SIGNAL_INDEX_E1, 2),
        (GnssSystem::GalileoSystem, SIGNAL_INDEX_E1, 29),
        (GnssSystem::GalileoSystem, SIGNAL_INDEX_E1, 30),
    ] {
        let gps = system == GnssSystem::GpsSystem;
        let eph = if gps {
            generator.gps_eph[svid as usize - 1]
        } else {
            generator.gal_eph[svid as usize - 1]
        }
        .unwrap();
        let iono = generator
            .nav_data
            .get_gps_iono()
            .copied()
            .unwrap_or_default();
        for start in [0, 110_000, 224_000, 298_000] {
            generator.trajectory.reset_trajectory_time();
            let mut pos = KinematicInfo::default();
            assert!(generator
                .trajectory
                .get_next_pos_vel_ecef(start as f64 / 1000.0, &mut pos));
            let time = generator.cur_time.add_milliseconds(start as f64);
            let mut param = SatelliteParam::default();
            get_satellite_param(
                &pos,
                &ecef_to_lla(&pos),
                &time,
                system,
                &eph,
                &iono,
                &mut param,
            );
            param.CN0 = 5200;
            let mut nav = if gps {
                NavData::new_gps(1)
            } else {
                NavData::new_galileo(1)
            }
            .unwrap();
            nav.set_ephemeris(svid as i32, &eph);
            if !gps {
                nav.set_galileo_iono_utc(
                    generator.nav_data.get_galileo_nequick(),
                    generator.nav_data.get_galileo_utc_param(),
                );
            }
            let mut sig = SatIfSignal::new(4200, 0, system, signal as i32, svid);
            sig.init_state(time, &param, Some(nav));
            let mut last_phase = 0.0;
            let mut last_step = 0.0;
            let mut gap = 1.0;
            let mut worst: f64 = 0.0;
            for ms in 0..250 {
                if ms > 0 {
                    assert!(generator.trajectory.get_next_pos_vel_ecef(0.001, &mut pos));
                }
                let t = time.add_milliseconds(ms as f64);
                get_satellite_param(
                    &pos,
                    &ecef_to_lla(&pos),
                    &t,
                    system,
                    &eph,
                    &iono,
                    &mut param,
                );
                param.CN0 = 5200;
                if ms % 50 == 0 {
                    sig.update_satellite_params(&param, 1575.42e6, &t);
                } else {
                    sig.push_sat_param_for_ms(&param, 1575.42e6, &t);
                }
                let step = std::f64::consts::TAU * get_doppler(&param, signal) / 4.2e6;
                sig.get_if_sample_cached(t);
                // Сумма data+pilot E1 иногда равна нулю: у нулевого отсчёта нет фазы.
                let nonzero = |s: &gnss_rust::complex_number::ComplexNumber| {
                    s.real * s.real + s.imag * s.imag > 1e-20
                };
                let first = sig.sample_array.iter().position(nonzero).unwrap();
                let last = sig.sample_array.iter().rposition(nonzero).unwrap();
                let arg = |i: usize| sig.sample_array[i].imag.atan2(sig.sample_array[i].real);
                if ms > 0 {
                    let difference =
                        arg(first) - last_phase - last_step * gap - step * first as f64;
                    let residual = (difference + std::f64::consts::FRAC_PI_2)
                        .rem_euclid(std::f64::consts::PI)
                        - std::f64::consts::FRAC_PI_2;
                    worst = worst.max(residual.abs());
                }
                last_phase = arg(last);
                gap = (4200 - last) as f64;
                last_step = step;
            }
            assert!(
                worst < 0.0002,
                "{system:?} SV{svid} start={start}: phase discontinuity {worst}"
            );
        }
    }
}

#[test]
fn power_event_is_applied_at_50ms_not_49ms_and_startup_agc_uses_actual_mix() {
    let dir = std::path::PathBuf::from(format!(".scratch/power_boundary_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for (duration, expected_cn0, last_ms) in [(0.050, 5200, 49), (0.051, 3500, 50)] {
        let mut config: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(PRESET).unwrap()).unwrap();
        let output = dir.join(format!("{last_ms}.C8"));
        config["output"]["name"] = output.to_str().unwrap().into();
        config["trajectory"]["trajectoryList"][0]["time"] = duration.into();
        config["power"]["initPower"]["value"] = 52.into();
        config["power"]["elevationAdjust"] = false.into();
        config["power"]["signalPower"] = serde_json::json!([
            {"system":"Galileo", "powerValue":{"value":74, "unit":"dBHz"}},
            {"system":"GPS", "powerValue":{"time":0.05, "value":35, "unit":"dBHz"}}
        ]);
        let path = dir.join(format!("{last_ms}.json"));
        std::fs::write(&path, serde_json::to_vec(&config).unwrap()).unwrap();
        let mut generator = IFDataGen::new();
        generator.load_config(path.to_str().unwrap()).unwrap();
        let gal = generator.nav_data.get_galileo_nequick().unwrap();
        assert_eq!((gal.ai0, gal.ai1, gal.ai2), (140.5, 0.14844, 0.011475));
        assert_eq!(generator.nav_data.get_galileo_utc_param().unwrap().tot, 72);
        generator.generate_data().unwrap();
        assert_eq!(generator.power_control.time_elaps_ms, last_ms);
        assert!(generator.gps_sat_number > 0);
        for eph in generator.gps_eph_visible.iter().flatten() {
            assert_eq!(
                generator.gps_sat_param[eph.svid as usize - 1].CN0,
                expected_cn0
            );
        }
        let bytes = std::fs::read(output).unwrap();
        let first_block = &bytes[..4200 * 50 * 2];
        let rms = (first_block
            .iter()
            .map(|&b| (b as i8 as f64 / 127.0).powi(2))
            .sum::<f64>()
            / (first_block.len() / 2) as f64)
            .sqrt();
        assert!((rms - 0.4).abs() < 0.01, "startup RMS={rms}");
    }
    std::fs::remove_dir_all(dir).unwrap();
}
