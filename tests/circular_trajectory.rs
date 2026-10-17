use gnss_rust::coordinate::{calc_conv_matrix_lla, ecef_to_lla, speed_ecef_to_local};
use gnss_rust::types::{KinematicInfo, LocalSpeed};
use gnss_rust::IFDataGen;

const PRESET: &str = "presets/gps_gal_circle100m_10kmh_4m2_300s.json";

#[test]
fn circular_preset_has_correct_radius_speed_height_and_velocity_derivative() {
    for course in [0.0, 90.0] {
        let mut generator = IFDataGen::new();
        generator.load_config(PRESET).unwrap();
        assert_eq!(generator.trajectory.get_time_length(), 300.0);
        assert_eq!(generator.output_param.SampleFreq, 4_200_000);
        let origin = generator.start_pos;
        let matrix = calc_conv_matrix_lla(&origin);
        // Проверяем также произвольный начальный курс через публичный API траектории.
        generator.trajectory.clear_trajectory_list();
        generator.trajectory.set_init_pos_vel_lla(
            origin,
            LocalSpeed {
                speed: 10.0 / 3.6,
                course: f64::to_radians(course),
                ..Default::default()
            },
            false,
        );
        use gnss_rust::trajectory::{TrajectoryDataType as D, TrajectoryType as T};
        assert_eq!(
            generator.trajectory.append_trajectory(
                T::TrajTypeHorizontalCircular,
                D::TrajDataTimeSpan,
                300.0,
                D::TrajDataRadius,
                100.0,
            ),
            0
        );
        let mut start = KinematicInfo::default();
        assert!(generator.trajectory.get_next_pos_vel_ecef(0.0, &mut start));
        let mut initial = LocalSpeed::default();
        speed_ecef_to_local(&matrix, &start, &mut initial);
        let omega = (10.0 / 3.6) / 100.0;
        let center_e = -initial.vn / omega;
        let center_n = initial.ve / omega;
        let mut prev = start;
        for _ in 0..3000 {
            let mut pos = KinematicInfo::default();
            assert!(generator.trajectory.get_next_pos_vel_ecef(0.1, &mut pos));
            let delta = KinematicInfo {
                vx: pos.x - start.x,
                vy: pos.y - start.y,
                vz: pos.z - start.z,
                ..Default::default()
            };
            let mut enu = LocalSpeed::default();
            speed_ecef_to_local(&matrix, &delta, &mut enu);
            let radius = ((enu.ve - center_e).powi(2) + (enu.vn - center_n).powi(2)).sqrt();
            assert!((radius - 100.0).abs() < 1e-6, "radius={radius}");
            let speed = (pos.vx.powi(2) + pos.vy.powi(2) + pos.vz.powi(2)).sqrt();
            assert!((speed - 10.0 / 3.6).abs() < 1e-10);
            // Горизонтальная плоскость ENU: кривизна Земли даёт менее 4 мм по высоте.
            assert!((ecef_to_lla(&pos).alt - origin.alt).abs() < 0.004);
            for (a, b, va, vb) in [
                (pos.x, prev.x, pos.vx, prev.vx),
                (pos.y, prev.y, pos.vy, prev.vy),
                (pos.z, prev.z, pos.vz, prev.vz),
            ] {
                assert!(((a - b) / 0.1 - (va + vb) / 2.0).abs() < 2e-6);
            }
            prev = pos;
        }
    }
}

#[test]
fn circular_preset_loads_speed_units_and_closes_one_lap() {
    let mut generator = IFDataGen::new();
    generator.load_config(PRESET).unwrap();
    let mut start = KinematicInfo::default();
    assert!(generator.trajectory.get_next_pos_vel_ecef(0.0, &mut start));
    let speed = (start.vx.powi(2) + start.vy.powi(2) + start.vz.powi(2)).sqrt();
    assert!((speed - 10.0 / 3.6).abs() < 1e-10);
    let mut end = KinematicInfo::default();
    let period = 2.0 * std::f64::consts::PI * 100.0 / speed;
    assert!(generator.trajectory.get_next_pos_vel_ecef(period, &mut end));
    assert!(
        ((end.x - start.x).powi(2) + (end.y - start.y).powi(2) + (end.z - start.z).powi(2)).sqrt()
            < 1e-7
    );
}

#[test]
fn generated_satellite_parameters_use_moving_receiver_position_and_velocity() {
    use gnss_rust::coordinate::lla_to_ecef;
    use gnss_rust::satellite_param::get_satellite_param;
    use gnss_rust::types::{GnssSystem, IonoParam, SatelliteParam};
    let dir = std::path::PathBuf::from(format!(".scratch/circular_smoke_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let mut config: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(PRESET).unwrap()).unwrap();
    let output = dir.join("moving.C8");
    config["output"]["name"] = output.to_str().unwrap().into();
    config["output"]["sampleFreq"] = 0.004.into();
    config["trajectory"]["trajectoryList"][0]["time"] = 0.12.into();
    config["power"]["initPower"]["value"] = 52.0.into();
    config["power"]["elevationAdjust"] = false.into();
    config["power"]["signalPower"][0]["powerValue"]["value"] = 58.0.into();
    let path = dir.join("moving.json");
    std::fs::write(&path, serde_json::to_vec(&config).unwrap()).unwrap();
    let mut generator = IFDataGen::new();
    generator.load_config(path.to_str().unwrap()).unwrap();
    generator.initialize().unwrap();
    let time = generator.cur_time.add_milliseconds(119.0);
    let stats = generator.generate_data().unwrap();
    assert_eq!(stats.total_samples, 480);
    assert_eq!(std::fs::metadata(&output).unwrap().len(), 960);
    generator.trajectory.reset_trajectory_time();
    let mut receiver = KinematicInfo::default();
    assert!(generator
        .trajectory
        .get_next_pos_vel_ecef(0.119, &mut receiver));
    let lla = ecef_to_lla(&receiver);
    let stationary = lla_to_ecef(&generator.start_pos);
    let mut max_difference: f64 = 0.0;
    for (system, ephs, actual) in [
        (
            GnssSystem::GpsSystem,
            &generator.gps_eph_visible[..],
            &generator.gps_sat_param[..],
        ),
        (
            GnssSystem::GalileoSystem,
            &generator.gal_eph_visible[..],
            &generator.gal_sat_param[..],
        ),
    ] {
        let mut count = 0;
        for eph in ephs.iter().flatten() {
            let mut expected = SatelliteParam::default();
            get_satellite_param(
                &receiver,
                &lla,
                &time,
                system,
                eph,
                &IonoParam::default(),
                &mut expected,
            );
            let actual = &actual[eph.svid as usize - 1];
            let expected_cn0 = if system == GnssSystem::GalileoSystem {
                5800
            } else {
                5200
            };
            assert_eq!(
                actual.CN0, expected_cn0,
                "power override must survive every ms"
            );
            assert!((actual.RelativeSpeed - expected.RelativeSpeed).abs() < 1e-6);
            let mut fixed = SatelliteParam::default();
            get_satellite_param(
                &stationary,
                &generator.start_pos,
                &time,
                system,
                eph,
                &IonoParam::default(),
                &mut fixed,
            );
            max_difference = max_difference.max((actual.RelativeSpeed - fixed.RelativeSpeed).abs());
            count += 1;
        }
        assert!(count > 0, "no satellites for {system:?}");
    }
    assert!(
        max_difference > 0.5,
        "receiver velocity must change satellite Doppler"
    );
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn power_override_schedule_is_sorted_and_loading_does_not_accumulate_events() {
    let dir = std::path::PathBuf::from(format!(".scratch/power_schedule_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let mut config: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(PRESET).unwrap()).unwrap();
    config["power"]["signalPower"][0]["powerValue"] = serde_json::json!([
        {"unit":"dBHz","value":60.0,"time":0.05},
        {"unit":"dBHz","value":58.0,"time":0.0}
    ]);
    let path = dir.join("power.json");
    std::fs::write(&path, serde_json::to_vec(&config).unwrap()).unwrap();
    let mut generator = IFDataGen::new();
    for _ in 0..2 {
        generator.load_config(path.to_str().unwrap()).unwrap();
        let entries = &generator.power_control.power_control_array;
        assert_eq!(entries.len(), 2);
        assert_eq!(
            (
                entries[0].system,
                entries[0].svid,
                entries[0].time,
                entries[0].cn0
            ),
            (2, 0, 0, 58.0)
        );
        assert_eq!((entries[1].time, entries[1].cn0), (50, 60.0));
    }
    std::fs::remove_dir_all(dir).unwrap();
}
