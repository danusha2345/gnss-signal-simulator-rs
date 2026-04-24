use gnss_rust::constants::{
    SIGNAL_INDEX_B1C, SIGNAL_INDEX_E1, SIGNAL_INDEX_G1, SIGNAL_INDEX_L1CA, SIGNAL_INDEX_L5,
};
use gnss_rust::powercontrol::{CPowerControl, ElevationAdjust, SignalPower};
use gnss_rust::trajectory::{CTrajectory, TrajectoryDataType, TrajectoryType, TRAJECTORY_NO_ERR};
use gnss_rust::types::{
    CompactConfig, KinematicInfo, GEN_B1C, GEN_E1, GEN_G1, GEN_L1CA, GEN_L5, PARSE_BDS,
    PARSE_GALILEO, PARSE_GLONASS, PARSE_GPS,
};

fn approx_eq(actual: f64, expected: f64, tolerance: f64, name: &str) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{name}: expected {expected}, got {actual}"
    );
}

#[test]
fn compact_config_maps_parse_and_signal_bits() {
    let mut config = CompactConfig::new();
    assert!(!config.should_parse_gps());
    assert!(!config.is_signal_enabled(GEN_L1CA));

    config.enable_system_parsing(PARSE_GPS);
    config.enable_system_parsing(PARSE_BDS);
    config.enable_system_parsing(PARSE_GALILEO);
    config.enable_system_parsing(PARSE_GLONASS);

    assert!(config.should_parse_gps());
    assert!(config.should_parse_bds());
    assert!(config.should_parse_galileo());
    assert!(config.should_parse_glonass());

    for (signal_index, bit) in [
        (SIGNAL_INDEX_L1CA, GEN_L1CA),
        (SIGNAL_INDEX_L5, GEN_L5),
        (SIGNAL_INDEX_B1C, GEN_B1C),
        (SIGNAL_INDEX_E1, GEN_E1),
        (SIGNAL_INDEX_G1, GEN_G1),
    ] {
        assert_eq!(
            CompactConfig::signal_index_to_gen_bit(signal_index),
            Some(bit)
        );
        config.enable_signal(bit);
        assert!(config.is_signal_enabled(bit));
    }

    assert_eq!(CompactConfig::signal_index_to_gen_bit(7), None);
    assert_eq!(CompactConfig::signal_index_to_gen_bit(31), None);
}

#[test]
fn power_control_sorts_and_releases_events_by_elapsed_time() {
    let mut power = CPowerControl::new();
    power.set_noise_floor(-169.5);
    power.set_init_cn0(45.5);
    power.set_elevation_adjust(ElevationAdjust::ElevationAdjustSinSqrtFade);

    for (time, cn0) in [(30, 42.0), (10, 48.0), (20, 44.0)] {
        power.add_control_element(&SignalPower {
            system: 0,
            svid: 3,
            time,
            cn0,
        });
    }

    power.sort();
    assert_eq!(power.get_noise_floor(), -169.5);
    assert_eq!(power.get_init_cn0(), 45.5);
    assert_eq!(
        power.get_elevation_adjust(),
        ElevationAdjust::ElevationAdjustSinSqrtFade
    );

    let first_batch: Vec<i32> = {
        let (items, count) = power.get_power_control_list(15);
        assert_eq!(count, 1);
        items.iter().map(|item| item.time).collect()
    };
    assert_eq!(first_batch, vec![10]);

    let second_batch: Vec<i32> = {
        let (items, count) = power.get_power_control_list(10);
        assert_eq!(count, 1);
        items.iter().map(|item| item.time).collect()
    };
    assert_eq!(second_batch, vec![20]);

    let third_batch: Vec<i32> = {
        let (items, count) = power.get_power_control_list(10);
        assert_eq!(count, 1);
        items.iter().map(|item| item.time).collect()
    };
    assert_eq!(third_batch, vec![30]);

    let (_, count) = power.get_power_control_list(10);
    assert_eq!(count, 0);

    power.reset_time();
    let reset_batch: Vec<i32> = {
        let (items, count) = power.get_power_control_list(15);
        assert_eq!(count, 1);
        items.iter().map(|item| item.time).collect()
    };
    assert_eq!(reset_batch, vec![10]);
}

#[test]
fn trajectory_segments_propagate_position_and_transition_between_segments() {
    let mut trajectory = CTrajectory::new();
    trajectory.set_init_pos_vel(KinematicInfo {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        vx: 1.0,
        vy: 0.0,
        vz: 0.0,
    });
    trajectory.set_trajectory_name("baseline");

    assert_eq!(
        trajectory.append_trajectory(
            TrajectoryType::TrajTypeConstSpeed,
            TrajectoryDataType::TrajDataTimeSpan,
            10.0,
            TrajectoryDataType::TrajDataSpeed,
            5.0,
        ),
        TRAJECTORY_NO_ERR
    );
    assert_eq!(
        trajectory.append_trajectory(
            TrajectoryType::TrajTypeConstAcc,
            TrajectoryDataType::TrajDataTimeSpan,
            2.0,
            TrajectoryDataType::TrajDataAcceleration,
            1.0,
        ),
        TRAJECTORY_NO_ERR
    );

    assert_eq!(trajectory.get_trajectory_name(), "baseline");
    approx_eq(
        trajectory.get_time_length(),
        12.0,
        1e-12,
        "trajectory length",
    );

    let mut pos_vel = KinematicInfo::default();
    assert!(trajectory.get_next_pos_vel_ecef(2.0, &mut pos_vel));
    approx_eq(pos_vel.x, 10.0, 1e-12, "const-speed x at 2s");
    approx_eq(pos_vel.vx, 5.0, 1e-12, "const-speed vx at 2s");

    assert!(trajectory.get_next_pos_vel_ecef(8.0, &mut pos_vel));
    approx_eq(pos_vel.x, 50.0, 1e-12, "const-speed x at 10s");

    assert!(trajectory.get_next_pos_vel_ecef(1.0, &mut pos_vel));
    approx_eq(pos_vel.x, 55.5, 1e-12, "const-acc x at 1s");
    approx_eq(pos_vel.vx, 6.0, 1e-12, "const-acc vx at 1s");

    assert!(trajectory.get_next_pos_vel_ecef(1.0, &mut pos_vel));
    approx_eq(pos_vel.x, 62.0, 1e-12, "const-acc x at 2s");
    approx_eq(pos_vel.vx, 7.0, 1e-12, "const-acc vx at 2s");

    assert!(!trajectory.get_next_pos_vel_ecef(0.1, &mut pos_vel));
}
