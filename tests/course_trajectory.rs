use gnss_rust::coordinate::{calc_conv_matrix_lla, lla_to_ecef, speed_ecef_to_local};
use gnss_rust::types::{KinematicInfo, LocalSpeed};
use gnss_rust::IFDataGen;

const PRESET: &str = "presets/gps_gal_circle100m_10kmh_4m2_300s.json";

fn load(segments: serde_json::Value) -> Result<IFDataGen, String> {
    let dir = std::path::PathBuf::from(format!(".scratch/course_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let mut config: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(PRESET).unwrap()).unwrap();
    config["trajectory"]["initVelocity"] = serde_json::json!({
        "type": "SCU", "speedUnit": "m/s", "speed": 10.0, "course": 90.0, "up": 0.0
    });
    config["trajectory"]["trajectoryList"] = segments;
    let path = dir.join(format!("{:x}.json", config_hash(&config)));
    std::fs::write(&path, serde_json::to_vec(&config).unwrap()).unwrap();
    let mut generator = IFDataGen::new();
    let result = generator.load_config(path.to_str().unwrap());
    std::fs::remove_file(&path).unwrap();
    result.map(|_| generator).map_err(|e| e.to_string())
}

fn config_hash(config: &serde_json::Value) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    config.to_string().hash(&mut hasher);
    hasher.finish()
}

/// Смещение и скорость в ENU относительно начальной точки.
fn enu(generator: &IFDataGen, pos: &KinematicInfo) -> (LocalSpeed, LocalSpeed) {
    let matrix = calc_conv_matrix_lla(&generator.start_pos);
    let origin = lla_to_ecef(&generator.start_pos);
    let delta = KinematicInfo {
        vx: pos.x - origin.x,
        vy: pos.y - origin.y,
        vz: pos.z - origin.z,
        ..Default::default()
    };
    let (mut d, mut v) = (LocalSpeed::default(), LocalSpeed::default());
    speed_ecef_to_local(&matrix, &delta, &mut d);
    speed_ecef_to_local(&matrix, pos, &mut v);
    (d, v)
}

#[test]
fn const_segments_with_course_draw_a_polyline() {
    let mut generator = load(serde_json::json!([
        {"type": "Const", "time": 10.0, "course": 0.0},
        {"type": "Const", "time": 10.0, "course": 90.0},
        {"type": "Const", "time": 10.0, "course": 180.0},
        {"type": "Const", "time": 5.0}
    ]))
    .unwrap();
    assert_eq!(generator.trajectory.get_time_length(), 35.0);
    // Конечные точки ломаной: север 100 м, затем восток 100 м, затем на юг к базе;
    // сегмент без course продолжает последний курс (юг).
    for (step, east, north, ve, vn) in [
        (10.0, 0.0, 100.0, 0.0, 10.0),
        (10.0, 100.0, 100.0, 10.0, 0.0),
        (10.0, 100.0, 0.0, 0.0, -10.0),
        (5.0, 100.0, -50.0, 0.0, -10.0),
    ] {
        let mut pos = KinematicInfo::default();
        assert!(generator.trajectory.get_next_pos_vel_ecef(step, &mut pos));
        let (d, v) = enu(&generator, &pos);
        // Прямые в ECEF на ~150 м от старта отходят от плоскости ENU стартовой точки на мм.
        assert!((d.ve - east).abs() < 1e-2, "east={} want {east}", d.ve);
        assert!((d.vn - north).abs() < 1e-2, "north={} want {north}", d.vn);
        assert!(d.vu.abs() < 1e-2, "up={}", d.vu);
        assert!((v.ve - ve).abs() < 1e-3 && (v.vn - vn).abs() < 1e-3);
    }
}

#[test]
fn const_without_course_keeps_initial_course() {
    let mut generator = load(serde_json::json!([{"type": "Const", "time": 10.0}])).unwrap();
    let mut pos = KinematicInfo::default();
    assert!(generator.trajectory.get_next_pos_vel_ecef(10.0, &mut pos));
    let (d, _) = enu(&generator, &pos);
    assert!((d.ve - 100.0).abs() < 1e-2 && d.vn.abs() < 1e-2);
}

#[test]
fn const_course_rejects_non_numeric_value() {
    let error = load(serde_json::json!([{"type": "Const", "time": 10.0, "course": "north"}]))
        .err()
        .unwrap();
    assert!(error.contains("course"), "{error}");
}

#[test]
fn negative_radius_turns_right_and_const_acc_reverses_smoothly() {
    // 10 м/с на восток: правая четверть окружности R=20 м (на юг), затем торможение
    // 5 м/с² 4 с — точка разворота в 10 м южнее, обратно на север с той же скоростью.
    let quarter = 20.0 * std::f64::consts::FRAC_PI_2 / 10.0;
    let mut generator = load(serde_json::json!([
        {"type": "Circular", "time": (quarter * 1000.0).round() / 1000.0, "radius": -20.0},
        {"type": "ConstAcc", "time": 4.0, "acceleration": -5.0}
    ]))
    .unwrap();
    let mut pos = KinematicInfo::default();
    assert!(generator
        .trajectory
        .get_next_pos_vel_ecef(quarter, &mut pos));
    let (d, v) = enu(&generator, &pos);
    assert!((d.ve - 20.0).abs() < 1e-2 && (d.vn + 20.0).abs() < 1e-2);
    assert!(v.ve.abs() < 1e-2 && (v.vn + 10.0).abs() < 1e-3);
    for (step, north, vn) in [(2.0, -30.0, 0.0), (2.0, -20.0, 10.0)] {
        assert!(generator.trajectory.get_next_pos_vel_ecef(step, &mut pos));
        let (d, v) = enu(&generator, &pos);
        assert!((d.vn - north).abs() < 2e-2, "north={} want {north}", d.vn);
        assert!((v.vn - vn).abs() < 1e-2, "vn={} want {vn}", v.vn);
    }
}

#[test]
fn zero_radius_is_rejected() {
    assert!(load(serde_json::json!([{"type": "Circular", "time": 1.0, "radius": 0.0}])).is_err());
}

#[test]
fn skylab_preset_loads_and_returns_to_start() {
    // Слово пишется вперёд и обратно по тому же следу: конец совпадает с началом.
    let mut generator = IFDataGen::new();
    generator
        .load_config("presets/skylab_spb_t03.json")
        .unwrap();
    assert_eq!(generator.output_param.Interval, 300_000);
    let (mut start, mut end) = (KinematicInfo::default(), KinematicInfo::default());
    assert!(generator.trajectory.get_next_pos_vel_ecef(0.0, &mut start));
    assert!(generator.trajectory.get_next_pos_vel_ecef(300.0, &mut end));
    let (d, v) = enu(&generator, &end);
    assert!(
        d.ve.hypot(d.vn) < 2.0 && d.vu.abs() < 1.0,
        "end offset {d:?}"
    );
    assert!((v.ve.hypot(v.vn) - 12.5).abs() < 0.1);
}

#[test]
fn const_acc_with_course_starts_from_rest_in_given_direction() {
    // Острый угол: торможение 5 м/с² до нуля через 10 м на восток, разгон на север.
    let mut generator = load(serde_json::json!([
        {"type": "ConstAcc", "time": 2.0, "acceleration": -5.0},
        {"type": "ConstAcc", "time": 2.0, "acceleration": 5.0, "course": 0.0},
        {"type": "Const", "time": 1.0, "course": 0.0}
    ]))
    .unwrap();
    for (step, east, north, ve, vn) in [
        (2.0, 10.0, 0.0, 0.0, 0.0),
        (2.0, 10.0, 10.0, 0.0, 10.0),
        (1.0, 10.0, 20.0, 0.0, 10.0),
    ] {
        let mut pos = KinematicInfo::default();
        assert!(generator.trajectory.get_next_pos_vel_ecef(step, &mut pos));
        let (d, v) = enu(&generator, &pos);
        assert!(
            (d.ve - east).abs() < 1e-2 && (d.vn - north).abs() < 1e-2,
            "{d:?}"
        );
        assert!(
            (v.ve - ve).abs() < 1e-3 && (v.vn - vn).abs() < 1e-3,
            "{v:?}"
        );
    }
}
