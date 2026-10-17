use gnss_rust::*;
#[test]
fn tmboc_switches_on_exactly_four_chips_per_33() {
    use gnss_rust::sat_if_signal::l1c_pilot_subcarrier;
    for chip in 0..10230 {
        for half in 0..12 {
            let phase = 2.0 * (chip as f64 + (half as f64 + 0.5) / 12.0);
            let fast = matches!(chip % 33, 0 | 4 | 6 | 29);
            let expected = if (if fast { half % 2 } else { half / 6 }) == 0 {
                1.0
            } else {
                -1.0
            };
            assert_eq!(l1c_pilot_subcarrier(phase, true), expected);
            assert_eq!(
                l1c_pilot_subcarrier(phase, false),
                if half < 6 { 1.0 } else { -1.0 }
            );
        }
    }
}
#[test]
fn accelerated_segment_enters_circle_with_current_speed_and_radius() {
    use trajectory::{TrajectoryDataType as D, TrajectoryType as T};
    let mut t = CTrajectory::new();
    t.set_init_pos_vel_lla(
        LlaPosition {
            lat: 0.7,
            lon: 0.4,
            alt: 100.0,
        },
        LocalSpeed {
            speed: 10.0,
            course: 0.0,
            ..Default::default()
        },
        false,
    );
    assert_eq!(
        t.append_trajectory(
            T::TrajTypeConstAcc,
            D::TrajDataTimeSpan,
            10.0,
            D::TrajDataAcceleration,
            1.0
        ),
        0
    );
    assert_eq!(
        t.append_trajectory(
            T::TrajTypeHorizontalCircular,
            D::TrajDataTimeSpan,
            20.0,
            D::TrajDataRadius,
            100.0
        ),
        0
    );
    let mut a = KinematicInfo::default();
    let mut b = a;
    t.get_next_pos_vel_ecef(10.001, &mut a);
    t.get_next_pos_vel_ecef(0.001, &mut b);
    let speed = (a.vx * a.vx + a.vy * a.vy + a.vz * a.vz).sqrt();
    let acceleration =
        ((b.vx - a.vx).powi(2) + (b.vy - a.vy).powi(2) + (b.vz - a.vz).powi(2)).sqrt() / 0.001;
    assert!((speed - 20.0).abs() < 1e-5);
    assert!((speed * speed / acceleration - 100.0).abs() < 0.001);
}
#[test]
fn tmboc_spectrum_and_reference_correlation() {
    use gnss_rust::sat_if_signal::l1c_pilot_subcarrier;
    let n = 33 * 24;
    let mut tone_full = (0.0, 0.0);
    let mut tone_low = (0.0, 0.0);
    let mut correlation = 0.0;
    for i in 0..n {
        let phase = 2.0 * (i as f64 + 0.5) / 24.0;
        let full = l1c_pilot_subcarrier(phase, true);
        let low = l1c_pilot_subcarrier(phase, false);
        let angle = std::f64::consts::TAU * 6.0 * (i as f64 + 0.5) / 24.0;
        tone_full.0 += full * angle.cos();
        tone_full.1 += full * angle.sin();
        tone_low.0 += low * angle.cos();
        tone_low.1 += low * angle.sin();
        correlation += full * low;
    }
    assert!(f64::hypot(tone_full.0, tone_full.1) / n as f64 > 0.07);
    assert!(f64::hypot(tone_low.0, tone_low.1) / (n as f64) < 1e-13);
    assert!((correlation / n as f64 - 29.0 / 33.0).abs() < 1e-14);
}
