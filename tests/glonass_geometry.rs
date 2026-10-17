//! Геометрия GLONASS проверяется независимо от исходного ECEF LOS.
use gnss_rust::{coordinate::*, satellite_param::*, types::*};
#[test]
fn glonass_atmosphere_uses_local_elevation_and_azimuth() {
    let mut nav = gnss_rust::json_interpreter::CNavData::new();
    gnss_rust::json_interpreter::read_nav_file_limited(
        &mut nav,
        "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx",
        100000,
    );
    let lla = LlaPosition {
        lat: 48.4928_f64.to_radians(),
        lon: (-114.2847_f64).to_radians(),
        alt: 100.0,
    };
    let receiver = lla_to_ecef(&lla);
    let t = gnss_rust::gnsstime::utc_to_glonass_time_corrected(UtcTime {
        Year: 2025,
        Month: 6,
        Day: 5,
        Hour: 10,
        Minute: 5,
        Second: 30.0,
    });
    let mut count = 0;
    for e in nav
        .glonass_ephemeris
        .iter()
        .filter(|e| e.day as i32 == t.Day && ((t.MilliSeconds / 1000) - e.tb as i32).abs() < 900)
        .take(24)
    {
        let mut p = SatelliteParam::default();
        get_glonass_satellite_param(&receiver, &lla, &t, e, &IonoParam::default(), &mut p);
        let sat = KinematicInfo {
            x: receiver.x + p.LosVector[0] * 2e7,
            y: receiver.y + p.LosVector[1] * 2e7,
            z: receiver.z + p.LosVector[2] * 2e7,
            ..Default::default()
        };
        let mut el = 0.0;
        let mut az = 0.0;
        sat_el_az_from_positions(&receiver, &sat, &mut el, &mut az);
        assert!(
            (el - p.Elevation).abs() < 1e-10,
            "SV{}: ENU={el} actual={}",
            e.n,
            p.Elevation
        );
        assert!((az - p.Azimuth).abs() < 1e-10);
        count += 1;
    }
    assert!(count > 5);
}
#[test]
fn cardinal_azimuth_is_clockwise_from_north() {
    let lla = LlaPosition {
        lat: 0.0,
        lon: 0.0,
        alt: 0.0,
    };
    for (los, azimuth) in [
        ([0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2),
        ([0.0, -1.0, 0.0], 3.0 * std::f64::consts::FRAC_PI_2),
        ([0.0, 0.0, 1.0], 0.0),
    ] {
        let mut el = 0.0;
        let mut az = 0.0;
        sat_el_az_from_lla(&lla, &los, &mut el, &mut az);
        assert!((az - azimuth).abs() < 1e-12);
        assert!(el.abs() < 1e-12);
    }
}
#[test]
fn glonass_internal_time_maps_to_gpst_including_day_boundaries() {
    for (year, month, day, hour) in [
        (2025, 6, 5, 10),
        (2025, 12, 31, 23),
        (2023, 12, 31, 22),
        (1996, 1, 1, 0),
    ] {
        let utc = UtcTime {
            Year: year,
            Month: month,
            Day: day,
            Hour: hour,
            Minute: 30,
            Second: 0.0009999999,
        };
        let expected = gnss_rust::gnsstime::utc_to_gps_time(utc, true);
        let actual = gnss_rust::gnsstime::glonass_corrected_to_gps(
            gnss_rust::gnsstime::utc_to_glonass_time_corrected(utc),
        );
        assert_eq!(actual.Week, expected.Week);
        assert_eq!(actual.MilliSeconds, expected.MilliSeconds);
        assert!((actual.SubMilliSeconds - expected.SubMilliSeconds).abs() < 1e-6);
    }
}
#[test]
fn glonass_ionosphere_uses_actual_frequency_slot() {
    for slot in [-7, 0, 6] {
        for (sig, frequency) in [
            (gnss_rust::SIGNAL_INDEX_G1, 1602e6 + slot as f64 * 562500.0),
            (gnss_rust::SIGNAL_INDEX_G2, 1246e6 + slot as f64 * 437500.0),
        ] {
            let actual = get_iono_delay_with_slot(10.0, GnssSystem::GlonassSystem, sig, slot);
            assert!((actual - 10.0 * (1575.42e6 / frequency).powi(2)).abs() < 1e-12);
        }
    }
}
