use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use std::path::Path;

const MIXED_RINEX: &str = "Rinex_Data/rinex_v3_20251560000.rnx";
const MERGED_RINEX: &str = "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx";

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {actual} to be within {tolerance} of {expected}"
    );
}

fn load_limited(path: &str, max_per_system: usize) -> CNavData {
    assert!(
        Path::new(path).exists(),
        "missing test RINEX fixture: {path}"
    );

    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, path, max_per_system);
    nav_data
}

#[test]
fn mixed_rinex_loads_all_constellation_headers() {
    let nav_data = load_limited(MIXED_RINEX, 3);

    assert_eq!(nav_data.gps_ephemeris.len(), 3);
    assert_eq!(nav_data.beidou_ephemeris.len(), 3);
    assert_eq!(nav_data.galileo_ephemeris.len(), 3);
    assert_eq!(nav_data.glonass_ephemeris.len(), 3);

    assert!(nav_data.gps_iono_alpha.is_some());
    assert!(nav_data.gps_iono_beta.is_some());
    assert_eq!(nav_data.leap_seconds, Some(18));

    let utc = nav_data.utc_param.expect("GPUT header should be parsed");
    assert_eq!(utc.TLS, 18);
    assert_ne!(utc.flag & 0b10, 0, "LEAP SECONDS flag should be set");
}

#[test]
fn mixed_rinex_header_values_match_fixture_fields() {
    let nav_data = load_limited(MIXED_RINEX, 1);

    let alpha = nav_data
        .gps_iono_alpha
        .expect("GPSA IONOSPHERIC CORR should be parsed");
    assert_close(alpha[0], 1.5832e-08, 1.0e-14);
    assert_close(alpha[1], 2.2352e-08, 1.0e-14);
    assert_close(alpha[2], -1.1921e-07, 1.0e-13);
    assert_close(alpha[3], -1.1921e-07, 1.0e-13);

    let beta = nav_data
        .gps_iono_beta
        .expect("GPSB IONOSPHERIC CORR should be parsed");
    assert_close(beta[0], 1.1264e+05, 1.0e-3);
    assert_close(beta[1], 1.4746e+05, 1.0e-3);
    assert_close(beta[2], -1.3107e+05, 1.0e-3);
    assert_close(beta[3], -3.9322e+05, 1.0e-3);

    let utc = nav_data.utc_param.expect("GPUT should be parsed");
    assert_close(utc.A0, -9.313_225_746_2e-10, 1.0e-18);
    assert_close(utc.A1, -8.881_784_197e-16, 1.0e-24);
    assert_eq!(utc.tot, 15, "61440 seconds should be stored in 2^12s units");
    assert_eq!(utc.WN, 2370);
    assert_eq!(utc.TLS, 18);
    assert_eq!(utc.TLSF, 18);
    assert_eq!(utc.WNLSF, 1929);
    assert_eq!(utc.DN, 7);
    assert_eq!(nav_data.leap_seconds, Some(18));
}

#[test]
fn limited_rinex_reader_applies_limits_per_constellation() {
    let nav_data = load_limited(MIXED_RINEX, 1);

    assert_eq!(nav_data.gps_ephemeris.len(), 1);
    assert_eq!(nav_data.glonass_ephemeris.len(), 1);
    assert_eq!(nav_data.beidou_ephemeris.len(), 1);
    assert_eq!(nav_data.galileo_ephemeris.len(), 1);

    assert_eq!(nav_data.gps_ephemeris[0].svid, 1);
    assert!(nav_data.glonass_ephemeris[0].slot > 0);
    assert!(nav_data.beidou_ephemeris[0].svid > 0);
    assert!(nav_data.galileo_ephemeris[0].svid > 0);
}

#[test]
fn merged_rinex_loads_glonass_records() {
    let nav_data = load_limited(MERGED_RINEX, 3);

    assert_eq!(nav_data.gps_ephemeris.len(), 3);
    assert_eq!(nav_data.glonass_ephemeris.len(), 3);
    assert_eq!(nav_data.galileo_ephemeris.len(), 3);
    assert_eq!(nav_data.leap_seconds, Some(18));

    for eph in &nav_data.glonass_ephemeris {
        assert!(eph.valid != 0);
        assert!(eph.slot > 0);
        assert!(eph.x.abs() > 1.0);
        assert!(eph.y.abs() > 1.0);
        assert!(eph.z.abs() > 1.0);
    }
}

#[test]
fn merged_rinex_header_with_many_bds_iono_lines_does_not_block_records() {
    let nav_data = load_limited(MERGED_RINEX, 1);

    assert_eq!(nav_data.gps_ephemeris.len(), 1);
    assert_eq!(nav_data.glonass_ephemeris.len(), 1);
    assert_eq!(nav_data.beidou_ephemeris.len(), 1);
    assert_eq!(nav_data.galileo_ephemeris.len(), 1);
    assert_eq!(nav_data.leap_seconds, Some(18));
}

#[test]
fn missing_nav_file_leaves_navigation_data_empty() {
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, "Rinex_Data/does_not_exist.rnx", 3);

    assert!(nav_data.gps_ephemeris.is_empty());
    assert!(nav_data.beidou_ephemeris.is_empty());
    assert!(nav_data.galileo_ephemeris.is_empty());
    assert!(nav_data.glonass_ephemeris.is_empty());
    assert!(nav_data.leap_seconds.is_none());
}
