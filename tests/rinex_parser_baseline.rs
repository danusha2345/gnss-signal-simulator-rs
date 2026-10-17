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

/// Regression for audit H8: BeiDou broadcast-orbit 6 carries TGD1 (data[25]) and
/// TGD2 (data[26]). The shared GPS parser used to read data[26] as IODC, truncating
/// it to 0 via the f64→u16 cast and never assigning tgd2, so the B2I group delay was
/// silently lost (every BeiDou record had tgd2 == 0).
#[test]
fn beidou_group_delays_are_parsed() {
    let nav_data = load_limited(MIXED_RINEX, 10);
    assert!(
        !nav_data.beidou_ephemeris.is_empty(),
        "expected BeiDou ephemerides from the mixed fixture"
    );
    let any_tgd2 = nav_data.beidou_ephemeris.iter().any(|e| e.tgd2 != 0.0);
    assert!(
        any_tgd2,
        "all BeiDou TGD2 are zero — the B2I group delay (data[26]) was dropped"
    );
    for e in &nav_data.beidou_ephemeris {
        assert!(e.tgd2.is_finite(), "BeiDou SV{} has non-finite TGD2", e.svid);
    }
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

/// Regression for GitHub issue #1 ("Incorrect week for E1 signal") / audit H7.
///
/// Galileo broadcast-orbit lines are written by some RINEX producers with the
/// trailing whitespace trimmed, so a line whose last field ends exactly on a
/// 19-char boundary has length 61 (orbit-5) or 80. The fixed-width field guards
/// used a strict `>` (`if line.len() > 61`), which dropped the GST week field
/// (`data[21]`) for those trimmed lines, leaving `eph.week == 0`. The receiver
/// then saw the GPS week jump between the correct value and a rollover default.
#[test]
fn galileo_week_survives_trimmed_orbit_lines() {
    let nav_data = load_limited(MERGED_RINEX, 50);
    assert!(
        !nav_data.galileo_ephemeris.is_empty(),
        "expected Galileo ephemerides from the merged fixture"
    );
    // The fixture is 2025-06-04 → GST/GPS week 2369. Before the fix the trimmed
    // orbit-5 lines yielded week 0 (or a 1024 rollover default).
    for eph in &nav_data.galileo_ephemeris {
        assert!(
            eph.week > 1024,
            "Galileo SV{} parsed week={} (GST week field dropped from a trimmed line)",
            eph.svid,
            eph.week
        );
    }
}

#[test]
fn rinex_accuracy_and_orbital_constants_follow_each_constellation() {
    let nav = load_limited(MERGED_RINEX, 10000);
    let gps = nav.gps_ephemeris.iter().find(|e| e.svid == 1).unwrap();
    let gal = nav.galileo_ephemeris.iter().find(|e| e.svid == 2).unwrap();
    assert_eq!(gps.ura, 0); // RINEX 2.0 m
    assert_eq!(gal.ura, 107); // RINEX 3.12 m
    for (eph, mu) in [(gps, 3.986005e14_f64), (gal, 3.986004418e14_f64)] {
        let recovered_mu = (eph.n - eph.delta_n).powi(2) * eph.sqrtA.powi(6);
        assert!((recovered_mu / mu - 1.0).abs() < 1e-14);
    }
}
