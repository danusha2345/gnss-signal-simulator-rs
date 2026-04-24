use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use std::path::Path;

const MIXED_RINEX: &str = "Rinex_Data/rinex_v3_20251560000.rnx";
const MERGED_RINEX: &str = "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx";

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
fn missing_nav_file_leaves_navigation_data_empty() {
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, "Rinex_Data/does_not_exist.rnx", 3);

    assert!(nav_data.gps_ephemeris.is_empty());
    assert!(nav_data.beidou_ephemeris.is_empty());
    assert!(nav_data.galileo_ephemeris.is_empty());
    assert!(nav_data.glonass_ephemeris.is_empty());
    assert!(nav_data.leap_seconds.is_none());
}
