//! Независимые проверки RINEX 3.04: data sources bits 0..2, clock bits 8..9.
use gnss_rust::{constants::*, galileo, types::*};
fn eph(source: u16) -> GpsEphemeris {
    GpsEphemeris {
        svid: 1,
        valid: 1,
        week: 2369,
        toe: 381600,
        source,
        ura: 10,
        tgd: 3e-9,
        tgd2: -7e-9,
        ..Default::default()
    }
}
#[test]
fn navigation_family_health_and_record_order() {
    let t = GnssTime {
        Week: 2369,
        MilliSeconds: 381610000,
        SubMilliSeconds: 0.0,
    };
    let mut records = vec![eph(517), eph(258)];
    for _ in 0..2 {
        assert_eq!(
            galileo::select_ephemeris(&records, t, 1, SIGNAL_INDEX_E1)
                .unwrap()
                .source,
            517
        );
        assert_eq!(
            galileo::select_ephemeris(&records, t, 1, SIGNAL_INDEX_E5A)
                .unwrap()
                .source,
            258
        );
        records.reverse();
    }
    let mut invalid = eph(517);
    invalid.health = 1;
    assert!(galileo::select_ephemeris(&[invalid], t, 1, SIGNAL_INDEX_E1).is_none());
    assert!(galileo::select_ephemeris(&[eph(5)], t, 1, SIGNAL_INDEX_E1).is_none());
    assert!(galileo::select_ephemeris(
        &records,
        t.add_milliseconds(4.0 * 3600000.0),
        1,
        SIGNAL_INDEX_E1
    )
    .is_none());
}
#[test]
fn bgd_recovers_ionosphere_free_clock_reference() {
    // (f1²*d1 - f2²*d2)/(f1²-f2²)=0; d1-d2=(1-gamma)*BGD.
    for (source, low, index, bgd) in [(517, 1207.14_f64, 2, -7e-9), (258, 1176.45_f64, 1, 3e-9)] {
        let d = galileo::group_delays(&eph(source));
        assert_eq!(d[0], bgd);
        assert!((1575.42_f64.powi(2) * d[0] - low.powi(2) * d[index]).abs() < 1e-17);
        assert!((d[0] - d[index] - (1.0 - (1575.42 / low).powi(2)) * bgd).abs() < 1e-22);
        // E5 AltBOC не должен терять групповую задержку.
        assert!((1575.42_f64.powi(2) * d[0] - 1191.795_f64.powi(2) * d[3]).abs() < 1e-17);
    }
}
#[test]
fn rinex_preserves_epochs_and_full_source_bits() {
    let mut nav = gnss_rust::json_interpreter::CNavData::new();
    gnss_rust::json_interpreter::read_nav_file_limited(
        &mut nav,
        "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx",
        100000,
    );
    assert!(nav.galileo_ephemeris.iter().any(|e| e.source == 517));
    assert!(nav.galileo_ephemeris.iter().any(|e| e.source == 258));
}

#[test]
fn epoch_and_iod_selection_crosses_gps_week() {
    let old = GpsEphemeris {
        week: 2369,
        toe: 604740,
        iodc: 1,
        ..eph(517)
    };
    let new = GpsEphemeris {
        week: 2370,
        toe: 60,
        iodc: 2,
        ..eph(517)
    };
    let before = GnssTime {
        Week: 2369,
        MilliSeconds: 604750000,
        SubMilliSeconds: 0.0,
    };
    // 2370:70 — toe=60 новой записи уже наступил (будущий toe не транслируется).
    let after = before.add_milliseconds(120000.0);
    for records in [[old, new], [new, old]] {
        assert_eq!(
            galileo::select_ephemeris(&records, before, 1, SIGNAL_INDEX_E1)
                .unwrap()
                .iodc,
            1
        );
        assert_eq!(
            galileo::select_ephemeris(&records, after, 1, SIGNAL_INDEX_E1)
                .unwrap()
                .iodc,
            2
        );
        // До наступления toe новой записи остаётся старая, даже если новая ближе.
        let just_before = before.add_milliseconds(100000.0);
        assert_eq!(
            galileo::select_ephemeris(&records, just_before, 1, SIGNAL_INDEX_E1)
                .unwrap()
                .iodc,
            1
        );
    }
}
