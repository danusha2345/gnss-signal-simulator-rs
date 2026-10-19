//! Выбор GPS/BDS эфемерид = набор, который спутник реально передавал (RINEX TTOM).
use gnss_rust::gnsstime::{utc_to_bds_time, utc_to_gps_time};
use gnss_rust::ifdatagen::select_broadcast_ephemeris;
use gnss_rust::json_interpreter::{read_nav_file, CNavData};
use gnss_rust::types::{BeiDouEphemeris, GpsEphemeris, UtcTime};

const RINEX: &str = "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx";

fn utc(hour: i32, minute: i32, second: f64) -> UtcTime {
    UtcTime {
        Year: 2025,
        Month: 6,
        Day: 5,
        Hour: hour,
        Minute: minute,
        Second: second,
    }
}

fn gps_at(nav: &CNavData, t: UtcTime, svid: u8) -> GpsEphemeris {
    let g = utc_to_gps_time(t, true);
    select_broadcast_ephemeris(
        &nav.gps_ephemeris,
        g.Week,
        (g.MilliSeconds / 1000) as i64,
        svid,
    )
    .expect("GPS ephemeris")
}

fn bds_at(nav: &CNavData, t: UtcTime, svid: u8) -> GpsEphemeris {
    let b = utc_to_bds_time(t);
    let records: Vec<GpsEphemeris> = nav
        .beidou_ephemeris
        .iter()
        .map(BeiDouEphemeris::to_gps_ephemeris)
        .collect();
    select_broadcast_ephemeris(&records, b.Week, (b.MilliSeconds / 1000) as i64, svid)
        .expect("BDS ephemeris")
}

#[test]
fn synthetic_gps_upload_switches_at_transmission_time() {
    let base = GpsEphemeris {
        svid: 5,
        valid: 1,
        week: 2369,
        ..Default::default()
    };
    // toe 10:00 передаётся с 08:00:18, toe 12:00 — с 10:00:18 (как в IGS BRDC).
    let old = GpsEphemeris {
        toe: 381_600,
        ttom: Some(374_418),
        iode: 57,
        ..base
    };
    let new = GpsEphemeris {
        toe: 388_800,
        ttom: Some(381_618),
        iode: 60,
        ..base
    };
    let recs = [old, new];
    let pick = |sow| {
        select_broadcast_ephemeris(&recs, 2369, sow, 5)
            .unwrap()
            .iode
    };
    assert_eq!(pick(381_610), 57, "до выгрузки в эфире ещё toe 10:00");
    assert_eq!(pick(381_948), 60, "10:05:48: в эфире уже toe 12:00");

    // Без TTOM — прежний ближайший toe.
    let bare = [
        GpsEphemeris { ttom: None, ..old },
        GpsEphemeris { ttom: None, ..new },
    ];
    let iode = select_broadcast_ephemeris(&bare, 2369, 381_948, 5)
        .unwrap()
        .iode;
    assert_eq!(iode, 57);
}

#[test]
fn rinex_ttom_is_parsed() {
    let mut nav = CNavData::new();
    read_nav_file(&mut nav, RINEX);
    let g05 = nav
        .gps_ephemeris
        .iter()
        .find(|e| e.svid == 5 && e.toe == 388_800)
        .expect("G05 toe 12:00");
    assert_eq!(g05.ttom, Some(381_618));
}

#[test]
fn t10_scenario_transmits_on_air_gps_and_bds_sets() {
    let mut nav = CNavData::new();
    read_nav_file(&mut nav, RINEX);

    // 10:05:30 UTC = 10:05:48 GPST: набор toe 10:00 сошёл с эфира в 10:00:18.
    let t10 = utc(10, 5, 30.0);
    for svid in [1u8, 2, 5, 7, 8, 14, 15, 17, 22, 30] {
        let e = gps_at(&nav, t10, svid);
        assert_eq!(e.toe, 388_800, "G{svid:02}: ожидался toe 12:00");
        let tx = e.ttom.expect("TTOM");
        assert!(tx <= 381_948, "G{svid:02}: TTOM {tx} в будущем");
    }
    // G13: внеплановая выгрузка toe 11:59:44 передаётся с 10:00:06.
    assert_eq!(gps_at(&nav, t10, 13).toe, 388_784);

    // 03:00:00 UTC (рабочий на железе сценарий): набор не меняется — toe 04:00.
    let t03 = utc(3, 0, 0.0);
    for svid in [1u8, 2, 3, 25, 31, 32] {
        assert_eq!(gps_at(&nav, t03, svid).toe, 360_000, "G{svid:02}");
    }

    // BDS передаёт набор с момента toe: на 10:05 — toe 10:00, не 11:00.
    for svid in [19u8, 27, 46] {
        assert_eq!(bds_at(&nav, t10, svid).toe, 381_600, "C{svid:02}");
    }
    // 10:35: ближайший toe был бы 11:00 (в будущем), в эфире — 10:00.
    assert_eq!(bds_at(&nav, utc(10, 35, 0.0), 27).toe, 381_600);
}
