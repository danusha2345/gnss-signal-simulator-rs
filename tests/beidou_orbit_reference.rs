//! Векторы RTKLIB eph2pos: GPST week2369/sow381948, BDT sow381934.
use gnss_rust::{json_interpreter::*, types::*};
#[test]
fn beidou_geo_and_meo_match_independent_ecef_reference() {
    let mut nav = CNavData::new();
    read_nav_file_filtered(
        &mut nav,
        "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx",
        None,
        &["BeiDou"],
    );
    let vectors = [
        (
            1,
            [-34_340_077.422_783_9, 24466321.530778196, 1099561.098835642],
        ),
        (
            6,
            [
                -16_365_418.798_750_42,
                38220761.733019315,
                -7751698.632500703,
            ],
        ),
        (
            21,
            [3709454.666727277, 18193331.215675507, 20826744.912819657],
        ),
        (
            59,
            [
                -32_259_001.102_472_32,
                27_085_355.829_062_84,
                1880060.816913033,
            ],
        ),
    ];
    for (sv, expected) in vectors {
        let mut e = nav
            .beidou_ephemeris
            .iter()
            .filter(|e| e.svid == sv)
            .min_by_key(|e| (381934 - e.toe).abs())
            .unwrap()
            .to_gps_ephemeris();
        let mut pos = KinematicInfo::default();
        gnss_rust::coordinate::gps_sat_pos_speed_eph(
            GnssSystem::BdsSystem,
            381934.0,
            &mut e,
            &mut pos,
            None,
        );
        let error = ((pos.x - expected[0]).powi(2)
            + (pos.y - expected[1]).powi(2)
            + (pos.z - expected[2]).powi(2))
        .sqrt();
        println!("C{sv:02}: {error} m");
        assert!(error < 0.02, "C{sv:02}: {error} m");
    }
}
