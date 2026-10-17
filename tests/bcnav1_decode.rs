//! Независимые битовые проверки BCH заголовка и CRC B-CNAV1.
use gnss_rust::{bcnav1bit::BCNav1Bit, bcnavbit::BCNavBit, types::*};
fn lfsr(value: u32, width: u32, tap: u32, n: usize) -> Vec<i32> {
    let mut r = value.reverse_bits() >> (32 - width);
    (0..n)
        .map(|_| {
            let bit = (r & 1) as i32;
            r = ((r & tap).count_ones() % 2) << (width - 1) | (r >> 1);
            bit
        })
        .collect()
}
fn crc(bits: &[i32]) -> u32 {
    let mut r = 0;
    for &b in bits {
        let top = (r >> 23) ^ b as u32;
        r = (r << 1) & 0xffffff;
        if top != 0 {
            r ^= 0x864cfb;
        }
    }
    r
}
fn uint(bits: &[i32]) -> u32 {
    bits.iter().fold(0, |v, &b| (v << 1) | b as u32)
}
fn payload(bits: &[i32]) -> (Vec<i32>, Vec<i32>) {
    let rows: Vec<_> = (0..36)
        .map(|row| {
            (0..48)
                .map(|col| bits[72 + col * 36 + row])
                .collect::<Vec<_>>()
        })
        .collect();
    let mut a = Vec::new();
    let mut b = Vec::new();
    for group in 0..11 {
        a.extend(&rows[3 * group]);
        a.extend(&rows[3 * group + 1]);
        b.extend(&rows[3 * group + 2]);
    }
    for row in &rows[33..36] {
        a.extend(row);
    }
    (a, b)
}
#[test]
fn b1c_bch_svid_and_soh_match_independent_lfsr() {
    let mut nav = BCNav1Bit::new();
    let mut bits = [0; 1800];
    for sv in 1..=63 {
        for soh in [0, 1, 99, 199] {
            assert_eq!(
                nav.get_frame_data(
                    GnssTime {
                        Week: 2369,
                        MilliSeconds: soh * 18000,
                        SubMilliSeconds: 0.0
                    },
                    sv,
                    0,
                    &mut bits
                ),
                0
            );
            assert_eq!(&bits[..21], lfsr(sv as u32, 6, 0x17, 21));
            assert_eq!(&bits[21..72], lfsr(soh as u32, 8, 0x9f, 51));
        }
    }
}
#[test]
fn bcnv_crc_matches_bitwise_polynomial_division() {
    let n = BCNavBit::new();
    let mut words = [0x123456, 0xabcdef, 0x090807, 0];
    n.append_crc(&mut words, 4);
    let bits: Vec<i32> = words[..3]
        .iter()
        .flat_map(|w| (0..24).rev().map(move |i| ((w >> i) & 1) as i32))
        .collect();
    assert_eq!(words[3], crc(&bits));
}
#[test]
fn b1c_transmits_real_week_hour_and_valid_crc() {
    let mut nav = BCNav1Bit::new();
    let mut bits = [0; 1800];
    nav.get_frame_data(
        GnssTime {
            Week: 2369,
            MilliSeconds: 381936000,
            SubMilliSeconds: 0.0,
        },
        21,
        0,
        &mut bits,
    );
    let (sf2, sf3) = payload(&bits);
    assert_eq!(uint(&sf2[..13]), 1013);
    assert_eq!(uint(&sf2[13..21]), 106);
    assert_eq!(crc(&sf2[..576]), uint(&sf2[576..600]));
    assert_eq!(crc(&sf3[..240]), uint(&sf3[240..264]));
}
#[test]
fn b1c_pilot_secondary_code_uses_bdt_epoch() {
    use gnss_rust::{
        pilotbit::get_pilot_bits, satellite_signal::SatelliteSignal, ComplexNumber, GnssSystem,
        SIGNAL_INDEX_B1C,
    };
    let mut signal = SatelliteSignal::new();
    signal.set_signal_attribute(GnssSystem::BdsSystem, SIGNAL_INDEX_B1C as i32, None, 21);
    let (code, length) = get_pilot_bits(GnssSystem::BdsSystem, SIGNAL_INDEX_B1C, 21).unwrap();
    for ms in [0, 13999, 14000, 28000, 381948000] {
        let gps = GnssTime {
            Week: 2370,
            MilliSeconds: ms,
            SubMilliSeconds: 0.0,
        };
        let bdt = gps.add_milliseconds(-14000.0);
        let bit = (bdt.MilliSeconds / 10) as usize % length;
        let mut data = ComplexNumber::new();
        let mut pilot = ComplexNumber::new();
        signal.get_satellite_signal(gps, &mut data, &mut pilot);
        assert_eq!(
            pilot.real < 0.0,
            code[bit / 32] & (1 << (bit % 32)) != 0,
            "GPS ms={ms}"
        );
    }
}
#[test]
fn b1c_sf3_page_schedule_and_utc_are_not_filler() {
    let mut nav = BCNav1Bit::new();
    nav.set_iono_utc(
        None,
        Some(&UtcParam {
            A0: -64.0 * 2f64.powi(-35),
            A1: 0.0,
            WN: 1013,
            WNLSF: 1013,
            TLS: 4,
            TLSF: 4,
            DN: 7,
            tot: 84,
            flag: 3,
            ..Default::default()
        }),
    );
    let mut bits = [0; 1800];
    for soh in 0..100 {
        nav.get_frame_data(
            GnssTime {
                Week: 2369,
                MilliSeconds: soh * 18000,
                SubMilliSeconds: 0.0,
            },
            21,
            0,
            &mut bits,
        );
        let (_, sf3) = payload(&bits);
        let expected = if soh % 10 == 0 {
            1
        } else if soh % 10 == 5 {
            3
        } else if soh % 5 == 1 && soh < 80 {
            2
        } else if soh == 99 {
            0
        } else {
            4
        };
        assert_eq!(uint(&sf3[..6]), expected);
        assert_eq!(crc(&sf3[..240]), uint(&sf3[240..264]));
        if expected == 1 {
            assert_eq!(uint(&sf3[116..132]), 0xffc0);
            assert_eq!(uint(&sf3[152..160]), 4);
            assert_eq!(uint(&sf3[160..176]), 84 * 256);
            assert_eq!(uint(&sf3[176..189]), 1013);
        }
    }
}
#[test]
fn rinex_frame_matches_pocketsdr_verified_fixture() {
    use gnss_rust::json_interpreter::{read_nav_file_filtered, CNavData};
    let mut source = CNavData::new();
    read_nav_file_filtered(
        &mut source,
        "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx",
        None,
        &["BeiDou"],
    );
    let e = source
        .beidou_ephemeris
        .iter()
        .filter(|e| e.svid == 21)
        .min_by_key(|e| (e.toe - 381934).abs())
        .unwrap()
        .to_gps_ephemeris();
    let mut nav = BCNav1Bit::new();
    assert!(nav.set_ephemeris(21, &e));
    let mut bits = [0; 1800];
    nav.get_frame_data(
        GnssTime {
            Week: 2369,
            MilliSeconds: 381936000,
            SubMilliSeconds: 0.0,
        },
        21,
        0,
        &mut bits,
    );
    let actual: String = bits
        .iter()
        .map(|&b| if b == 0 { '0' } else { '1' })
        .collect();
    assert_eq!(
        actual,
        include_str!("fixtures/b1c_reference_symbols.txt").trim()
    );
}
#[test]
fn bdut_rinex_header_reaches_b1c_with_bdt_leap_seconds() {
    use gnss_rust::json_interpreter::{read_nav_file_filtered, CNavData};
    let mut source = CNavData::new();
    read_nav_file_filtered(
        &mut source,
        "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx",
        None,
        &["BeiDou"],
    );
    let u = source.bds_utc.unwrap();
    assert_eq!((u.WN, u.TLS, u.TLSF, u.WNLSF, u.tot), (1013, 4, 4, 573, 84));
    let mut nav = BCNav1Bit::new();
    nav.set_iono_utc(None, Some(&u));
    let mut bits = [0; 1800];
    nav.get_frame_data(
        GnssTime {
            Week: 2369,
            MilliSeconds: 0,
            SubMilliSeconds: 0.0,
        },
        21,
        0,
        &mut bits,
    );
    let (_, sf3) = payload(&bits);
    assert_eq!(uint(&sf3[116..132]), 0xffc0);
    assert_eq!(uint(&sf3[152..160]), 4);
}
