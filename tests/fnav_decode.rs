//! Независимые ожидаемые поля — Galileo OS SIS ICD 4.2, GNSS-SDR Galileo_FNAV.h.
use gnss_rust::fnavbit::FNavBit;
use gnss_rust::types::*;

#[test]
fn fnav_page_has_exactly_500_symbols() {
    let nav = FNavBit::new();
    let mut symbols = [-1; 600];
    assert_eq!(
        nav.get_frame_data(
            GnssTime {
                Week: 2369,
                MilliSeconds: 381950000,
                SubMilliSeconds: 0.0
            },
            1,
            0,
            &mut symbols
        ),
        0
    );
    assert!(symbols[..500].iter().all(|&b| b == 0 || b == 1));
    assert_eq!(field(&decode(&symbols[..500]), 1, 6), 63);
    assert!(
        symbols[500..].iter().all(|&b| b == -1),
        "F/NAV must not emit symbols after 500"
    );
}

// Декодер использует обратное направление регистра и стандартные octal 171/133.
fn decode(symbols: &[i32]) -> Vec<u8> {
    assert_eq!(&symbols[..12], &[1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0]);
    let mut coded = [0u8; 488];
    for i in 0..488 {
        coded[(i % 61) * 8 + i / 61] = symbols[12 + i] as u8;
    }
    let mut metric = [10000u32; 64];
    metric[0] = 0;
    let mut previous = vec![[0u8; 64]; 244];
    for t in 0..244 {
        let mut next = [10000u32; 64];
        for (state, &cost) in metric.iter().enumerate() {
            for input in 0..2 {
                let register = (input << 6) | state;
                let target = register >> 1;
                let a = ((register & 0o171).count_ones() & 1) as u8;
                let b = (((register & 0o133).count_ones() & 1) ^ 1) as u8;
                let distance = cost + (a ^ coded[2 * t]) as u32 + (b ^ coded[2 * t + 1]) as u32;
                if distance < next[target] {
                    next[target] = distance;
                    previous[t][target] = state as u8;
                }
            }
        }
        metric = next;
    }
    assert!(metric[0] < 10, "FEC distance {}", metric[0]);
    let mut state = 0usize;
    let mut bits = vec![0u8; 244];
    for t in (0..244).rev() {
        bits[t] = (state >> 5) as u8;
        state = previous[t][state] as usize;
    }
    assert_eq!(&bits[238..], &[0; 6]);
    assert!(crc_matches(&bits));
    bits
}
fn crc_matches(bits: &[u8]) -> bool {
    // Прямое полиномиальное деление D*x^24; без production helper и CRC-таблицы.
    let mut dividend = bits[..214].to_vec();
    dividend.resize(238, 0);
    for i in 0..214 {
        if dividend[i] != 0 {
            for j in 0..=24 {
                dividend[i + j] ^= ((0x1864cfbu32 >> (24 - j)) & 1) as u8;
            }
        }
    }
    dividend[214..238] == bits[214..238]
}
fn field(b: &[u8], start: usize, width: usize) -> u32 {
    b[start - 1..start - 1 + width]
        .iter()
        .fold(0, |v, &bit| (v << 1) | bit as u32)
}
fn signed(b: &[u8], start: usize, width: usize) -> i32 {
    let x = field(b, start, width);
    ((x << (32 - width)) as i32) >> (32 - width)
}
fn fixture() -> FNavBit {
    let mut n = FNavBit::new();
    let e = GpsEphemeris {
        valid: 1,
        svid: 7,
        iodc: 683,
        ura: 107,
        health: (2 << 4) | (1 << 3) | 7,
        toc: 345660,
        toe: 345600,
        af0: -12345.0 * 2f64.powi(-34),
        af1: 23456.0 * 2f64.powi(-46),
        af2: -7.0 * 2f64.powi(-59),
        tgd: -9.0 * 2f64.powi(-32),
        M0: -std::f64::consts::PI / 4.0,
        omega_dot: -35.0 * std::f64::consts::PI * 2f64.powi(-43),
        ecc: 8589935.0 * 2f64.powi(-33),
        sqrtA: 2852000000.0 * 2f64.powi(-19),
        omega0: std::f64::consts::PI / 2.0,
        idot: 123.0 * std::f64::consts::PI * 2f64.powi(-43),
        i0: std::f64::consts::PI / 4.0,
        w: -std::f64::consts::PI / 2.0,
        delta_n: -211.0 * std::f64::consts::PI * 2f64.powi(-43),
        cuc: -100.0 * 2f64.powi(-29),
        cus: 200.0 * 2f64.powi(-29),
        crc: 300.0 / 32.0,
        crs: -400.0 / 32.0,
        cic: 7.0 * 2f64.powi(-29),
        cis: -8.0 * 2f64.powi(-29),
        ..GpsEphemeris::default()
    };
    n.set_ephemeris(7, &e);
    let iono = IonoNequick {
        ai0: 140.5,
        ai1: -0.25,
        ai2: 0.001953125,
        flag: 18,
    };
    let utc = UtcParam {
        A0: -3.0 * 2f64.powi(-30),
        A1: 5.0 * 2f64.powi(-50),
        TLS: 18,
        TLSF: 19,
        tot: 72,
        WN: 1345,
        WNLSF: 1350,
        DN: 6,
        ..UtcParam::default()
    };
    let mut routed = gnss_rust::nav_data::NavData::FNav(n);
    routed.set_galileo_iono_utc(Some(&iono), Some(&utc));
    let gnss_rust::nav_data::NavData::FNav(n) = routed else {
        unreachable!()
    };
    n
}
fn page(n: &FNavBit, ms: i32) -> Vec<u8> {
    let mut symbols = [0; 500];
    assert_eq!(
        n.get_frame_data(
            GnssTime {
                Week: 2369,
                MilliSeconds: ms,
                SubMilliSeconds: 0.0
            },
            7,
            0,
            &mut symbols
        ),
        0
    );
    decode(&symbols)
}

#[test]
fn fnav_fec_crc_and_all_ephemeris_utc_fields_match_icd() {
    let n = fixture();
    let p = page(&n, 1234);
    assert_eq!(field(&p, 1, 6), 1);
    for (s, w, v) in [
        (7, 6, 7),
        (13, 10, 683),
        (23, 14, 5761),
        (95, 8, 107),
        (103, 11, 562),
        (139, 5, 18),
        (154, 2, 2),
        (156, 12, 1345),
        (168, 20, 0),
        (188, 1, 1),
    ] {
        assert_eq!(field(&p, s, w), v, "page1 bit{s}");
    }
    for (s, w, v) in [
        (37, 31, -12345),
        (68, 21, 23456),
        (89, 6, -7),
        (114, 11, -64),
        (125, 14, 64),
        (144, 10, -9),
    ] {
        assert_eq!(signed(&p, s, w), v);
    }
    assert!(p[188..214].iter().all(|&b| b == 0));
    let p = page(&n, 10000);
    assert_eq!(field(&p, 1, 6), 2);
    for (s, w, v) in [
        (7, 10, 683),
        (73, 32, 8589935),
        (105, 32, 2852000000),
        (183, 12, 1345),
        (195, 20, 10),
    ] {
        assert_eq!(field(&p, s, w), v);
    }
    for (s, w, v) in [
        (17, 32, -536870912),
        (49, 24, -35),
        (137, 32, 1073741824),
        (169, 14, 123),
    ] {
        assert_eq!(signed(&p, s, w), v);
    }
    let p = page(&n, 20000);
    assert_eq!(field(&p, 1, 6), 3);
    for (s, w, v) in [
        (7, 10, 683),
        (161, 14, 5760),
        (175, 12, 1345),
        (187, 20, 20),
    ] {
        assert_eq!(field(&p, s, w), v);
    }
    for (s, w, v) in [
        (17, 32, 536870912),
        (49, 32, -1073741824),
        (81, 16, -211),
        (97, 16, -100),
        (113, 16, 200),
        (129, 16, 300),
        (145, 16, -400),
    ] {
        assert_eq!(signed(&p, s, w), v);
    }
    assert!(p[206..214].iter().all(|&b| b == 0));
    let p = page(&n, 30000);
    assert_eq!(field(&p, 1, 6), 4);
    for (s, w, v) in [
        (7, 10, 683),
        (105, 8, 18),
        (113, 8, 72),
        (121, 8, 65),
        (129, 8, 70),
        (137, 3, 6),
        (140, 8, 19),
        (148, 8, 72),
        (184, 6, 1),
        (190, 20, 30),
    ] {
        assert_eq!(field(&p, s, w), v, "page4 bit{s}");
    }
    for (s, w, v) in [
        (17, 16, 7),
        (33, 16, -8),
        (49, 32, -3),
        (81, 24, 5),
        (156, 16, 0),
        (172, 12, 0),
    ] {
        assert_eq!(signed(&p, s, w), v);
    }
    assert!(p[209..214].iter().all(|&b| b == 0));
    let mut bad = p.clone();
    bad[80] ^= 1;
    assert!(!crc_matches(&bad));
}

#[test]
fn fnav_week_page_alignment_and_invalid_arguments() {
    let n = fixture();
    let p = page(&n, 604801234);
    assert_eq!(field(&p, 156, 12), 1346);
    assert_eq!(field(&p, 168, 20), 0);
    let p = page(&n, -100000);
    assert_eq!(field(&p, 156, 12), 1344);
    assert_eq!(field(&p, 168, 20), 604700);
    assert_eq!(page(&n, 10000), page(&n, 19999));
    let mut out = [-1; 500];
    let t = GnssTime::default();
    for svid in [0, -1, 37] {
        assert_eq!(n.get_frame_data(t, svid, 0, &mut out), -1);
    }
    assert_eq!(n.get_frame_data(t, 7, 0, &mut out[..499]), -1);
    assert!(out.iter().all(|&x| x == -1));
}

#[test]
fn fnav_almanac_pages_and_split_signed_field_decode() {
    let mut n = fixture();
    let almanac: Vec<_> = [3, 12, 19]
        .into_iter()
        .enumerate()
        .map(|(i, svid)| GpsAlmanac {
            valid: 1,
            svid,
            week: 2369,
            toa: 360000,
            health: ((i + 1) << 4) as u8,
            sqrtA: 5440.588203494177 + 1.25,
            ecc: 328.0 * 2f64.powi(-16),
            w: -std::f64::consts::PI / 2.0,
            i0: 0.9773843811168246 + 42.0 * std::f64::consts::PI * 2f64.powi(-14),
            omega0: -0.75 * std::f64::consts::PI,
            omega_dot: -7.0 * std::f64::consts::PI * 2f64.powi(-33),
            M0: std::f64::consts::PI / 4.0,
            af0: -16.0 * 2f64.powi(-19),
            af1: 17.0 * 2f64.powi(-38),
            ..GpsAlmanac::default()
        })
        .collect();
    n.set_almanac(&almanac);
    let p5 = page(&n, 40000);
    let p6 = page(&n, 90000);
    for (s, w, v) in [
        (1, 6, 5),
        (7, 4, 4),
        (11, 2, 1),
        (13, 10, 600),
        (23, 6, 3),
        (29, 13, 640),
        (42, 11, 328),
        (152, 2, 1),
        (154, 6, 12),
        (160, 13, 640),
        (173, 11, 328),
    ] {
        assert_eq!(field(&p5, s, w), v);
    }
    for (s, w, v) in [
        (53, 16, -16384),
        (69, 11, 42),
        (80, 16, -24576),
        (96, 11, -7),
        (107, 16, 8192),
        (123, 16, -16),
        (139, 13, 17),
        (184, 16, -16384),
        (200, 11, 42),
    ] {
        assert_eq!(signed(&p5, s, w), v);
    }
    assert_eq!(
        ((field(&p5, 211, 4) << 12) | field(&p6, 11, 12)) as u16 as i16,
        -24576
    );
    for (s, w, v) in [
        (1, 6, 6),
        (7, 4, 4),
        (79, 2, 2),
        (81, 6, 19),
        (87, 13, 640),
        (100, 11, 328),
        (210, 2, 3),
    ] {
        assert_eq!(field(&p6, s, w), v);
    }
    for (s, w, v) in [
        (23, 11, -7),
        (34, 16, 8192),
        (50, 16, -16),
        (66, 13, 17),
        (111, 16, -16384),
        (127, 11, 42),
        (138, 16, -24576),
        (154, 11, -7),
        (165, 16, 8192),
        (181, 16, -16),
        (197, 13, 17),
    ] {
        assert_eq!(signed(&p6, s, w), v);
    }
    for ms in (0..1200000).step_by(10000) {
        page(&n, ms);
    }
    n.set_almanac(&[]);
    assert_eq!(field(&page(&n, 40000), 23, 6), 0);
}

#[test]
fn fnav_decoding_corrects_an_isolated_channel_error() {
    let n = fixture();
    let mut symbols = [0; 500];
    n.get_frame_data(
        GnssTime {
            Week: 2369,
            MilliSeconds: 10000,
            SubMilliSeconds: 0.0,
        },
        7,
        0,
        &mut symbols,
    );
    let expected = decode(&symbols);
    symbols[123] ^= 1;
    assert_eq!(decode(&symbols), expected);
}

#[test]
fn fnav_matches_independent_fixed_transmission_vector() {
    let nav = fixture();
    let mut symbols = [0i32; 500];
    nav.get_frame_data(
        GnssTime {
            Week: 2369,
            MilliSeconds: 0,
            SubMilliSeconds: 0.0,
        },
        7,
        0,
        &mut symbols,
    );
    let expected = include_str!("fixtures/fnav_page1_symbols.txt").trim();
    assert_eq!(expected.len(), 500);
    for (i, b) in expected.bytes().enumerate() {
        assert_eq!(symbols[i], (b - b'0') as i32, "symbol {i}");
    }
}
