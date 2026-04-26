use gnss_rust::bcnav1bit::BCNav1Bit;
use gnss_rust::bcnav2bit::BCNav2Bit;
use gnss_rust::bcnav3bit::BCNav3Bit;
use gnss_rust::cnav2bit::CNav2Bit;
use gnss_rust::cnavbit::CNavBit;
use gnss_rust::constants::{
    FREQ_BDS_B1C, FREQ_BDS_B1I, FREQ_BDS_B2A, FREQ_BDS_B2AB, FREQ_BDS_B2B, FREQ_BDS_B2I,
    FREQ_BDS_B3I, FREQ_GAL_E1, FREQ_GAL_E5, FREQ_GAL_E5A, FREQ_GAL_E5B, FREQ_GAL_E6, FREQ_GLO_G1,
    FREQ_GLO_G2, FREQ_GLO_G3, FREQ_GPS_L1, FREQ_GPS_L2, FREQ_GPS_L5, SIGNAL_CENTER_FREQ,
};
use gnss_rust::d1d2navbit::D1D2NavBit;
use gnss_rust::fnavbit::FNavBit;
use gnss_rust::gnavbit::GNavBit;
use gnss_rust::inavbit::INavBit;
use gnss_rust::l5cnavbit::L5CNavBit;
use gnss_rust::lnavbit::LNavBit;
use gnss_rust::types::*;

fn t(ms: i32) -> GnssTime {
    GnssTime {
        Week: 2369,
        MilliSeconds: ms,
        SubMilliSeconds: 0.0,
    }
}

fn bits_to_u32(bits: &[i32]) -> u32 {
    bits.iter().fold(0u32, |acc, &bit| (acc << 1) | bit as u32)
}

fn bits_to_symbols(bits: &[i32]) -> Vec<i32> {
    bits.chunks(6)
        .map(|chunk| chunk.iter().fold(0i32, |acc, &bit| (acc << 1) | bit))
        .collect()
}

fn assert_bits(name: &str, bits: &[i32], expected: u32) {
    assert_eq!(bits_to_u32(bits), expected, "{name}");
}

fn assert_binary(name: &str, bits: &[i32]) {
    assert!(
        bits.iter().all(|&bit| bit == 0 || bit == 1),
        "{name} contains a non-binary symbol"
    );
}

fn assert_nonzero(name: &str, bits: &[i32]) {
    assert!(bits.iter().any(|&bit| bit != 0), "{name} is all zero");
}

fn deinterleave_gps_l1c_payload(bits: &[i32]) -> [i32; 1748] {
    let mut combined = [0i32; 1748];
    for input_idx in 0..1748 {
        let output_idx = 52 + 38 * (input_idx % 46) + (input_idx / 46);
        combined[input_idx] = bits[output_idx];
    }
    combined
}

fn deinterleave_b1c_payload(bits: &[i32]) -> ([i32; 1200], [i32; 528]) {
    let mut rows = [[0i32; 48]; 36];
    for row in 0..36 {
        for col in 0..48 {
            rows[row][col] = bits[72 + row + 36 * col];
        }
    }

    let mut subframe2 = [0i32; 1200];
    let mut subframe3 = [0i32; 528];
    for group in 0..11 {
        for col in 0..48 {
            subframe2[group * 96 + col] = rows[group * 3][col];
            subframe2[group * 96 + 48 + col] = rows[group * 3 + 1][col];
            subframe3[group * 48 + col] = rows[group * 3 + 2][col];
        }
    }
    for col in 0..48 {
        subframe2[22 * 48 + col] = rows[33][col];
        subframe2[23 * 48 + col] = rows[34][col];
        subframe2[24 * 48 + col] = rows[35][col];
    }
    (subframe2, subframe3)
}

fn sample_gps_eph(svid: u8) -> GpsEphemeris {
    GpsEphemeris {
        ura: 2,
        iodc: 45,
        iode: 45,
        svid,
        source: 0,
        valid: 1,
        flag: 0b1111_1111,
        health: 0,
        toe: 345_600,
        toc: 345_600,
        top: 345_600,
        week: 2369,
        M0: 0.1,
        delta_n: 4.5e-9,
        delta_n_dot: 0.0,
        ecc: 0.01,
        sqrtA: 5153.795_890_81,
        axis_dot: 0.0,
        omega0: 1.0,
        i0: 0.94,
        w: 0.5,
        omega_dot: -8.0e-9,
        idot: 0.0,
        cuc: 1.0e-6,
        cus: -1.2e-6,
        crc: 200.0,
        crs: -100.0,
        cic: 5.0e-8,
        cis: -5.0e-8,
        af0: 1.0e-4,
        af1: -1.0e-12,
        af2: 0.0,
        tgd: -1.1e-8,
        tgd2: 2.2e-9,
        tgd_ext: [0.0, 0.0, 2.0e-9, 3.0e-9, 0.0],
        axis: 26_560_000.0,
        n: 0.0,
        root_ecc: 0.0,
        omega_t: 0.0,
        omega_delta: 0.0,
        Ek: 0.0,
        Ek_dot: 0.0,
    }
}

fn sample_alm(len: usize) -> Vec<GpsAlmanac> {
    (0..len)
        .map(|i| GpsAlmanac {
            valid: 1,
            flag: 0,
            health: 0,
            svid: (i + 1) as u8,
            toa: 1200 << 12,
            week: 2369,
            M0: 0.0,
            ecc: 0.01,
            sqrtA: 5153.795,
            omega0: 1.0,
            i0: 0.94,
            w: 0.5,
            omega_dot: -8.0e-9,
            af0: 0.0,
            af1: 0.0,
        })
        .collect()
}

fn sample_iono() -> IonoParam {
    IonoParam {
        a0: 1.2e-8,
        a1: -1.5e-8,
        a2: 0.0,
        a3: 0.0,
        b0: 1.2e5,
        b1: 9.6e4,
        b2: 7.2e4,
        b3: 4.8e4,
        flag: 1,
    }
}

fn sample_nequick() -> IonoNequick {
    IonoNequick {
        ai0: 120.0,
        ai1: 0.0,
        ai2: 0.0,
        flag: 1,
    }
}

fn sample_utc() -> UtcParam {
    UtcParam {
        A0: 1.0e-6,
        A1: -1.0e-12,
        A2: 0.0,
        WN: 2369,
        WNLSF: 2370,
        tot: 16,
        TLS: 18,
        TLSF: 18,
        DN: 4,
        flag: 3,
    }
}

fn sample_glo_eph(slot: u8) -> GlonassEphemeris {
    GlonassEphemeris {
        valid: 1,
        flag: 1,
        freq: 0,
        slot,
        P: 0,
        M: 0,
        Ft: 0,
        n: slot,
        Bn: 0,
        En: 0,
        tb: 900,
        day: 100,
        tk: 1200,
        gamma: 1e-9,
        tn: 0.0,
        dtn: 0.0,
        x: 1.0e6,
        y: -1.2e6,
        z: 2.0e6,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
        ax: 0.0,
        ay: 0.0,
        az: 0.0,
        tc: 0.0,
        PosVelT: KinematicInfo::default(),
    }
}

#[test]
fn signal_center_frequency_matrix_matches_icd_fixtures() {
    assert_eq!(
        SIGNAL_CENTER_FREQ[GnssSystem::GpsSystem as usize][0..5],
        [
            FREQ_GPS_L1,
            FREQ_GPS_L1,
            FREQ_GPS_L2,
            FREQ_GPS_L2,
            FREQ_GPS_L5
        ]
    );
    assert_eq!(
        SIGNAL_CENTER_FREQ[GnssSystem::BdsSystem as usize][0..7],
        [
            FREQ_BDS_B1C,
            FREQ_BDS_B1I,
            FREQ_BDS_B2I,
            FREQ_BDS_B3I,
            FREQ_BDS_B2A,
            FREQ_BDS_B2B,
            FREQ_BDS_B2AB,
        ]
    );
    assert_eq!(
        SIGNAL_CENTER_FREQ[GnssSystem::GalileoSystem as usize][0..5],
        [
            FREQ_GAL_E1,
            FREQ_GAL_E5A,
            FREQ_GAL_E5B,
            FREQ_GAL_E5,
            FREQ_GAL_E6
        ]
    );
    assert_eq!(
        SIGNAL_CENTER_FREQ[GnssSystem::GlonassSystem as usize][0..3],
        [FREQ_GLO_G1, FREQ_GLO_G2, FREQ_GLO_G3]
    );
}

#[test]
fn gps_nav_headers_have_bit_exact_icd_sync_fields() {
    let eph = sample_gps_eph(1);
    let alm = sample_alm(32);
    let iono = sample_iono();
    let utc = sample_utc();

    let mut lnav = LNavBit::new();
    assert_ne!(lnav.set_ephemeris(1, &eph), 0);
    let alm_arr: [GpsAlmanac; 32] = alm.clone().try_into().unwrap();
    lnav.set_almanac(&alm_arr);
    lnav.set_iono_utc(&iono, &utc);
    let mut lnav_bits = [0i32; 300];
    assert_eq!(lnav.get_frame_data(t(0), 1, 0, &mut lnav_bits), 0);
    assert_bits("GPS LNAV TLM preamble", &lnav_bits[0..8], 0x8b);
    assert_bits("GPS LNAV HOW next TOW", &lnav_bits[30..47], 1);
    assert_bits("GPS LNAV HOW subframe id", &lnav_bits[49..52], 1);

    let mut cnav = CNavBit::new();
    assert_ne!(cnav.set_ephemeris(3, &sample_gps_eph(3)), 0);
    cnav.set_almanac(&alm);
    cnav.set_iono_utc(&iono, &utc);
    assert_eq!(
        cnav.eph_message[2][0][0],
        (0x8b << 12) | (3 << 6) | 10,
        "GPS CNAV message 10 preamble/SVID/type header"
    );
    assert_eq!(
        cnav.eph_message[2][1][0],
        (0x8b << 12) | (3 << 6) | 11,
        "GPS CNAV message 11 preamble/SVID/type header"
    );
    let mut cnav_bits = vec![0i32; 600];
    assert_eq!(cnav.get_frame_data(t(0), 3, 0, &mut cnav_bits), 0);
    assert_binary("GPS CNAV encoded frame", &cnav_bits);

    let mut l5 = L5CNavBit::new();
    assert_ne!(l5.set_ephemeris(4, &sample_gps_eph(4)), 0);
    let alm_arr: [GpsAlmanac; 32] = alm.try_into().unwrap();
    l5.set_almanac(&alm_arr);
    l5.set_iono_utc(&iono, &utc);
    let mut l5_bits = [0i32; 600];
    assert_eq!(l5.get_frame_data(t(0), 4, 1, &mut l5_bits), 0);
    assert_binary("GPS L5 CNAV encoded frame", &l5_bits);
    assert_eq!(
        &l5_bits[..48],
        &[
            1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0,
            1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0,
        ],
        "GPS L5 CNAV convolution/interleaver prefix"
    );
    assert_eq!(
        &l5_bits[552..600],
        &[
            1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1,
            0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0,
        ],
        "GPS L5 CNAV encoded CRC/FEC suffix"
    );

    let mut cnav2 = CNav2Bit::new();
    assert_ne!(cnav2.set_ephemeris(6, &sample_gps_eph(6)), 0);
    cnav2.set_almanac(&sample_alm(32));
    cnav2.set_iono_utc(&iono, &utc);
    let mut cnav2_bits = vec![0i32; 1800];
    assert_eq!(cnav2.get_frame_data(t(0), 6, 0, &mut cnav2_bits), 0);
    assert_eq!(
        &cnav2_bits[..52],
        &[
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0,
            1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1,
        ],
        "GPS CNAV2 subframe 1 TOI=1 BCH(51,8) symbols"
    );
    let l1c_payload = deinterleave_gps_l1c_payload(&cnav2_bits);
    assert_binary("GPS CNAV2 interleaved payload", &l1c_payload);
    assert_nonzero("GPS CNAV2 subframe 2 symbols", &l1c_payload[..1200]);
    assert_nonzero("GPS CNAV2 subframe 3 symbols", &l1c_payload[1200..]);
    assert_eq!(
        &l1c_payload[..32],
        &[
            0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0,
        ],
        "GPS CNAV2 packed subframe 2 information head"
    );
    assert_eq!(
        &l1c_payload[600..632],
        &[
            0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0,
            1, 1, 0,
        ],
        "GPS CNAV2 subframe 2 LDPC parity head"
    );
    assert_eq!(
        &l1c_payload[1200..1232],
        &[
            0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0,
            0, 1, 0,
        ],
        "GPS CNAV2 packed subframe 3 information head"
    );
    assert_eq!(
        &l1c_payload[1474..1506],
        &[
            0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
            1, 0, 0,
        ],
        "GPS CNAV2 subframe 3 LDPC parity head"
    );
}

#[test]
fn galileo_nav_sync_words_are_bit_exact() {
    let eph = sample_gps_eph(10);
    let alm = sample_alm(36);
    let utc = sample_utc();

    let mut inav = INavBit::new();
    assert_ne!(inav.set_ephemeris(10, &eph), 0);
    inav.set_almanac(&alm);
    inav.set_iono_utc(&sample_nequick(), &utc);
    let mut inav_bits = vec![0i32; 600];
    assert_eq!(inav.get_frame_data(t(2_000), 10, 1, &mut inav_bits), 0);
    assert_bits("Galileo I/NAV even page sync", &inav_bits[0..10], 0x160);
    assert_bits("Galileo I/NAV odd page sync", &inav_bits[250..260], 0x160);

    let mut fnav = FNavBit::new();
    assert_ne!(fnav.set_ephemeris(11, &sample_gps_eph(11)), 0);
    fnav.set_almanac(&alm);
    fnav.set_iono_utc(&sample_iono(), &utc);
    let mut fnav_bits = vec![0i32; 600];
    assert_eq!(fnav.get_frame_data(t(1_000), 11, 0, &mut fnav_bits), 0);
    assert_bits("Galileo F/NAV sync", &fnav_bits[0..12], 0xb70);
}

#[test]
fn beidou_nav_headers_are_bit_exact_for_supported_message_families() {
    let eph_bds = sample_gps_eph(8);
    let alm = sample_alm(63);
    let iono = sample_iono();
    let utc = sample_utc();

    let mut d1d2 = D1D2NavBit::new();
    assert_ne!(d1d2.set_ephemeris(6, &eph_bds), 0);
    d1d2.set_almanac(&alm);
    d1d2.set_iono_utc(&iono, &utc);
    let mut d1d2_bits = vec![0i32; 300];
    assert_eq!(d1d2.get_frame_data(t(6_000), 6, 0, &mut d1d2_bits), 0);
    assert_bits("BeiDou D1/D2 preamble", &d1d2_bits[0..11], 0x712);

    let mut b1c = BCNav1Bit::new();
    assert!(b1c.set_ephemeris(38, &eph_bds));
    assert!(b1c.set_almanac(&alm));
    b1c.set_iono_utc(Some(&iono), Some(&utc));
    let mut b1c_bits = vec![0i32; 2048];
    assert_eq!(b1c.get_frame_data(t(18_000), 38, 0, &mut b1c_bits), 0);
    assert_bits("BeiDou B1C BCH-coded SVID 38", &b1c_bits[0..21], 0x12c149);
    let (b1c_subframe2, b1c_subframe3) = deinterleave_b1c_payload(&b1c_bits);
    assert_binary("BeiDou B1C deinterleaved subframe 2", &b1c_subframe2);
    assert_binary("BeiDou B1C deinterleaved subframe 3", &b1c_subframe3);
    assert_nonzero("BeiDou B1C deinterleaved subframe 2", &b1c_subframe2);
    assert_nonzero("BeiDou B1C deinterleaved subframe 3", &b1c_subframe3);
    let b1c_symbols2 = bits_to_symbols(&b1c_subframe2);
    let b1c_symbols3 = bits_to_symbols(&b1c_subframe3);
    assert_eq!(
        &b1c_symbols2[100..116],
        &[61, 3, 58, 47, 48, 26, 32, 44, 44, 52, 21, 30, 3, 43, 33, 42],
        "BeiDou B1C LDPC(200,100) parity head"
    );
    assert_eq!(
        &b1c_symbols2[192..200],
        &[16, 2, 5, 24, 0, 41, 24, 13],
        "BeiDou B1C LDPC(200,100) parity tail"
    );
    assert_eq!(
        &b1c_symbols3[44..60],
        &[63, 5, 31, 3, 56, 0, 27, 62, 4, 9, 21, 1, 63, 37, 6, 37],
        "BeiDou B1C LDPC(88,44) parity head"
    );
    assert_eq!(
        &b1c_symbols3[80..88],
        &[41, 9, 25, 30, 36, 54, 0, 49],
        "BeiDou B1C LDPC(88,44) parity tail"
    );

    let mut b2a = BCNav2Bit::new();
    let _ = b2a.set_ephemeris(20, &eph_bds);
    b2a.set_almanac(&alm);
    b2a.set_iono_utc(Some(&iono), Some(&utc));
    let mut b2a_bits = vec![0i32; 1500];
    assert!(b2a.get_frame_data(t(9_000), 20, 0, &mut b2a_bits) >= 0);
    assert_bits("BeiDou B-CNAV2 preamble", &b2a_bits[0..24], 0xe24de8);

    let mut b2b = BCNav3Bit::new();
    assert!(b2b.set_ephemeris(22, &eph_bds));
    assert!(b2b.set_almanac(&alm));
    assert!(b2b.set_iono_utc(Some(&iono), Some(&utc)));
    let mut b2b_bits = vec![0i32; 2048];
    assert_eq!(b2b.get_frame_data(t(21_000), 22, 0, &mut b2b_bits), 0);
    assert_bits("BeiDou B-CNAV3 preamble", &b2b_bits[0..16], 0xeb90);
    assert_bits("BeiDou B-CNAV3 PRN", &b2b_bits[16..22], 22);
    assert_bits("BeiDou B-CNAV3 reserved field", &b2b_bits[22..28], 0);
}

#[test]
fn glonass_gnav_time_marker_and_string_padding_are_bit_exact() {
    let mut gnav = GNavBit::new();
    assert_ne!(gnav.set_glonass_ephemeris(1, &sample_glo_eph(1)), 0);
    gnav.set_almanac(&sample_alm(24));
    gnav.set_iono_utc(None, Some(&sample_utc()));

    let mut bits = vec![0i32; 100];
    assert_eq!(gnav.get_frame_data(t(30_000), 1, 0, &mut bits), 0);
    assert_eq!(bits[84], 0, "GLONASS idle bit");
    assert!(bits[93..100].iter().all(|&bit| bit == 0));

    let mut marker = [0i32; 30];
    GNavBit::get_time_marker(&mut marker);
    assert_bits("GLONASS time mark", &marker, 0x3e375096);
}
