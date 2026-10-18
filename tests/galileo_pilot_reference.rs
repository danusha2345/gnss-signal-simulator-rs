//! Независимый эталон PocketSDR 03787da: E1-B/E1-C PRN 1..50 и CS25.
//! SHA-256 вычислен по 4092 байтам 0/1, без CBOC-поднесущей.
use gnss_rust::{
    constants::SIGNAL_INDEX_E1, pilotbit::get_pilot_bits, prngenerate::PrnGenerate,
    types::GnssSystem,
};
use sha2::{Digest, Sha256};

#[test]
fn e1_data_and_pilot_codes_match_their_independent_reference() {
    for line in include_str!("fixtures/galileo_e1_primary.sha256").lines() {
        let fields: Vec<_> = line.split_whitespace().collect();
        let sv = fields[0].parse().unwrap();
        let prn = PrnGenerate::new(GnssSystem::GalileoSystem, SIGNAL_INDEX_E1 as i32, sv);
        let code = match fields[1] {
            "E1B" => prn.data_prn.unwrap(),
            "E1C" => prn.pilot_prn.unwrap(),
            _ => panic!("Unknown reference channel"),
        };
        let bytes: Vec<u8> = code.into_iter().map(|b| b as u8).collect();
        assert_eq!(bytes.len(), 4092);
        let actual = Sha256::digest(&bytes)
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<String>();
        assert_eq!(actual, fields[2], "PRN {sv} {}", fields[1]);
    }
}

#[test]
fn e1_pilot_secondary_code_matches_cs25_in_transmission_order() {
    let (words, len) = get_pilot_bits(GnssSystem::GalileoSystem, SIGNAL_INDEX_E1, 1).unwrap();
    assert_eq!(len, 25);
    let bits: String = (0..len)
        .map(|i| {
            if words[i / 32] & (1 << (i % 32)) == 0 {
                '0'
            } else {
                '1'
            }
        })
        .collect();
    assert_eq!(bits, "0011100000001010110110010");
}
