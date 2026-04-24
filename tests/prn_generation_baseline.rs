use gnss_rust::constants::{
    SIGNAL_INDEX_B1C, SIGNAL_INDEX_B1I, SIGNAL_INDEX_B2A, SIGNAL_INDEX_E1, SIGNAL_INDEX_E6,
    SIGNAL_INDEX_G1, SIGNAL_INDEX_G3, SIGNAL_INDEX_L1C, SIGNAL_INDEX_L1CA, SIGNAL_INDEX_L5,
};
use gnss_rust::prngenerate::{
    PrnGenerate, PRN_ATTRIBUTE_BOC, PRN_ATTRIBUTE_CBOC, PRN_ATTRIBUTE_QMBOC,
};
use gnss_rust::types::GnssSystem;

struct SignalCase {
    name: &'static str,
    system: GnssSystem,
    signal_index: usize,
    svid: i32,
    data_len: usize,
    pilot_len: Option<usize>,
    chip_rate: i32,
    data_period: i32,
    pilot_period: i32,
    required_attributes: u32,
}

fn assert_binary_code(name: &str, code: &[i32]) {
    assert!(!code.is_empty(), "{name} code should not be empty");
    assert!(
        code.iter().all(|&chip| chip == 0 || chip == 1),
        "{name} code must contain only binary chips"
    );
    assert!(
        code.iter().any(|&chip| chip == 0) && code.iter().any(|&chip| chip == 1),
        "{name} code should not be degenerate"
    );
}

fn balance_ratio(code: &[i32]) -> f64 {
    let ones = code.iter().filter(|&&chip| chip != 0).count() as i32;
    let zeros = code.len() as i32 - ones;
    ((ones - zeros).abs() as f64) / code.len() as f64
}

fn bipolar_correlation(left: &[i32], right: &[i32]) -> i32 {
    assert_eq!(left.len(), right.len());
    left.iter()
        .zip(right.iter())
        .map(|(&a, &b)| {
            let a = if a != 0 { -1 } else { 1 };
            let b = if b != 0 { -1 } else { 1 };
            a * b
        })
        .sum()
}

#[test]
fn supported_signal_codes_have_expected_lengths_attributes_and_binary_chips() {
    let cases = [
        SignalCase {
            name: "GPS L1CA",
            system: GnssSystem::GpsSystem,
            signal_index: SIGNAL_INDEX_L1CA,
            svid: 1,
            data_len: 1023,
            pilot_len: None,
            chip_rate: 1023,
            data_period: 1,
            pilot_period: 1,
            required_attributes: 0,
        },
        SignalCase {
            name: "GPS L1C",
            system: GnssSystem::GpsSystem,
            signal_index: SIGNAL_INDEX_L1C,
            svid: 7,
            data_len: 10230,
            pilot_len: Some(10230),
            chip_rate: 2046,
            data_period: 10,
            pilot_period: 10,
            required_attributes: PRN_ATTRIBUTE_BOC | PRN_ATTRIBUTE_QMBOC,
        },
        SignalCase {
            name: "GPS L5",
            system: GnssSystem::GpsSystem,
            signal_index: SIGNAL_INDEX_L5,
            svid: 11,
            data_len: 10230,
            pilot_len: Some(10230),
            chip_rate: 10230,
            data_period: 1,
            pilot_period: 1,
            required_attributes: 0,
        },
        SignalCase {
            name: "BDS B1I",
            system: GnssSystem::BdsSystem,
            signal_index: SIGNAL_INDEX_B1I,
            svid: 6,
            data_len: 2046,
            pilot_len: None,
            chip_rate: 2046,
            data_period: 1,
            pilot_period: 1,
            required_attributes: 0,
        },
        SignalCase {
            name: "BDS B1C",
            system: GnssSystem::BdsSystem,
            signal_index: SIGNAL_INDEX_B1C,
            svid: 19,
            data_len: 10230,
            pilot_len: Some(10230),
            chip_rate: 2046,
            data_period: 10,
            pilot_period: 10,
            required_attributes: PRN_ATTRIBUTE_BOC | PRN_ATTRIBUTE_QMBOC,
        },
        SignalCase {
            name: "BDS B2A",
            system: GnssSystem::BdsSystem,
            signal_index: SIGNAL_INDEX_B2A,
            svid: 23,
            data_len: 10230,
            pilot_len: Some(10230),
            chip_rate: 10230,
            data_period: 1,
            pilot_period: 1,
            required_attributes: 0,
        },
        SignalCase {
            name: "Galileo E1",
            system: GnssSystem::GalileoSystem,
            signal_index: SIGNAL_INDEX_E1,
            svid: 3,
            data_len: 4092,
            pilot_len: Some(4092),
            chip_rate: 2046,
            data_period: 4,
            pilot_period: 4,
            required_attributes: PRN_ATTRIBUTE_BOC | PRN_ATTRIBUTE_CBOC,
        },
        SignalCase {
            name: "Galileo E6",
            system: GnssSystem::GalileoSystem,
            signal_index: SIGNAL_INDEX_E6,
            svid: 12,
            data_len: 5115,
            pilot_len: Some(5115),
            chip_rate: 5115,
            data_period: 1,
            pilot_period: 1,
            required_attributes: 0,
        },
        SignalCase {
            name: "GLONASS G1",
            system: GnssSystem::GlonassSystem,
            signal_index: SIGNAL_INDEX_G1,
            svid: 1,
            data_len: 511,
            pilot_len: None,
            chip_rate: 511,
            data_period: 1,
            pilot_period: 1,
            required_attributes: 0,
        },
        SignalCase {
            name: "GLONASS G3",
            system: GnssSystem::GlonassSystem,
            signal_index: SIGNAL_INDEX_G3,
            svid: 5,
            data_len: 10230,
            pilot_len: Some(10230),
            chip_rate: 10230,
            data_period: 1,
            pilot_period: 1,
            required_attributes: PRN_ATTRIBUTE_BOC,
        },
    ];

    for case in cases {
        let generator = PrnGenerate::new(case.system, case.signal_index as i32, case.svid);
        let data = generator
            .get_data_prn()
            .unwrap_or_else(|| panic!("{} data PRN should be generated", case.name));
        let attribute = generator
            .get_attribute()
            .unwrap_or_else(|| panic!("{} PRN attribute should be present", case.name));

        assert_binary_code(case.name, data);
        assert_eq!(data.len(), case.data_len, "{} data length", case.name);
        assert!(
            balance_ratio(data) < 0.10,
            "{} data code is unexpectedly unbalanced",
            case.name
        );
        assert_eq!(
            attribute.chip_rate, case.chip_rate,
            "{} chip rate",
            case.name
        );
        assert_eq!(
            attribute.data_period, case.data_period,
            "{} data period",
            case.name
        );
        assert_eq!(
            attribute.pilot_period, case.pilot_period,
            "{} pilot period",
            case.name
        );
        assert_eq!(
            attribute.attribute & case.required_attributes,
            case.required_attributes,
            "{} required attributes",
            case.name
        );

        match (case.pilot_len, generator.get_pilot_prn()) {
            (Some(expected_len), Some(pilot)) => {
                assert_binary_code(case.name, pilot);
                assert_eq!(pilot.len(), expected_len, "{} pilot length", case.name);
                assert!(
                    balance_ratio(pilot) < 0.10,
                    "{} pilot code is unexpectedly unbalanced",
                    case.name
                );
            }
            (None, None) => {}
            (Some(_), None) => panic!("{} pilot PRN should be generated", case.name),
            (None, Some(_)) => panic!("{} pilot PRN should not be generated", case.name),
        }
    }
}

#[test]
fn gps_l1ca_prns_are_distinct_and_have_gold_code_correlation_bounds() {
    let prn1 = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, 1);
    let prn2 = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, 2);
    let code1 = prn1.get_data_prn().expect("GPS PRN 1 should exist");
    let code2 = prn2.get_data_prn().expect("GPS PRN 2 should exist");

    assert_eq!(code1.len(), 1023);
    assert_eq!(code2.len(), 1023);
    assert_eq!(bipolar_correlation(code1, code1), code1.len() as i32);

    let cross_correlation = bipolar_correlation(code1, code2).abs();
    assert!(
        cross_correlation <= 80,
        "GPS L1CA PRN 1/2 cross-correlation too high: {cross_correlation}"
    );
}

#[test]
fn pilot_and_data_components_are_separate_for_modern_signals() {
    for (system, signal_index, svid, name) in [
        (
            GnssSystem::GpsSystem,
            SIGNAL_INDEX_L1C,
            3,
            "GPS L1C data/pilot",
        ),
        (
            GnssSystem::BdsSystem,
            SIGNAL_INDEX_B1C,
            14,
            "BDS B1C data/pilot",
        ),
        (
            GnssSystem::GalileoSystem,
            SIGNAL_INDEX_E1,
            8,
            "Galileo E1 data/pilot",
        ),
    ] {
        let generator = PrnGenerate::new(system, signal_index as i32, svid);
        let data = generator
            .get_data_prn()
            .unwrap_or_else(|| panic!("{name} data PRN should exist"));
        let pilot = generator
            .get_pilot_prn()
            .unwrap_or_else(|| panic!("{name} pilot PRN should exist"));

        assert_eq!(data.len(), pilot.len(), "{name} lengths should match");
        assert_ne!(data, pilot, "{name} channels should use different codes");
    }
}

#[test]
fn get_prn_bit_reads_data_channel_and_wraps_chip_indices() {
    let generator = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, 9);
    let code = generator
        .get_data_prn()
        .expect("GPS L1CA PRN should exist for wrap test");
    let wrapped_index = code.len() as i32 + 17;

    assert_eq!(generator.get_prn_bit(0), code[0] != 0);
    assert_eq!(generator.get_prn_bit(17), code[17] != 0);
    assert_eq!(generator.get_prn_bit(wrapped_index), code[17] != 0);
}

#[test]
fn unsupported_signal_or_svid_does_not_generate_codes() {
    let invalid_svid = PrnGenerate::new(GnssSystem::GpsSystem, SIGNAL_INDEX_L1CA as i32, 33);
    assert!(invalid_svid.get_data_prn().is_none());
    assert!(invalid_svid.get_pilot_prn().is_none());

    let invalid_signal = PrnGenerate::new(GnssSystem::GalileoSystem, SIGNAL_INDEX_L1CA as i32, 1);
    assert!(invalid_signal.get_data_prn().is_none());
    assert!(invalid_signal.get_pilot_prn().is_none());
}
