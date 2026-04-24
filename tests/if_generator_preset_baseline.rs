use gnss_rust::types::{
    OutputFormat, GEN_B1C, GEN_B1I, GEN_B2A, GEN_B2B, GEN_B2I, GEN_B3I, GEN_E1, GEN_E5A, GEN_E5B,
    GEN_E6, GEN_G1, GEN_G2, GEN_G3, GEN_L1C, GEN_L1CA, GEN_L2C, GEN_L2P, GEN_L5,
};
use gnss_rust::IFDataGen;
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};

fn preset_paths() -> Vec<PathBuf> {
    let mut paths: Vec<PathBuf> = fs::read_dir("presets")
        .expect("presets directory should exist")
        .map(|entry| entry.expect("preset entry should be readable").path())
        .filter(|path| path.extension().and_then(|ext| ext.to_str()) == Some("json"))
        .collect();
    paths.sort();
    paths
}

fn generation_bit_for_signal(system: &str, signal: &str) -> Option<u32> {
    match (system, signal) {
        ("GPS", "L1CA") => Some(GEN_L1CA),
        ("GPS", "L1C") => Some(GEN_L1C),
        ("GPS", "L2C") => Some(GEN_L2C),
        ("GPS", "L2P") => Some(GEN_L2P),
        ("GPS", "L5") => Some(GEN_L5),
        ("BDS", "B1C") => Some(GEN_B1C),
        ("BDS", "B1I") => Some(GEN_B1I),
        ("BDS", "B2a") => Some(GEN_B2A),
        ("BDS", "B2I") => Some(GEN_B2I),
        ("BDS", "B2b") => Some(GEN_B2B),
        ("BDS", "B3I") => Some(GEN_B3I),
        ("Galileo", "E1") => Some(GEN_E1),
        ("Galileo", "E5a") => Some(GEN_E5A),
        ("Galileo", "E5b") => Some(GEN_E5B),
        ("Galileo", "E6") => Some(GEN_E6),
        ("GLONASS", "G1") => Some(GEN_G1),
        ("GLONASS", "G2") => Some(GEN_G2),
        ("GLONASS", "G3") => Some(GEN_G3),
        _ => None,
    }
}

fn output_filename(param: &[u8; 256]) -> String {
    let len = param
        .iter()
        .position(|&byte| byte == 0)
        .unwrap_or(param.len());
    String::from_utf8_lossy(&param[..len]).to_string()
}

fn parse_preset(path: &Path) -> Value {
    let content = fs::read_to_string(path).expect("preset should be readable");
    serde_json::from_str(&content).unwrap_or_else(|err| panic!("invalid JSON in {path:?}: {err}"))
}

#[test]
fn all_presets_are_valid_and_reference_safe_outputs() {
    let paths = preset_paths();
    assert!(!paths.is_empty(), "preset inventory should not be empty");

    for path in paths {
        let json = parse_preset(&path);
        let name = path.display();
        let output = json
            .get("output")
            .unwrap_or_else(|| panic!("{name} missing output section"));
        let trajectory = json
            .get("trajectory")
            .unwrap_or_else(|| panic!("{name} missing trajectory section"));

        let format = output
            .get("format")
            .and_then(Value::as_str)
            .unwrap_or_else(|| panic!("{name} missing output.format"));
        assert!(matches!(format, "IQ8" | "IQ4"), "{name} unsupported format");

        let output_name = output
            .get("name")
            .and_then(Value::as_str)
            .unwrap_or_else(|| panic!("{name} missing output.name"));
        assert!(
            output_name.starts_with("generated_files/"),
            "{name} output should stay under generated_files"
        );
        assert!(
            (format == "IQ8" && output_name.ends_with(".C8"))
                || (format == "IQ4" && output_name.ends_with(".C4")),
            "{name} output extension should match format"
        );

        assert!(
            output
                .get("sampleFreq")
                .and_then(Value::as_f64)
                .unwrap_or(0.0)
                > 0.0,
            "{name} sampleFreq should be positive"
        );
        assert!(
            output
                .get("centerFreq")
                .and_then(Value::as_f64)
                .unwrap_or(0.0)
                > 0.0,
            "{name} centerFreq should be positive"
        );
        assert!(
            trajectory
                .get("trajectoryList")
                .and_then(Value::as_array)
                .and_then(|items| items.first())
                .and_then(|item| item.get("time"))
                .and_then(Value::as_f64)
                .unwrap_or(0.0)
                > 0.0,
            "{name} trajectory duration should be positive"
        );

        let ephemeris = json
            .get("ephemeris")
            .and_then(|section| section.get("name"))
            .and_then(Value::as_str)
            .unwrap_or_else(|| panic!("{name} missing ephemeris.name"));
        assert!(
            Path::new(ephemeris).exists(),
            "{name} ephemeris path should exist: {ephemeris}"
        );

        let enabled: Vec<(&str, &str)> = output
            .get("systemSelect")
            .and_then(Value::as_array)
            .unwrap_or_else(|| panic!("{name} missing output.systemSelect"))
            .iter()
            .filter(|entry| {
                entry
                    .get("enable")
                    .and_then(Value::as_bool)
                    .unwrap_or(false)
            })
            .map(|entry| {
                let system = entry
                    .get("system")
                    .and_then(Value::as_str)
                    .expect("enabled signal should have system");
                let signal = entry
                    .get("signal")
                    .and_then(Value::as_str)
                    .expect("enabled signal should have signal");
                (system, signal)
            })
            .collect();
        assert!(
            !enabled.is_empty(),
            "{name} should enable at least one signal"
        );
        for (system, signal) in enabled {
            assert!(
                generation_bit_for_signal(system, signal).is_some(),
                "{name} has unsupported enabled signal {system} {signal}"
            );
        }
    }
}

#[test]
fn load_config_sets_generation_bits_for_edge_signal_presets() {
    let mut gps_l2p = IFDataGen::new();
    gps_l2p
        .load_config("presets/gps_l2p.json")
        .expect("GPS L2P preset should load");
    assert!(
        gps_l2p
            .output_param
            .CompactConfig
            .is_signal_enabled(GEN_L2P),
        "GPS L2P preset must enable GEN_L2P"
    );
    assert_eq!(gps_l2p.output_param.Format, OutputFormat::OutputFormatIQ8);
    assert_eq!(gps_l2p.output_param.SampleFreq, 11_000_000);
    assert_eq!(gps_l2p.output_param.Interval, 10_000);
    assert_eq!(
        output_filename(&gps_l2p.output_param.filename),
        "generated_files/gps_l2p.C8"
    );

    let mut glo_g3 = IFDataGen::new();
    glo_g3
        .load_config("presets/glo_g3.json")
        .expect("GLONASS G3 preset should load");
    assert!(
        glo_g3.output_param.CompactConfig.is_signal_enabled(GEN_G3),
        "GLONASS G3 preset must enable GEN_G3"
    );
    assert_eq!(glo_g3.output_param.Format, OutputFormat::OutputFormatIQ8);
    assert_eq!(glo_g3.output_param.SampleFreq, 12_000_000);
    assert_eq!(
        output_filename(&glo_g3.output_param.filename),
        "generated_files/glo_g3.C8"
    );
}
