//! Контракт загрузки, результата и сохранности предыдущего артефакта.
use gnss_rust::IFDataGen;
use serde_json::{json, Value};
use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicUsize, Ordering};
static NEXT: AtomicUsize = AtomicUsize::new(0);
struct Fixture {
    dir: PathBuf,
    config: Value,
}
impl Fixture {
    fn new() -> Self {
        let dir = PathBuf::from(format!(
            ".scratch/contract_{}_{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir_all(&dir).unwrap();
        let mut config: Value = serde_json::from_str(
            &fs::read_to_string("presets/gps_gal_circle100m_10kmh_4m2_300s.json").unwrap(),
        )
        .unwrap();
        config["trajectory"]["trajectoryList"][0]["time"] = 0.002.into();
        config["output"]["sampleFreq"] = 0.004.into();
        config["output"]["name"] = dir.join("out.C8").to_str().unwrap().into();
        Self { dir, config }
    }
    fn write(&self, pretty: bool) -> PathBuf {
        let path = self.dir.join("config.json");
        let data = if pretty {
            serde_json::to_vec_pretty(&self.config)
        } else {
            serde_json::to_vec(&self.config)
        }
        .unwrap();
        fs::write(&path, data).unwrap();
        path
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.dir);
    }
}

#[test]
fn formatting_and_deleted_config_do_not_change_glonass_generation() {
    let mut f = Fixture::new();
    f.config["time"]["hour"] = 15.into();
    f.config["output"]["systemSelect"] = json!([{"system":"GLONASS","signal":"G1","enable":true}]);
    f.config["output"]["centerFreq"] = 1602.0.into();
    let mut outputs = Vec::new();
    for pretty in [true, false] {
        let path = f.write(pretty);
        let mut g = IFDataGen::new();
        g.load_config(path.to_str().unwrap()).unwrap();
        fs::remove_file(path).unwrap();
        let stats = g.generate_data().unwrap();
        assert_eq!(stats.satellites_processed, 9);
        assert_eq!(stats.total_samples, 8);
        assert_eq!(stats.file_size_mb, Some(16.0 / 1048576.0));
        assert!(stats.total_time_ms > 0.0 && stats.signal_processing_time_ms > 0.0);
        assert!(!stats.cuda_accelerated && !stats.avx512_accelerated);
        outputs.push(fs::read(f.dir.join("out.C8")).unwrap());
    }
    assert_eq!(outputs[0], outputs[1]);
}

#[test]
fn rejects_invalid_config_without_touching_output() {
    let cases = [
        ("/output/sampleFreq", json!(-1)),
        ("/output/sampleFreq", json!(0)),
        ("/output/sampleFreq", json!(4.200001)),
        ("/trajectory/trajectoryList/0/time", json!(0.0025)),
        ("/output/centerFreq", json!(3000)),
        ("/output/format", json!("unknown")),
        ("/time/month", json!(13)),
        ("/time/day", json!(32)),
        ("/time/second", json!(60)),
        ("/trajectory/initPosition/latitude", json!(91)),
        ("/output/name", json!("x".repeat(256))),
        ("/output/systemSelect", json!([])),
    ];
    for (key, value) in cases {
        let mut f = Fixture::new();
        *f.config.pointer_mut(key).unwrap() = value;
        fs::write(f.dir.join("out.C8"), b"previous").unwrap();
        let path = f.write(true);
        let mut g = IFDataGen::new();
        assert!(
            g.load_config(path.to_str().unwrap()).is_err(),
            "accepted {key}"
        );
        assert!(g.generate_data().is_err());
        assert_eq!(fs::read(f.dir.join("out.C8")).unwrap(), b"previous");
    }
}

#[test]
fn empty_navigation_preserves_previous_output_and_reloading_clears_satellites() {
    let mut f = Fixture::new();
    let path = f.write(true);
    let mut g = IFDataGen::new();
    g.load_config(path.to_str().unwrap()).unwrap();
    g.generate_data().unwrap();
    let previous = fs::read(f.dir.join("out.C8")).unwrap();
    let nav = f.dir.join("empty.rnx");
    fs::write(
        &nav,
        "                                                            END OF HEADER\n",
    )
    .unwrap();
    f.config["ephemeris"]["name"] = nav.to_str().unwrap().into();
    let path = f.write(true);
    g.load_config(path.to_str().unwrap()).unwrap();
    assert!(g
        .generate_data()
        .unwrap_err()
        .to_string()
        .contains("No usable satellite"));
    assert_eq!(fs::read(f.dir.join("out.C8")).unwrap(), previous);
    assert!(!fs::read_dir(&f.dir).unwrap().any(|e| e
        .unwrap()
        .file_name()
        .to_string_lossy()
        .ends_with(".partial")));
}

#[test]
fn failure_to_reload_invalidates_previous_scenario_and_input_cannot_be_output() {
    let mut f = Fixture::new();
    let path = f.write(true);
    let mut g = IFDataGen::new();
    g.load_config(path.to_str().unwrap()).unwrap();
    assert!(g
        .load_config(f.dir.join("missing.json").to_str().unwrap())
        .is_err());
    assert!(g.generate_data().is_err());
    f.config["output"]["name"] = path.to_str().unwrap().into();
    let path = f.write(true);
    let before = fs::read(&path).unwrap();
    assert!(g.load_config(path.to_str().unwrap()).is_err());
    assert_eq!(fs::read(path).unwrap(), before);
}

#[test]
fn generation_reports_actual_channels_and_clipping() {
    let mut f = Fixture::new();
    f.config["output"]["sampleFreq"] = 4.2.into();
    f.config["trajectory"]["trajectoryList"][0]["time"] = 0.01.into();
    let path = f.write(true);
    let mut g = IFDataGen::new();
    g.load_config(path.to_str().unwrap()).unwrap();
    let stats = g.generate_data().unwrap();
    assert_eq!(stats.total_samples, 42000);
    assert_eq!(stats.satellites_processed, 19);
    assert_eq!(stats.samples_generated, stats.total_samples);
    assert!(stats.clipped_samples > 0);
    assert_eq!(
        fs::metadata(f.dir.join("out.C8")).unwrap().len(),
        stats.total_samples * 2
    );
}

#[test]
fn legacy_entry_uses_the_same_generation_contract() {
    let f = Fixture::new();
    let path = f.write(false);
    let mut generator = IFDataGen::new();
    generator
        .main(vec!["legacy".into(), path.to_str().unwrap().into()])
        .unwrap();
    let legacy = fs::read(f.dir.join("out.C8")).unwrap();
    generator.generate_data().unwrap();
    assert_eq!(fs::read(f.dir.join("out.C8")).unwrap(), legacy);
}

#[test]
fn metadata_matches_iq_and_detects_corruption() {
    let f = Fixture::new();
    let path = f.write(true);
    let mut g = IFDataGen::new();
    g.load_config(path.to_str().unwrap()).unwrap();
    g.generate_data().unwrap();
    let iq = f.dir.join("out.C8");
    gnss_rust::artifact::verify(&iq).unwrap();
    let manifest: Value =
        serde_json::from_slice(&fs::read(f.dir.join("out.manifest.json")).unwrap()).unwrap();
    assert_eq!(manifest["sample_rate_hz"], 4000);
    assert_eq!(manifest["center_frequency_hz"], 1575420000);
    assert_eq!(manifest["duration_ms"], 2);
    assert_eq!(manifest["bytes"], 16);
    fs::write(f.dir.join("out.TXT"), b"sample_rate=4800000\n").unwrap();
    assert!(gnss_rust::artifact::verify(&iq)
        .unwrap_err()
        .to_string()
        .contains("TXT"));
    g.generate_data().unwrap();
    gnss_rust::artifact::verify(&iq).unwrap();
    let mut manifest: Value =
        serde_json::from_slice(&fs::read(f.dir.join("out.manifest.json")).unwrap()).unwrap();
    manifest["center_frequency_hz"] = 1575435000_u64.into();
    fs::write(
        f.dir.join("out.manifest.json"),
        serde_json::to_vec(&manifest).unwrap(),
    )
    .unwrap();
    assert!(gnss_rust::artifact::verify(&iq)
        .unwrap_err()
        .to_string()
        .contains("TXT settings"));
    g.generate_data().unwrap();
    fs::write(&iq, b"corrupted").unwrap();
    assert!(gnss_rust::artifact::verify(&iq)
        .unwrap_err()
        .to_string()
        .contains("IQ"));
}
#[test]
fn full_boc_rejects_narrow_band_before_generation() {
    let mut f = Fixture::new();
    f.config["output"]["bocMode"] = "full".into();
    let mut g = IFDataGen::new();
    assert!(g
        .load_config(f.write(true).to_str().unwrap())
        .unwrap_err()
        .to_string()
        .contains("Full BOC"));
    f.config["output"]["sampleFreq"] = 20.into();
    g.load_config(f.write(true).to_str().unwrap()).unwrap();
}
