use gnss_rust::IFDataGen;
use std::fs;
use std::path::{Path, PathBuf};

fn smoke_dir() -> PathBuf {
    let dir = std::env::temp_dir().join(format!("gnss_rust_if_smoke_{}", std::process::id()));
    fs::create_dir_all(&dir).expect("smoke output directory should be creatable");
    dir
}

fn write_config(format_name: &str, output_path: &Path) -> PathBuf {
    let dir = smoke_dir();
    let config_path = dir.join(format!("smoke_{format_name}.json"));
    let output_name = output_path
        .to_str()
        .expect("temporary output path should be UTF-8");
    let config = format!(
        r#"{{
  "version": 1.0,
  "description": "short deterministic {format_name} smoke",
  "time": {{
    "type": "UTC",
    "year": 2025,
    "month": 6,
    "day": 5,
    "hour": 10,
    "minute": 5,
    "second": 30
  }},
  "trajectory": {{
    "name": "short smoke",
    "initPosition": {{
      "type": "LLA",
      "format": "d",
      "longitude": -114.2847,
      "latitude": 48.4928,
      "altitude": 100.0
    }},
    "initVelocity": {{
      "type": "SCU",
      "speed": 0,
      "course": 0
    }},
    "trajectoryList": [
      {{
        "type": "Const",
        "time": 0.002
      }}
    ]
  }},
  "ephemeris": {{
    "type": "RINEX",
    "name": "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx"
  }},
  "output": {{
    "type": "IFdata",
    "format": "{format_name}",
    "sampleFreq": 0.004,
    "centerFreq": 1575.42,
    "name": "{output_name}",
    "config": {{
      "elevationMask": 5
    }},
    "systemSelect": [
      {{
        "system": "GPS",
        "signal": "L1CA",
        "enable": true
      }}
    ]
  }},
  "power": {{
    "noiseFloor": -172,
    "initPower": {{
      "unit": "dBHz",
      "value": 45
    }},
    "elevationAdjust": true
  }}
}}"#
    );
    fs::write(&config_path, config).expect("smoke config should be writable");
    config_path
}

fn run_smoke(format_name: &str, bytes_per_sample: usize) {
    let dir = smoke_dir();
    let output_path = dir.join(format!("smoke.{format_name}"));
    let _ = fs::remove_file(&output_path);
    let config_path = write_config(format_name, &output_path);

    let mut generator = IFDataGen::new();
    generator
        .load_config(config_path.to_str().unwrap())
        .expect("smoke config should load");
    let stats = generator
        .generate_data()
        .expect("smoke generation should pass");

    let expected_samples = 8usize;
    let expected_bytes = expected_samples * bytes_per_sample;
    assert_eq!(stats.total_samples as usize, expected_samples);

    let data = fs::read(&output_path).expect("smoke IF file should be written");
    assert_eq!(data.len(), expected_bytes, "{format_name} byte size");
    assert!(
        data.iter().any(|&byte| byte != 0),
        "{format_name} smoke output should not be all zero"
    );
}

#[test]
fn short_iq8_generation_writes_interleaved_iq_bytes() {
    run_smoke("IQ8", 2);
}

#[test]
fn short_iq4_generation_writes_one_packed_byte_per_sample() {
    run_smoke("IQ4", 1);
}
