//! Согласованность IQ и sidecar. Manifest публикуется последним как запись о завершении.
use sha2::{Digest, Sha256};
use std::io::{self, Read};
use std::path::Path;

pub fn sha256_file(path: &Path) -> io::Result<String> {
    let mut file = std::fs::File::open(path)?;
    let mut hasher = Sha256::new();
    let mut buf = vec![0u8; 1024 * 1024];
    loop {
        let n = file.read(&mut buf)?;
        if n == 0 {
            break;
        }
        hasher.update(&buf[..n]);
    }
    Ok(hex(&hasher.finalize()))
}

/// Проверяет завершённый набор перед переносом/Replay. Не запускает передачу.
pub fn verify(path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let manifest: serde_json::Value =
        serde_json::from_slice(&std::fs::read(path.with_extension("manifest.json"))?)?;
    if manifest["schema"] != 1 {
        return Err("Unsupported manifest schema".into());
    }
    if manifest["iq_sha256"].as_str() != Some(sha256_file(path)?.as_str()) {
        return Err("IQ SHA-256 mismatch".into());
    }
    if manifest["bytes"].as_u64() != Some(std::fs::metadata(path)?.len()) {
        return Err("IQ size mismatch".into());
    }
    if manifest["txt_sha256"].as_str() != Some(sha256_file(&path.with_extension("TXT"))?.as_str()) {
        return Err("TXT SHA-256 mismatch".into());
    }
    let fs = manifest["sample_rate_hz"]
        .as_u64()
        .ok_or("Invalid sample rate")?;
    let center = manifest["center_frequency_hz"]
        .as_u64()
        .ok_or("Invalid center frequency")?;
    let duration = manifest["duration_ms"].as_u64().ok_or("Invalid duration")?;
    let samples = manifest["samples"].as_u64().ok_or("Invalid sample count")?;
    let bytes_per_sample = match manifest["format"].as_str() {
        Some("IQ4") => 1,
        Some("IQ8") => 2,
        _ => return Err("Invalid IQ format".into()),
    };
    if fs == 0
        || fs % 1000 != 0
        || duration.checked_mul(fs / 1000) != Some(samples)
        || samples.checked_mul(bytes_per_sample) != manifest["bytes"].as_u64()
    {
        return Err("Inconsistent sample/duration/size metadata".into());
    }
    let expected_txt = format!("sample_rate={fs}\ncenter_frequency={center}\n");
    if std::fs::read(path.with_extension("TXT"))? != expected_txt.as_bytes() {
        return Err("TXT settings do not match manifest".into());
    }
    Ok(())
}

pub(crate) fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}
