//! Проверки контракта пресета до выделения буферов и открытия выходного файла.
use crate::types::UtcTime;
use serde_json::Value;

pub(crate) fn validate_config(json: &Value) -> Result<UtcTime, String> {
    let time = &json["time"];
    if time["type"].as_str() != Some("UTC") {
        return Err("time.type must be UTC".into());
    }
    let integer = |name: &str| -> Result<i32, String> {
        time[name]
            .as_i64()
            .and_then(|v| i32::try_from(v).ok())
            .ok_or_else(|| format!("Missing or invalid time.{name}"))
    };
    let utc = UtcTime {
        Year: integer("year")?,
        Month: integer("month")?,
        Day: integer("day")?,
        Hour: integer("hour")?,
        Minute: integer("minute")?,
        Second: time["second"].as_f64().ok_or("Missing time.second")?,
    };
    let leap = utc.Year % 4 == 0 && (utc.Year % 100 != 0 || utc.Year % 400 == 0);
    let days = [
        31,
        if leap { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    if !(1980..=2099).contains(&utc.Year)
        || !(1..=12).contains(&utc.Month)
        || !(1..=days[(utc.Month.clamp(1, 12) - 1) as usize]).contains(&utc.Day)
        || !(0..24).contains(&utc.Hour)
        || !(0..60).contains(&utc.Minute)
        || !utc.Second.is_finite()
        || !(0.0..60.0).contains(&utc.Second)
        || (utc.Year == 1980 && utc.Month == 1 && utc.Day < 6)
    {
        return Err("Invalid UTC date/time (supported: GPS epoch through 2099; leap-second labels are not supported)".into());
    }

    let output = &json["output"];
    let fs = output["sampleFreq"]
        .as_f64()
        .ok_or("Missing output.sampleFreq")?
        * 1e6;
    if !fs.is_finite()
        || fs < 1000.0
        || fs > i32::MAX as f64
        || (fs / 1000.0 - (fs / 1000.0).round()).abs() > 1e-9
    {
        return Err("sampleFreq must be positive and a multiple of 0.001 MHz".into());
    }
    let center = output["centerFreq"]
        .as_f64()
        .ok_or("Missing output.centerFreq")?
        * 1e6;
    if !center.is_finite()
        || center <= 0.0
        || center > i32::MAX as f64
        || (center - center.round()).abs() > 1e-6
    {
        return Err("centerFreq must fit a positive integer Hz frequency".into());
    }
    if !matches!(output["format"].as_str(), Some("IQ8" | "IQ4")) {
        return Err("output.format must be IQ8 or IQ4".into());
    }
    match output["bocMode"].as_str().unwrap_or("auto") {
        "auto" | "boc11" => {}
        "full" => {
            let has_boc = output["systemSelect"].as_array().is_some_and(|a| {
                a.iter().any(|e| {
                    e["enable"] == true
                        && matches!(e["signal"].as_str(), Some("E1" | "L1C" | "B1C"))
                })
            });
            if has_boc && fs / 2.0 <= (1575.42e6 - center).abs() + 7.161e6 {
                return Err("Full BOC requires Fs/2 > abs(1575.42MHz-center) + 7.161MHz".into());
            }
        }
        _ => return Err("output.bocMode must be auto, full or boc11".into()),
    }
    let filename = output["name"].as_str().ok_or("Missing output.name")?;
    if filename.is_empty() || filename.len() >= 256 || filename.contains('\0') {
        return Err("output.name must contain 1..255 bytes without NUL".into());
    }
    let systems = output["systemSelect"]
        .as_array()
        .ok_or("Missing output.systemSelect")?;
    let mut enabled = false;
    for entry in systems {
        let enable = entry["enable"]
            .as_bool()
            .ok_or("Missing systemSelect.enable")?;
        if !enable {
            continue;
        }
        if !matches!(
            (entry["system"].as_str(), entry["signal"].as_str()),
            (Some("GPS"), Some("L1CA" | "L1C" | "L2C" | "L2P" | "L5"))
                | (Some("Galileo"), Some("E1" | "E5a" | "E5b" | "E6"))
                | (
                    Some("BDS"),
                    Some("B1C" | "B1I" | "B2I" | "B3I" | "B2a" | "B2b")
                )
                | (Some("GLONASS"), Some("G1" | "G2" | "G3"))
        ) {
            return Err("Unsupported enabled system/signal".into());
        }
        enabled = true;
    }
    if !enabled {
        return Err("At least one signal must be enabled".into());
    }

    let segments = json["trajectory"]["trajectoryList"]
        .as_array()
        .ok_or("Missing trajectoryList")?;
    if segments.is_empty() {
        return Err("Empty trajectoryList".into());
    }
    let mut total_ms = 0.0;
    for segment in segments {
        let ms = segment["time"].as_f64().ok_or("Missing trajectory time")? * 1000.0;
        if !ms.is_finite() || ms < 1.0 || (ms - ms.round()).abs() > 1e-6 {
            return Err(
                "Trajectory duration must be a positive whole number of milliseconds".into(),
            );
        }
        total_ms += ms.round();
    }
    if total_ms > (i32::MAX - 50) as f64 {
        return Err("Trajectory duration is too long".into());
    }

    let pos = &json["trajectory"]["initPosition"];
    let number = |name: &str| -> Result<f64, String> {
        pos[name]
            .as_f64()
            .filter(|v| v.is_finite())
            .ok_or_else(|| format!("Invalid initPosition.{name}"))
    };
    match pos["type"].as_str() {
        Some("LLA") => {
            let (lat_limit, lon_limit) = match pos["format"].as_str().unwrap_or("d") {
                "d" => (90.0, 180.0),
                "rad" => (std::f64::consts::FRAC_PI_2, std::f64::consts::PI),
                _ => return Err("Unsupported position format".into()),
            };
            if number("latitude")?.abs() > lat_limit || number("longitude")?.abs() > lon_limit {
                return Err("Invalid latitude/longitude".into());
            }
            number("altitude")?;
        }
        Some("ECEF") => {
            let radius = number("x")?.hypot(number("y")?).hypot(number("z")?);
            if !radius.is_finite() || radius < 1.0 {
                return Err("Invalid ECEF position".into());
            }
        }
        _ => return Err("Unsupported initPosition.type".into()),
    }
    Ok(utc)
}
