use gnss_rust::gps_pilot::{generate, GpsPilotConfig};
use gnss_rust::types::*;

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let config = GpsPilotConfig {
        rinex_path: args
            .get(1)
            .cloned()
            .unwrap_or_else(|| "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx".to_string()),
        output_path: "generated_files/gps_pilot.C8".to_string(),
        receiver_lla: LlaPosition {
            lat: 55.7539_f64.to_radians(),
            lon: 37.6208_f64.to_radians(),
            alt: 150.0,
        },
        utc_time: UtcTime {
            Year: 2025,
            Month: 6,
            Day: 5,
            Hour: 10,
            Minute: 5,
            Second: 30.0,
        },
        duration_s: 300.0,
        sample_rate_hz: 5_000_000.0,
        if_freq_hz: 0.0,
        cn0_db: 55.0,
        elevation_mask_deg: 5.0,
    };

    let start = std::time::Instant::now();
    match generate(&config) {
        Ok(result) => {
            println!(
                "Generated {} samples ({} satellites) in {:.1}s",
                result.total_samples,
                result.satellites.len(),
                start.elapsed().as_secs_f64()
            );
        }
        Err(e) => eprintln!("Error: {}", e),
    }
}
