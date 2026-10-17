//! Проверка IQ/TXT/manifest перед переносом на носитель.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args_os()
        .nth(1)
        .ok_or("Usage: verify_artifact <IQ file>")?;
    gnss_rust::artifact::verify(std::path::Path::new(&path))?;
    println!("IQ/TXT/manifest: OK");
    Ok(())
}
