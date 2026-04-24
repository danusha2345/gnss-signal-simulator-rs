use gnss_rust::{HybridAccelerator, SafeAvx512Processor};

#[test]
fn avx512_prn_batch_matches_scalar_for_full_and_tail_chunks() {
    let input: Vec<f32> = (0..37)
        .map(|idx| if idx % 3 == 0 { -1.0 } else { 1.0 })
        .collect();
    let amplitude = 0.25;

    let accelerated = SafeAvx512Processor::process_prn_batch(&input, amplitude);
    let expected: Vec<f32> = input.iter().map(|value| value * amplitude).collect();

    assert_eq!(accelerated.len(), expected.len());
    for (actual, expected) in accelerated.iter().zip(expected.iter()) {
        assert!(
            (actual - expected).abs() < f32::EPSILON,
            "expected {expected}, got {actual}"
        );
    }
}

#[test]
fn hybrid_prn_processing_preserves_values_and_length() {
    let input: Vec<f32> = (0..64)
        .map(|idx| if idx % 2 == 0 { 1.0 } else { -1.0 })
        .collect();

    let processed = HybridAccelerator::new().optimal_prn_processing(&input);

    assert_eq!(processed, input);
}

#[cfg(feature = "gpu")]
#[test]
fn cuda_availability_probe_matches_constructor() {
    use gnss_rust::CudaGnssAccelerator;

    assert_eq!(
        CudaGnssAccelerator::is_available(),
        CudaGnssAccelerator::new().is_ok()
    );
}
