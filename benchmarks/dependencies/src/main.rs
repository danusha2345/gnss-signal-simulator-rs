use rand_distr::{Distribution, StandardNormal};
use rand_new::{RngExt, SeedableRng as NewSeed};
use rand_old::{Rng, SeedableRng as OldSeed};
use std::{hint::black_box, time::Instant};
fn measure(name: &str, mut generator: impl FnMut() -> f64) {
    let n = 2_000_000;
    let start = Instant::now();
    let mut values = Vec::with_capacity(n);
    for _ in 0..n {
        values.push(black_box(generator()));
    }
    let elapsed = start.elapsed().as_secs_f64();
    let mean = values.iter().sum::<f64>() / n as f64;
    let var = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n as f64;
    let lag = values.windows(2).map(|w| w[0] * w[1]).sum::<f64>() / (n - 1) as f64;
    let mut band = [0.0; 8]; // Суммарная энергия восьми DFT-полос, 256 сегментов × 256 отсчётов.
    for window in values[..65536].chunks(256) {
        for k in 1..128 {
            let mut re = 0.0;
            let mut im = 0.0;
            for (i, v) in window.iter().enumerate() {
                let a = std::f64::consts::TAU * (k * i) as f64 / 256.0;
                re += v * a.cos();
                im += v * a.sin();
            }
            band[(k - 1) / 16] += (re * re + im * im) / 256.0;
        }
    }
    println!("{name}: seconds={elapsed:.6}, mean={mean:.6}, variance={var:.6}, lag1={lag:.6}, bands={band:?}");
    assert!(mean.abs() < 0.004 && (var - 1.0).abs() < 0.01 && lag.abs() < 0.005);
    // Оценка округления только буфера f32; время/фазы остаются f64.
    let mse = values
        .iter()
        .map(|v| (*v - (*v as f32) as f64).powi(2))
        .sum::<f64>()
        / n as f64;
    let different = values
        .iter()
        .filter(|v| {
            ((**v * 0.28 * 127.0).clamp(-128.0, 127.0) as i8)
                != (((**v as f32 as f64) * 0.28 * 127.0).clamp(-128.0, 127.0) as i8)
        })
        .count();
    println!("f32 MSE={mse:e}, changed IQ8 components={different}/{n}");
}
fn main() {
    for _ in 0..3 {
        let mut old = rand_old::rngs::SmallRng::seed_from_u64(42);
        let mut spare = None;
        measure("rand0.8-polar", || {
            if let Some(x) = spare.take() {
                return x;
            }
            loop {
                let x = 2.0 * old.gen::<f64>() - 1.0;
                let y = 2.0 * old.gen::<f64>() - 1.0;
                let r = x * x + y * y;
                if r > 0.0 && r < 1.0 {
                    let f = (-2.0 * r.ln() / r).sqrt();
                    spare = Some(y * f);
                    return x * f;
                }
            }
        });
        let mut new = rand_new::rngs::SmallRng::seed_from_u64(42);
        measure("rand0.10-Normal", || StandardNormal.sample(&mut new));
        // Compile/use versioned random API as well.
        black_box(new.random::<f64>());
    }
}
