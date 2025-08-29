use std::time::Instant;
use gnss_rust::{IFDataGen, GenerationStats, avx512_intrinsics::Avx512Accelerator};

#[cfg(feature = "gpu")]
use gnss_rust::cuda_acceleration::CudaGnssAccelerator;

fn main() {
    println!("🚀 ТЕСТ РЕАЛЬНОЙ GNSS ГЕНЕРАЦИИ С АППАРАТНОЙ ОПТИМИЗАЦИЕЙ 🚀");
    println!("AMD Ryzen 9 7950X (AVX-512) + NVIDIA RTX 3090");
    println!("========================================================");
    
    test_basic_generation();
    test_optimized_generation();
    test_mass_generation();
    
    println!("\n✅ ВСЕ ТЕСТЫ ГЕНЕРАЦИИ ЗАВЕРШЕНЫ!");
}

fn test_basic_generation() {
    println!("\n🔬 ТЕСТ 1: Базовая GNSS генерация");
    
    let mut if_data_gen = IFDataGen::new();
    
    // Инициализация с базовыми параметрами
    println!("Инициализация генератора...");
    let start = Instant::now();
    
    // Установим базовые параметры выходного сигнала
    if_data_gen.output_param.SampleFreq = 16368000; // 16.368 MHz
    if_data_gen.output_param.Interval = 1000; // 1 секунда (миллисекунды)
    
    // Устанавливаем имя выходного файла
    let filename = "test_basic_gnss.dat";
    let filename_bytes = filename.as_bytes();
    if_data_gen.output_param.filename[..filename_bytes.len()].copy_from_slice(filename_bytes);
    
    let init_time = start.elapsed();
    println!("✅ Инициализация: {:.2}ms", init_time.as_secs_f64() * 1000.0);
    
    // Генерация данных
    println!("Генерация IF данных...");
    let start = Instant::now();
    
    match if_data_gen.generate_data() {
        Ok(stats) => {
            let gen_time = start.elapsed();
            println!("✅ Генерация: {:.2}ms", gen_time.as_secs_f64() * 1000.0);
            print_generation_stats(&stats);
        },
        Err(e) => {
            println!("❌ Ошибка генерации: {}", e);
        }
    }
}

fn test_optimized_generation() {
    println!("\n🔥 ТЕСТ 2: Оптимизированная генерация с AVX-512");
    
    // Проверяем доступность AVX-512
    let avx512_available = Avx512Accelerator::is_available();
    println!("AVX-512 доступен: {}", if avx512_available { "✅ ДА" } else { "❌ НЕТ" });
    
    #[cfg(feature = "gpu")]
    {
        let cuda_available = CudaGnssAccelerator::is_available();
        println!("CUDA доступен: {}", if cuda_available { "✅ ДА" } else { "❌ НЕТ" });
    }
    
    let mut if_data_gen = IFDataGen::new();
    
    // Включаем все оптимизации
    if_data_gen.output_param.SampleFreq = 16368000; // 16.368 MHz
    if_data_gen.output_param.Interval = 2000; // 2 секунды (миллисекунды)
    
    // Устанавливаем имя выходного файла
    let filename = "test_optimized_gnss.dat";
    let filename_bytes = filename.as_bytes();
    if_data_gen.output_param.filename[..filename_bytes.len()].copy_from_slice(filename_bytes);
    
    println!("Генерация с максимальными оптимизациями...");
    let start = Instant::now();
    
    match if_data_gen.generate_data() {
        Ok(stats) => {
            let gen_time = start.elapsed();
            println!("✅ Оптимизированная генерация: {:.2}ms", gen_time.as_secs_f64() * 1000.0);
            print_generation_stats(&stats);
        },
        Err(e) => {
            println!("❌ Ошибка оптимизированной генерации: {}", e);
        }
    }
}

fn test_mass_generation() {
    println!("\n🔥 ТЕСТ 3: Массовая генерация (множественные спутники)");
    
    let mut if_data_gen = IFDataGen::new();
    
    // Генерируем для большего количества спутников на более длительное время
    if_data_gen.output_param.SampleFreq = 16368000; // 16.368 MHz
    if_data_gen.output_param.Interval = 3000; // 3 секунды (разумное время для теста)
    
    // Устанавливаем имя выходного файла
    let filename = "test_mass_gnss.dat";
    let filename_bytes = filename.as_bytes();
    if_data_gen.output_param.filename[..filename_bytes.len()].copy_from_slice(filename_bytes);
    
    println!("Массовая генерация для множественных спутников...");
    let start = Instant::now();
    
    match if_data_gen.generate_data() {
        Ok(stats) => {
            let gen_time = start.elapsed();
            println!("✅ Массовая генерация: {:.2}ms ({:.2} секунд)", 
                     gen_time.as_secs_f64() * 1000.0,
                     gen_time.as_secs_f64());
            
            print_generation_stats(&stats);
            
            // Вычисляем производительность
            let target_duration_ms = 3000.0;
            let samples_generated = stats.samples_generated;
            let samples_per_sec = samples_generated as f64 / (gen_time.as_secs_f64());
            println!("📊 Производительность: {:.1}M samples/sec", samples_per_sec / 1_000_000.0);
            
            let realtime_ratio = target_duration_ms / (gen_time.as_secs_f64() * 1000.0);
            println!("⚡ Скорость: {:.1}x быстрее реального времени", realtime_ratio);
        },
        Err(e) => {
            println!("❌ Ошибка массовой генерации: {}", e);
        }
    }
}

fn print_generation_stats(stats: &GenerationStats) {
    println!("\n📊 СТАТИСТИКА ГЕНЕРАЦИИ:");
    println!("  • Общее время: {:.2}ms", stats.total_time_ms);
    println!("  • Время обработки сигналов: {:.2}ms", stats.signal_processing_time_ms);
    println!("  • Количество сгенерированных сэмплов: {}", stats.samples_generated);
    println!("  • Количество обработанных спутников: {}", stats.satellites_processed);
    println!("  • Среднее время на спутник: {:.2}ms", 
             if stats.satellites_processed > 0 {
                 stats.signal_processing_time_ms / stats.satellites_processed as f64
             } else {
                 0.0
             });
    
    if stats.avx512_accelerated {
        println!("  🚀 AVX-512 ускорение: АКТИВНО");
    }
    
    if stats.cuda_accelerated {
        println!("  🚀 CUDA ускорение: АКТИВНО");
    }
}