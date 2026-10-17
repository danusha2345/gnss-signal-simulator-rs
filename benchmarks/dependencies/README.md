# Микротест RNG и округления f32

```sh
CARGO_TARGET_DIR="$HOME/storage/gnss-rust-dependency-eval" cargo run --release --locked --manifest-path benchmarks/dependencies/Cargo.toml
```

Три повторения, фиксированный seed, 2 млн компонентов. Время включает заполнение
Vec; расчёт статистики и DFT выполняется после замера. Восемь полос DFT: последние
15 частот против 16 в остальных полосах, поэтому сравнивать среднюю энергию на bin.
Это отдельный executable: основной генератор продолжает использовать прежний RNG.
Для измерения throughput приложения необходимо отдельное A/B с versioned noise mode.
