//! # LDPC (Low-Density Parity-Check) кодирование
//!
//! Этот модуль реализует LDPC кодирование для навигационных сообщений BeiDou.
//! LDPC используется в сигналах B1C, B2a и B2b для исправления ошибок.

/// Таблица экспоненты в значение для GF(64)
#[allow(dead_code)]
const E2V_TABLE: [u32; 128] = [
    1, 2, 4, 8, 16, 32, 3, 6, 12, 24, 48, 35, 5, 10, 20, 40, 19, 38, 15, 30, 60, 59, 53, 41, 17,
    34, 7, 14, 28, 56, 51, 37, 9, 18, 36, 11, 22, 44, 27, 54, 47, 29, 58, 55, 45, 25, 50, 39, 13,
    26, 52, 43, 21, 42, 23, 46, 31, 62, 63, 61, 57, 49, 33, 1, 2, 4, 8, 16, 32, 3, 6, 12, 24, 48,
    35, 5, 10, 20, 40, 19, 38, 15, 30, 60, 59, 53, 41, 17, 34, 7, 14, 28, 56, 51, 37, 9, 18, 36,
    11, 22, 44, 27, 54, 47, 29, 58, 55, 45, 25, 50, 39, 13, 26, 52, 43, 21, 42, 23, 46, 31, 62, 63,
    61, 57, 49, 33, 1, 2,
];

/// Таблица значение в экспоненту для GF(64)
#[allow(dead_code)]
const V2E_TABLE: [u32; 64] = [
    0, 0, 1, 6, 2, 12, 7, 26, 3, 32, 13, 35, 8, 48, 27, 18, 4, 24, 33, 16, 14, 52, 36, 54, 9, 45,
    49, 38, 28, 41, 19, 56, 5, 62, 25, 11, 34, 31, 17, 47, 15, 23, 53, 51, 37, 44, 55, 40, 10, 61,
    46, 30, 50, 22, 39, 43, 29, 60, 42, 21, 20, 59, 57, 58,
];

/// Умножение в поле Галуа GF(64) в представлении BeiDou ICD/SignalSim.
pub(crate) fn gf6_int_mul(a: i32, b: i32) -> i32 {
    if a == 0 || b == 0 {
        0
    } else {
        E2V_TABLE[(V2E_TABLE[a as usize] + V2E_TABLE[b as usize]) as usize] as i32
    }
}

/// LDPC кодирование для символов BeiDou
///
/// # Параметры
/// - `symbol_stream`: массив символов для кодирования (входные данные + место для паритетных битов)
/// - `symbol_length`: длина входных символов
/// - `matrix_gen`: генераторная матрица как строка ASCII символов ('0'-'63')
///
/// # Возвращает
/// 0 в случае успеха, -1 в случае ошибки
pub fn ldpc_encode(symbol_stream: &mut [i32], symbol_length: usize, matrix_gen: &str) -> i32 {
    if matrix_gen.is_empty() || symbol_stream.is_empty() {
        return -1;
    }

    let matrix_bytes = matrix_gen.as_bytes();
    let mut matrix_index = 0;

    // Проверяем, что у нас достаточно места для паритетных битов
    if symbol_stream.len() < symbol_length * 2 {
        return -1;
    }

    // Генерируем паритетные биты
    for i in 0..symbol_length {
        let mut sum = 0;

        for j in 0..symbol_length {
            // Проверяем границы матрицы
            if matrix_index >= matrix_bytes.len() {
                return -1;
            }

            let matrix_element = (matrix_bytes[matrix_index] as i32) - ('0' as i32);
            if !(0..=63).contains(&matrix_element) {
                return -1;
            }

            sum ^= gf6_int_mul(matrix_element, symbol_stream[j]);
            matrix_index += 1;
        }

        // Помещаем паритетный бит после исходных данных
        symbol_stream[symbol_length + i] = sum;
    }

    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gf6_int_mul() {
        // Тест базовых случаев
        assert_eq!(gf6_int_mul(0, 5), 0);
        assert_eq!(gf6_int_mul(5, 0), 0);
        assert_eq!(gf6_int_mul(1, 1), 1);

        // Тест некоторых значений
        assert_eq!(gf6_int_mul(2, 3), 6);
        assert_eq!(gf6_int_mul(7, 9), 63);
        assert_eq!(gf6_int_mul(5, 11), 39);
    }

    #[test]
    fn field_multiplication_matches_polynomial_arithmetic() {
        let base = crate::bcnavbit::BCNavBit::new();
        for a in 0..64 {
            for b in 0..64 {
                let mut x = a;
                let mut y = b;
                let mut expected = 0;
                while y != 0 {
                    if y & 1 != 0 {
                        expected ^= x;
                    }
                    y >>= 1;
                    x <<= 1;
                    if x & 64 != 0 {
                        x ^= 0x43;
                    }
                }
                assert_eq!(gf6_int_mul(a, b), expected);
                assert_eq!(base.gf6_int_mul(a, b), expected);
            }
        }
    }

    #[test]
    fn test_ldpc_encode_basic() {
        let mut symbols = [0i32; 20]; // 10 данных + 10 паритетных

        // Заполняем тестовыми данными
        for i in 0..10 {
            symbols[i] = (i + 1) as i32;
        }

        // Создаем простую матрицу 10x10
        let matrix = "1234567890".repeat(10);

        let result = ldpc_encode(&mut symbols, 10, &matrix);
        assert_eq!(result, 0);

        // Проверяем, что паритетные биты были вычислены
        for i in 10..20 {
            assert_ne!(symbols[i], 0); // Должны быть ненулевыми для ненулевых данных
        }
    }

    #[test]
    fn test_ldpc_encode_error_cases() {
        let mut symbols = [0i32; 10];

        // Тест с пустой матрицей
        assert_eq!(ldpc_encode(&mut symbols, 5, ""), -1);

        // Тест с недостаточным размером массива
        assert_eq!(ldpc_encode(&mut symbols, 6, "123456"), -1);
    }
}
