//! # CRC24Q реализация для навигационных сообщений
//! 
//! Этот модуль реализует CRC24Q (24-битный циклический избыточный код)
//! используемый в GNSS навигационных сообщениях для обнаружения ошибок.

/// CRC24Q таблица для быстрого вычисления
const CRC24Q_TABLE: [u32; 256] = [
    0x00000000u32, 0x01864CFBu32, 0x028AD50Du32, 0x030C99F6u32, 0x0493E6E1u32, 0x0515AA1Au32, 0x061933ECu32, 0x079F7F17u32,
    0x08A18139u32, 0x0927CDC2u32, 0x0A2B5434u32, 0x0BAD18CFu32, 0x0C3267D8u32, 0x0DB42B23u32, 0x0EB8B2D5u32, 0x0F3EFE2Eu32,
    0x10C54E89u32, 0x11430272u32, 0x124F9B84u32, 0x13C9D77Fu32, 0x1456A868u32, 0x15D0E493u32, 0x16DC7D65u32, 0x175A319Eu32,
    0x1864CFB0u32, 0x19E2834Bu32, 0x1AEE1ABDu32, 0x1B685646u32, 0x1CF72951u32, 0x1D7165AAu32, 0x1E7DFC5Cu32, 0x1FFBB0A7u32,
    0x200CD1E9u32, 0x218A9D12u32, 0x228604E4u32, 0x2300481Fu32, 0x249F3708u32, 0x25197BF3u32, 0x2615E205u32, 0x2793AEFEu32,
    0x28AD50D0u32, 0x292B1C2Bu32, 0x2A2785DDu32, 0x2BA1C926u32, 0x2C3EB631u32, 0x2DB8FACAu32, 0x2EB4633Cu32, 0x2F322FC7u32,
    0x30C99F60u32, 0x314FD39Bu32, 0x32434A6Du32, 0x33C50696u32, 0x345A7981u32, 0x35DC357Au32, 0x36D0AC8Cu32, 0x3756E077u32,
    0x38681E59u32, 0x39EE52A2u32, 0x3AE2CB54u32, 0x3B6487AFu32, 0x3CFBF8B8u32, 0x3D7DB443u32, 0x3E712DB5u32, 0x3FF7614Eu32,
    0x4019A3D2u32, 0x419FEF29u32, 0x429376DFu32, 0x43153A24u32, 0x448A4533u32, 0x450C09C8u32, 0x4600903Eu32, 0x4786DCC5u32,
    0x48B822EBu32, 0x493E6E10u32, 0x4A32F7E6u32, 0x4BB4BB1Du32, 0x4C2BC40Au32, 0x4DAD88F1u32, 0x4EA11107u32, 0x4F275DFCu32,
    0x50DCED5Bu32, 0x515AA1A0u32, 0x52563856u32, 0x53D074ADu32, 0x544F0BBAu32, 0x55C94741u32, 0x56C5DEB7u32, 0x5743924Cu32,
    0x587D6C62u32, 0x59FB2099u32, 0x5AF7B96Fu32, 0x5B71F594u32, 0x5CEE8A83u32, 0x5D68C678u32, 0x5E645F8Eu32, 0x5FE21375u32,
    0x6015723Bu32, 0x61933EC0u32, 0x629FA736u32, 0x6319EBCDu32, 0x648694DAu32, 0x6500D821u32, 0x660C41D7u32, 0x678A0D2Cu32,
    0x68B4F302u32, 0x6932BFF9u32, 0x6A3E260Fu32, 0x6BB86AF4u32, 0x6C2715E3u32, 0x6DA15918u32, 0x6EADC0EEu32, 0x6F2B8C15u32,
    0x70D03CB2u32, 0x71567049u32, 0x725AE9BFu32, 0x73DCA544u32, 0x7443DA53u32, 0x75C596A8u32, 0x76C90F5Eu32, 0x774F43A5u32,
    0x7871BD8Bu32, 0x79F7F170u32, 0x7AFB6886u32, 0x7B7D247Du32, 0x7CE25B6Au32, 0x7D641791u32, 0x7E688E67u32, 0x7FEEC29Cu32,
    0x803347A4u32, 0x81B50B5Fu32, 0x82B992A9u32, 0x833FDE52u32, 0x84A0A145u32, 0x8526EDBEu32, 0x862A7448u32, 0x87AC38B3u32,
    0x8892C69Du32, 0x89148A66u32, 0x8A181390u32, 0x8B9E5F6Bu32, 0x8C01207Cu32, 0x8D876C87u32, 0x8E8BF571u32, 0x8F0DB98Au32,
    0x90F6092Du32, 0x917045D6u32, 0x927CDC20u32, 0x93FA90DBu32, 0x9465EFCCu32, 0x95E3A337u32, 0x96EF3AC1u32, 0x9769763Au32,
    0x98578814u32, 0x99D1C4EFu32, 0x9ADD5D19u32, 0x9B5B11E2u32, 0x9CC46EF5u32, 0x9D42220Eu32, 0x9E4EBBF8u32, 0x9FC8F703u32,
    0xA03F964Du32, 0xA1B9DAB6u32, 0xA2B54340u32, 0xA3330FBBu32, 0xA4AC70ACu32, 0xA52A3C57u32, 0xA626A5A1u32, 0xA7A0E95Au32,
    0xA89E1774u32, 0xA9185B8Fu32, 0xAA14C279u32, 0xAB928E82u32, 0xAC0DF195u32, 0xAD8BBD6Eu32, 0xAE872498u32, 0xAF016863u32,
    0xB0FAD8C4u32, 0xB17C943Fu32, 0xB2700DC9u32, 0xB3F64132u32, 0xB4693E25u32, 0xB5EF72DEu32, 0xB6E3EB28u32, 0xB765A7D3u32,
    0xB85B59FDu32, 0xB9DD1506u32, 0xBAD18CF0u32, 0xBB57C00Bu32, 0xBCC8BF1Cu32, 0xBD4EF3E7u32, 0xBE426A11u32, 0xBFC426EAu32,
    0xC02AE476u32, 0xC1ACA88Du32, 0xC2A0317Bu32, 0xC3267D80u32, 0xC4B90297u32, 0xC53F4E6Cu32, 0xC633D79Au32, 0xC7B59B61u32,
    0xC88B654Fu32, 0xC90D29B4u32, 0xCA01B042u32, 0xCB87FCB9u32, 0xCC1883AEu32, 0xCD9ECF55u32, 0xCE9256A3u32, 0xCF141A58u32,
    0xD0EFAAFFu32, 0xD169E604u32, 0xD2657FF2u32, 0xD3E33309u32, 0xD47C4C1Eu32, 0xD5FA00E5u32, 0xD6F69913u32, 0xD770D5E8u32,
    0xD84E2BC6u32, 0xD9C8673Du32, 0xDAC4FECBu32, 0xDB42B230u32, 0xDCDDCD27u32, 0xDD5B81DCu32, 0xDE57182Au32, 0xDFD154D1u32,
    0xE026359Fu32, 0xE1A07964u32, 0xE2ACE092u32, 0xE32AAC69u32, 0xE4B5D37Eu32, 0xE5339F85u32, 0xE63F0673u32, 0xE7B94A88u32,
    0xE887B4A6u32, 0xE901F85Du32, 0xEA0D61ABu32, 0xEB8B2D50u32, 0xEC145247u32, 0xED921EBCu32, 0xEE9E874Au32, 0xEF18CBB1u32,
    0xF0E37B16u32, 0xF16537EDu32, 0xF269AE1Bu32, 0xF3EFE2E0u32, 0xF4709DF7u32, 0xF5F6D10Cu32, 0xF6FA48FAu32, 0xF77C0401u32,
    0xF842FA2Fu32, 0xF9C4B6D4u32, 0xFAC82F22u32, 0xFB4E63D9u32, 0xFCD11CCEu32, 0xFD575035u32, 0xFE5BC9C3u32, 0xFFDD8538u32,
];

/// Вычисляет CRC24Q для массива 32-битных слов
/// 
/// # Параметры
/// - `bit_stream`: массив 32-битных слов с данными
/// - `length_bits`: длина данных в битах
/// 
/// # Возвращает
/// 24-битный CRC код
pub fn crc24q_encode(bit_stream: &[u32], length_bits: usize) -> u32 {
    let byte_count = length_bits.div_ceil(32) * 4;
    let mut crc_result = 0u32;
    let mut data = if !bit_stream.is_empty() { bit_stream[0] } else { 0 };
    
    for i in 0..byte_count {
        crc_result = (crc_result << 8) ^ CRC24Q_TABLE[((data >> 24) ^ (crc_result >> 16)) as u8 as usize];
        data <<= 8;
        
        if (i & 3) == 3 && ((i >> 2) + 1) < bit_stream.len() {
            // Переходим к следующему 32-битному слову
            data = bit_stream[(i >> 2) + 1];
        }
    }
    
    crc_result & 0xFFFFFF // Возвращаем только младшие 24 бита
}

/// Проверяет CRC24Q для полученных данных
/// 
/// # Параметры  
/// - `bit_stream`: массив 32-битных слов с данными и CRC
/// - `data_length_bits`: длина только данных в битах (без CRC)
/// - `received_crc`: полученный CRC код
/// 
/// # Возвращает
/// true если CRC корректен
pub fn crc24q_verify(bit_stream: &[u32], data_length_bits: usize, received_crc: u32) -> bool {
    let calculated_crc = crc24q_encode(bit_stream, data_length_bits);
    calculated_crc == (received_crc & 0xFFFFFF)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc24q_basic() {
        // Тест с известными данными
        let test_data = [0x12345678u32, 0x9ABCDEF0u32];
        let crc = crc24q_encode(&test_data, 64);
        
        // CRC должен быть валидным 24-битным значением
        assert!(crc <= 0xFFFFFF);
        assert!(crc > 0); // Для ненулевых данных CRC не должен быть 0
    }
    
    #[test] 
    fn test_crc24q_zero() {
        // Тест с нулевыми данными
        let test_data = [0u32; 4];
        let crc = crc24q_encode(&test_data, 128);
        assert_eq!(crc, 0); // CRC для нулевых данных должен быть 0
    }
}