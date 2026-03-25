
use gnss_rust::lnavbit::LNavBit;
use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use gnss_rust::types::*;

fn main() {
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx", 10000);
    let target_sow = 381948.0;
    
    for &svid in &[2u8, 7, 8, 10, 13, 15, 23, 27, 30] {
        let eph = nav_data.gps_ephemeris.iter()
            .filter(|e| e.svid == svid && e.valid != 0)
            .min_by(|a, b| (a.toe as f64 - target_sow).abs().partial_cmp(&(b.toe as f64 - target_sow).abs()).unwrap());
        if let Some(eph) = eph {
            let mut lnav = LNavBit::new();
            lnav.set_ephemeris(svid as i32, eph);
            
            // Get frame data for SF1 (tow that gives subframe=1)
            // tow % 5 + 1 = 1 → tow % 5 = 0
            // Start tow = 381948000/6000 = 63658, sf = (63658%5)+1 = 4
            // SF1 at tow 63660 → ms = 63660*6000 = 381960000
            let sf1_time = GnssTime { Week: 2369, MilliSeconds: 381960000, SubMilliSeconds: 0.0 };
            let sf2_time = GnssTime { Week: 2369, MilliSeconds: 381966000, SubMilliSeconds: 0.0 };
            let sf3_time = GnssTime { Week: 2369, MilliSeconds: 381972000, SubMilliSeconds: 0.0 };
            
            let mut bits1 = [0i32; 300];
            let mut bits2 = [0i32; 300];
            let mut bits3 = [0i32; 300];
            lnav.get_frame_data(sf1_time, svid as i32, 0, &mut bits1);
            lnav.get_frame_data(sf2_time, svid as i32, 0, &mut bits2);
            lnav.get_frame_data(sf3_time, svid as i32, 0, &mut bits3);
            
            // Print first 30 bits (TLM word) and word 3 (IODC/WN)
            let tlm: String = bits1[..8].iter().map(|b| if *b != 0 { '1' } else { '0' }).collect();
            
            // Reconstruct word 3 (bits 60-89)
            let mut w3: u32 = 0;
            for i in 0..30 { w3 = (w3 << 1) | (bits1[60+i] as u32 & 1); }
            
            // Extract IODC from SF1
            // Word 3 has WN(10) + L2(2) + URA(4) + Health(6) + IODC_MSB(2)
            // After D30* correction...
            let d30_w2 = bits1[59] as u32 & 1;
            let w3_data = if d30_w2 == 1 { ((w3 >> 6) ^ 0xFFFFFF) & 0xFFFFFF } else { (w3 >> 6) & 0xFFFFFF };
            let iodc_msb = w3_data & 0x3;
            
            // Word 8 has IODC_LSB(8) + toc(16) 
            let mut w8: u32 = 0;
            for i in 0..30 { w8 = (w8 << 1) | (bits1[210+i] as u32 & 1); }
            let d30_w7 = bits1[209] as u32 & 1;
            let w8_data = if d30_w7 == 1 { ((w8 >> 6) ^ 0xFFFFFF) & 0xFFFFFF } else { (w8 >> 6) & 0xFFFFFF };
            let iodc_lsb = (w8_data >> 16) & 0xFF;
            let iodc = (iodc_msb << 8) | iodc_lsb;
            
            // SF2 word 3: IODE
            let mut w3_sf2: u32 = 0;
            for i in 0..30 { w3_sf2 = (w3_sf2 << 1) | (bits2[60+i] as u32 & 1); }
            let d30_w2_sf2 = bits2[59] as u32 & 1;
            let w3_sf2_data = if d30_w2_sf2 == 1 { ((w3_sf2 >> 6) ^ 0xFFFFFF) & 0xFFFFFF } else { (w3_sf2 >> 6) & 0xFFFFFF };
            let iode_sf2 = (w3_sf2_data >> 16) & 0xFF;
            
            // SF3 word 10: IODE
            let mut w10_sf3: u32 = 0;
            for i in 0..30 { w10_sf3 = (w10_sf3 << 1) | (bits3[270+i] as u32 & 1); }
            let d30_w9_sf3 = bits3[269] as u32 & 1;
            let w10_sf3_data = if d30_w9_sf3 == 1 { ((w10_sf3 >> 6) ^ 0xFFFFFF) & 0xFFFFFF } else { (w10_sf3 >> 6) & 0xFFFFFF };
            let iode_sf3 = (w10_sf3_data >> 16) & 0xFF;
            
            let status = if iodc_lsb == iode_sf2 && iode_sf2 == iode_sf3 { "OK" } else { "MISMATCH" };
            
            println!("PRN {:02}: TLM={} IODC={} IODE_SF2={} IODE_SF3={} eph.iode={} eph.iodc={} [{}]",
                svid, tlm, iodc, iode_sf2, iode_sf3, eph.iode, eph.iodc, status);
        }
    }
}
