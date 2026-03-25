use gnss_rust::lnavbit::LNavBit;
use gnss_rust::json_interpreter::{read_nav_file_limited, CNavData};
use gnss_rust::types::*;

fn main() {
    let mut nav_data = CNavData::new();
    read_nav_file_limited(&mut nav_data, "Rinex_Data/BRDC00IGS_R_20251560000_01D_MN.rnx", 10000);
    let target_sow = 381948.0;

    let svids: &[u8] = &[2, 7, 8, 10, 13, 15, 23, 27, 30];

    for &svid in svids {
        let eph = nav_data.gps_ephemeris.iter()
            .filter(|e| e.svid == svid && e.valid != 0)
            .min_by(|a, b| (a.toe as f64 - target_sow).abs().partial_cmp(&(b.toe as f64 - target_sow).abs()).unwrap());

        if let Some(eph) = eph {
            let mut lnav = LNavBit::new();
            lnav.set_ephemeris(svid as i32, eph);

            if let (Some(alpha), Some(beta), Some(utc)) = (
                nav_data.get_gps_iono_alpha(),
                nav_data.get_gps_iono_beta(),
                nav_data.utc_param.as_ref(),
            ) {
                let iono = IonoParam {
                    a0: alpha[0], a1: alpha[1], a2: alpha[2], a3: alpha[3],
                    b0: beta[0], b1: beta[1], b2: beta[2], b3: beta[3],
                    flag: 1,
                };
                lnav.set_iono_utc(&iono, utc);
            }

            // Dump raw stream123 words (before parity, before WN insertion)
            println!("PRN {:02} raw stream words:", svid);
            for sf in 0..3 {
                print!("  SF{}: ", sf + 1);
                for w in 0..8 {
                    print!("{:06X} ", lnav.gps_stream123[(svid - 1) as usize][sf][w] & 0xFFFFFF);
                }
                println!();
            }

            // Also dump with get_frame_data (includes WN, parity)
            for (sf, ms) in [(1, 381960000i32), (2, 381966000), (3, 381972000)] {
                let t = GnssTime { Week: 2369, MilliSeconds: ms, SubMilliSeconds: 0.0 };
                let mut bits = [0i32; 300];
                lnav.get_frame_data(t, svid as i32, 0, &mut bits);

                // Extract 10 x 30-bit words
                print!("  SF{} bits: ", sf);
                for w in 0..10 {
                    let mut word: u32 = 0;
                    for b in 0..30 {
                        word = (word << 1) | (bits[w * 30 + b] as u32 & 1);
                    }
                    print!("{:08X} ", word);
                }
                println!();
            }

            println!("  eph: iode={} iodc={} toe={} toc={} health={} ura={} flag={}",
                eph.iode, eph.iodc, eph.toe, eph.toc, eph.health, eph.ura, eph.flag);
            println!();
        }
    }
}
