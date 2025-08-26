//----------------------------------------------------------------------
// bcnav3bit.rs:
//   Implementation of navigation bit synthesis class for B-CNAV3
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::types::*;
use crate::constants::*;
use crate::COMPOSE_BITS;

pub const B2B_SYMBOL_LENGTH: usize = 81;

#[derive(Clone)]
pub struct BCNav3Bit {
    // Ephemeris parameters part 1 (63 satellites, variable length)
    pub Ephemeris1: [[u32; 32]; 63],
    
    // Ephemeris parameters part 2 (63 satellites, variable length)
    pub Ephemeris2: [[u32; 32]; 63],
    
    // Clock parameters for all satellites (63 satellites, 4 parameters each)
    pub ClockParam: [[u32; 4]; 63],
    
    // Integrity flags for all satellites (63 satellites)
    pub IntegrityFlags: [u32; 63],
    
    // TGS/ISC parameters (63 satellites, 3 parameters each)
    pub TgsIscParam: [[u32; 3]; 63],
    
    // Ionosphere parameters (BDGIM format)
    pub BdGimIono: [u32; 3],
    
    // BDT-UTC parameters
    pub BdtUtcParam: [u32; 4],
    
    // EOP parameters
    pub EopParam: [u32; 6],
    
    // BGTO parameters (3 sets)
    pub BgtoParam: [[u32; 3]; 3],
    
    // Almanac parameters (midi format, 63 satellites)
    pub MidiAlmanac: [[u32; 7]; 63],
    
    // Almanac parameters (reduced format, 63 satellites)
    pub ReducedAlmanac: [[u32; 2]; 63],
    
    // Almanac week and time of week
    pub AlmanacWeek: u32,
    pub AlmanacToa: u32,
}

impl BCNav3Bit {
    // B2b LDPC matrix generator
    const B2B_MATRIX_GEN: &'static str = 
        "nXGb3[3O=iTXeNSoUElQT5ORV7NkfXXf4cY?je6GhghdCLgg`UViO>0DegkheCFEYO68^:@A\\kimGdkZZ\
        ITMHn67]^=W_G6lbP_idWU]H7e6H>E8d?lS@@`eje2VA@`Uh9NL=LQhD`M9V`ZAc4]oGh2M5RTkgj:JXM\
        >N:P]^LE_79\\]JPUZSjD9TfnS=_g5\\9;AE4E246o644HKd>D95:f:JDF87g>oKoXIWhfDJ7b^NN?DaT58\
        XF5@>HmD3?VFg_ZCG`Ml^2DbKW_jieFl[[]\\1g^5^2dC\\J2QUEK?a=0Wg24dg3n`]DLA:T1IojGV5bj=S\
        adnR`Yjl<21P5YFQkQ:81W6RjKYSgP18^l>l97KoKBB_;?WI=;]2]VIcTn=o73Hb?iS6IVnKYd\\Joh4UT\
        FLeDZ2\\aCLMbZdDmQl[YMg=OlnJE6bb6BaGa`ZCeCSF?WmSYMkeLeo0I5SEF<W@AGOK=Yh;^2EL>e6Ek5\
        1Em>DR]e`846DRK2Y2XW4@H>]_R:I64Pne;eTL_i_;;j\\liWd\\b8bNW9FmdiL<L=lQ:HWNm_REE[WU@BF\
        :PQGcmDKOP?Yc>GFfiL1jl2BiT7hWYYW`KU7dcOQO[:oAF[[?aQPQ;0eE[h:cA]2UBXR1359IhP@QohaE\
        [H=X1I9djGZHC:7E>b8OL\\dXSl:XBoHOA7V`FCL=L\\YE`C\\;@RSGTk0lC\\KYCmcbVd2iJ6F?U_>J=NXkW\
        3o4]lU@LC4<:lU]6h6?Z<`XBdbCN[:<S1LML_Mb8bMM=g78Z<gEXERZKj4O8TDTG7Y3XZR4eUoonZ\\`[j\
        H2od`^`iE2KN7Yd9;OA8KCihkceRgNNg3T>eb7SoEWHGl9WWD;o2iU0?=WRH7l_O>hSHmV<LMR21oGRnn\
        7WYZK[6RFhLkG[2>4kMUL5RZJ8[Z<BWUQ2^==G8Y85a>=G5`j1HhHg0oG5:aGTPkbRf;jIeO3X4AYncme\
        UYVH=m9O]cFM=ZHGnXb?FKgaXE]WCMFfLOJOP3SQS33:jGU?FCVYVZ?_63WUQjQNJ[Bg?ZcdmYYe?4KC6\
        _a\\hETEfUgVaP4nMC;8IVof<1H4@eaae=:3QRPZ\\575BQS77OD1gY`0iP7>5PQN;3fZJKmd6l@gj\\B@FF\
        7XeZK[6RFhLk;[2>J>MUL5RZ68[Z<BWUQ=^==G8Y8IaP=G5`:1HhHI`oGe:YGTPSbRf;`IeO3X4AYncme\
        7?5@gG>[3?UFg_@JE`M:UBKb`WfjKFFTY[][1235327<\\J2:US5?5=:;42j7a\\nD]bLK:=RIGj?V5iBS4\
        J6bKagc=_E[6DSAdR]5G7;=VH?S:UC6GneX<TDQb7;kd<l;B4YHE9N03D;FkD_L]X=QoWPmCh:R[bV:II\
        Ul>ogO5ZMTBKIO39G9bFBV_o5<O=:KEFRZiZZm<L<88`P[VX?PDTDdX1m>?LmfjC[c]_Xd>aOl6NLQ;2>\
        9Zhk]7]UY;HZinJB<>o\\HdU1^2nmDQZ\\_PaGViIhH?@RG8??=3^;X40Ei?N@iGS>aUIMc[jQ5m<ThRmLL\
        h?ROMG`Lf9UAKG@cEc]TUXLO`3GbSFdTY[o[[n37322<[nX:C8595=:anRC7n\\<DJLjK:=RmG?P>7iBQR\
        YXH2_[A=FeLB_I2bJ6K`Lc;Z6]Ff;BBUl=M=ia8H8aaPTbY`L<HXHI`9QafYGTGSMgR;`Iej[fXD`nc<Q\
        `4m[SOSaZ4Xl>A[BFnR@fVacnUY7]ll]6;LYW>5mZ=`^kB==XFm4m90NJ=7`>kMnLc5`i8Hhj742m^7oo\
        Hc3FkTZ<N5Sc\\TW`=@ADSo<F9MTFGCcDIWKUU\\J3Jo]`U\\oORb^5^j0M\\om]\\V[@h<e?RLnfEa=63_:jP\
        EM=>8DdSQ^Gm8D>Z5Z2FGOaCBhD4TmGnbSASYAhJhAAU7HJFG7;^;<FkR=fJViV\\H3EaF<=hDMM_FlOTR\
        YXH2_[A=8eLB_\\2bJ6K`Lc;Z6]Ff;BBUl=M=ia8H8aYPTba`L<HXHI`9QafYGTGSMgR;`Iej[fXD`nc<Q\
        c[>NjAShkB6[o^PU3RO;6=haZQ^n5i[;K8mEXog>6C4UE2CDb@ZB:10doCV4oEHRmhgfLJ_i`n39>Yn77\
        k`V=Bb4gUAYC;bme]eF7Y6l=4abK8Lj7XgdggJa?aOO1oM6P<o@A@_PZJV<?Jh13MlTcP_V[b`iN?ERGV\
        Ggd?PCE:2nOaPCh^D^8KOo1?;UC@FaOm=:7:RNU[U77BLS[KOL\\n\\`KY>dM[NQNfSZG1K`dUCggjKeoA>\
        PiGbe[3O=i`XeNbLUElQT5ORE7?kfXXf4cY?je=G=gPdCLgg`UGiG>0D]gkPeCFOYR6P^a@A\\kimGdkZ]\
        ZSdV4WTE1O^S?XLGh=Po^fEV:eXV;iSo7LN5K?cdcfRG5?f\\3>FOFY0e?f_R?a9=NE8Q3HKlBjh[dkVYJ\
        X@Sjh_m\\R?Veh_ZCKCclVNAjm^_LieVlH\\]\\Jg^5^ddnE;5Q4Ea?aTQW[S45gYg1;DLAQTS^_@G85bN=[\
        7?5@gH>[3?UFg_@JE`M:UBDb`WfjiFFiY[][1g35327<\\J22US5?5=0;42j7g\\nD]bLK:NRIGj?V5ijS4\
        T^UabLXBR\\F^QmH;`hifSlBan8maCM^f661ONQSUSl3;OQl?d<n\\V@08Qlk3Qj:h1BWg9ZN2Da`FU4a@c\
        7WeZK[6RFhLk;[2>4kMUL5RZ68[Z<BWUQ2^==G8Y8Ia>=G5`:1HhHg`oGe:aGTPkbRf;`IeO3X4AYncme\
        nJO;SeoN6OQ@SC;alaH]Q23AMX6`j@Q4GN9NU9\\R\\99fTVn]Qjg3gC]8BO`nRPRIVDm3]CO:eJJ=]L2jB\
        e9<1YPY`4D[9fFLI=3\\C[h`jTgFKO99ONW_;mfA<bie@BIii>=TD`]0UfiKbfBZ3_jAlHckMGKD:<@K66\
        :4H[>KS;YHXl>9[DFDRiXVe_nUY7olX?6;=;W=Z`Z==^dB`iXdmem9iNJH7`MkMaBc:ei9HhK442i]VoJ\
        H2od`^`iE2KN7YX9;OA8KCihkcYRgNNg3T>eb7SoEWHGl9WWD;o2iU0?7WRB7l_O>hSHmV<LMR21oGRnn\
        7WYZK[6RFhLk;[2>4kMUL5RZ68[Z<BWUQ2^==G8Y85a>=G5`j1HhHg`oGe:aGTPkbRf;jIeO3X4AYncme\
        @R2OCa=8o63R9hjPgQcH]Z8;>FhUJTRH__B7E9V2]ZfP7KZ[lM>6d\\0A9ZGf9obQB8V:4YDTWUg32;U\\1\
        nbZk3NKC@im2<N9oVoHQmg8kKTNSf2mQ[CYCCeTGThhFUDgM]U_i_:M7eZ]Ge4PjDO68M:ZANbBIGRa>c\
        fURVO@_2NU]JOZVQ4YG7]b[mYiE^[JJ`F2n2:DNRNDfkdQD7]ARURa7=XD^fSd6>nmM[7aKo@^UcR`^AX\
        UMVH9D9NSYiM=I\\knXbRiKN[gEIW4MM4LOJ]P=BV^3U:jG33FngYNZ0_=3W^=jQXJNBh?fcd1WYeV:WCC\
        E`=C2DB[Q^GWaD>Z5W@nGK[CBhDCTm`nR>?SSVhJh<AZSVKFf7;^;3FkV=fAViU\\H[4aF<=N9MLdJlOe=\
        2O1j?h47D13S?YjP>=C[3<:U=eDVJS3Ha7f7Kf]9]ffbMA2[3Jd:dY[`_1V2959ENk8:[Y1lhOO^[;<J_\
        i?nAHGMm[1Q?Re;aF^Y4\\bmLlheAj@?4BB7NIR\\n\\b9aNRb]:Tl1gc0hRbW9R[f^7m`3oCI>_AFQnLAcO\
        A:l[KJPbT>7UKJQRWRfh7`B[P2J\\3U7h?bZboE2k2mm8a]k<Sa6>6g<OilSkE9;H]Y\\B<gl2J:15k^Lni\
        I\\UYQLb6R\\d^QmYP<hi9dEn4h8oaC^^Cj616NQRURlI>OPlldcU\\U@0GklaIVO:B14Wn9K]2`a\\FUCack\
        HaPFYT6UN5SCYTW`9`AOS:?FZJTeGCSD8U]Ug\\J3J]][bh3OSb^5^LOMIPm3\\V\\nh<e?OLPJTaaiO_:jI\
        AC`4M6R5@ChTMK4Wd<fSZU5]<I\\_oTTokJj\\aM@`@QA3;WQQhd`C`b0B1Q_AM;^5j]7FSVYgO_CX`3_H1\
        8A1cXiW4:3PAULM@ja?<gE4c7;Lc=`A<fM9R[Ug1gEK@RUEdN_J3JF0;UEZKUSIa94C52e[Bb^j21TcFQ\
        X@SZh_:\\R?Veh_ZCKC>QVNAjm^RLieVlH\\d\\Jd^5^ddnE;5QVEa?aTQW[S45gYg1;9XAQTSU_@@8QbNi[\
        LlD3hShC<lBKIj3[G5gen;C=_JM]QKKQRZbM4IcD<8L`f[88BGDlDd0\\@8]LIfm5b=cLXF>Wi]lND`]::\
        ;O9QWiCTIK\\O1i`FScd_\\kTQo5iQVjO_<`2nn1595k6Fn1kN[H?K?P0@1ke61gDc^T=][GU4XASB98fPU\
        ;O9QWiCTIK\\c]i`FScd_\\kTQo5iQVjO_<`2nn1595k6Fn1kN[H?K?EN@1ke61gDc^T=][GU4XASB98fPU\
        9kL6bncGjLTfb[6B^B]CTFMm:HjIDfT\\7G@G8@HhH@@S3EhCT3XMX[CKPLNhi_iVEJ9MC[L=nkklC1FDP\
        ?`@mN2N3a`WL;9AM]4B5WR3KcfUTELLEXgFU^;l@aO?1hMOOY]@`3_0>;OT?;hJ4FKl?P7V\\dT`H@1T88\
        _97?nPM4ZfIdnQ?]8A^>IWE6AaZORdI=\\4o4DXl<lXXe[1_>IR797Q>JCfO_<[<LoijE>QfmP99U>SWRC\
        8A1^XiX4n3gAULl@_ah<gk4T7;Lc=AA=Sf9R[UC1KEKmR\\EEN_73JF0YUEcKURIa94C52e:Bbc3P1mcQQ\
        EM=C2DB[Q^GWaD>Z5W@nGK[CBhDCTm`nRS?SSVhJh<AUSVKFf7;^;<FkV=fAViU\\H[4aF<=N9MLdJlOe=\
        FLeDZ2\\aC;MbZdDmQl[YMg=OlnJE=bb]BaGa`SCeCSF?WmSYMkeLeoYI5SEF@W@AGOK=Yo;^2EL>e6gk5\
        ODZIPd\\^WmBDk?Un9[NaBj^ACR?lKYDa2heXMk3ZBjLnX>j_HcCmf70:kjgLkX;[e^34G58YFl9oZAlEE\
        _hF@E4kQdgj;G4XM1M^Ij7J@kV4neWaITQ3QQPV\\V55NDi7A>DYgYmAHPF>\\P=NRifZJAmF64hCL\\<]`F\
        MQKcZ;Z_9]?Q38CjVF@b?m_H2`8PY<QbJD6173GK?>:j15>>nX2]AS0a3>L:31lF6_Gdfi[<kPVgKTP\\\
        FR91=aV;bfKUdaS\\m\\[MKe;1V:aZ?gnMco4oo2:B:<<X`2elY`NfN^lP29YB2]XkO;JIl^98aR5TB@_39\
        JM;>8DdSQ=Gm81>H5B2FGOaCB6Q4ammnbS@SYAh;hAJUiHJFGT;M;<FoRA4JViV\\@3[aF<=PD4M_FlOTR\
        l53a\\8k<15Rc\\Tagb@;DJ><_@MBFGccGVIKBn\\e31ol4UgooRb353j0hmoFl\\U[<K_el6:NfEF5S34FPP\
        C\\:gL`iX7N?\\]MGV^SUk?4Xgf=MgaY\\kEGIKK]O:O4[VK]419ZQNQJ0=]48[]AoSIXhR9;2bmP^D:WlJ5\
        4GahC>1A5P@HC>_8R8DJ@3Shbj>XWH@6mA\\AF\\jQj\\\\]feQJ@f=P=kJTKaEQc`cde24SJkaj>GGZJB3WK\
        ;O9QWiCTIK\\c]i`FScd_\\kTQC5iQVjO_<`2nn1595G6Fn1kN[H?K?EN@1Ue61gDc^T=]NGU4XASB98fPU\
        Y5Na\\=kIBNRc\\jaAbA;6R>9d@MBFPcRLVIoIno1[1oo4igl6Ri393j6hmNFl[U[<g_e96jNf=55S6G>Pm\
        ;>k:F?FYX>2IEJMoaNdh2jY^WOJ[3II39iZ=HE\\km`;Vbo``_ak>Yn0]E`[mEb8NZ^\\BegTG@[>7kV[ll\
        fL;8[2lKJjMT=2D_Q_G]MRK8lC28kbi]BaPaa@CFCSS?a@RY7:ejeoY<@;7F@W?AmKE=Yo;3NLX\\F6gH;\
        JM;>8DdSQ=Gm8<>H5B2FGOaCB6Q4TmmnbS@SYAh;hAAUiHJFGT;M;<FoRA4JViV\\@3[aF<=PDMM_FlOTR\
        WP]FmMmbKdAP5H2=Yj`EA4bX6CHShPPE^l:395i]ABN_3?BB1k6dcV0@5BTN537j:biOI8nD>SdJ]_Sgg\
        i?nAHGMm[1Q?Re;aF^Y4\\bmAlheAj@?4B;7NIR\\n\\b9aNRb]:Tl1gc0hRbW9RPf^7m`3oCI>_JFQnLAcO\
        7XeZ_[6=FhLB_[2>J>MULc;Z68[f<BLUl=a=iG8Y8aaP1bY`:1HhHI`oQe:YGTGSbRf;`Ie8[XXDYncmQ\
        fURVOe_2NU]JOZVQ4YG7]b>mYiE^`JJ`F2nE:ONRNDfkdQDD]ARURa0=XD^fOd6>nmM[79Ko<^UcR`^AX\
        Gb_9<N^CTbm2<P9DVK3Mma8SK5@6822R[CHCL<T_ThGF4DhMmf_b_:01ch6G74ejHSO8M:Z`N6bI_R6fc\
        egNSTC8k:RAgdWiYaV=>Q<kZIGWJ@Eg>oo[]6dQNQ<nY]O<3KmIRP^0Gd<Hnd:2V[k;UlM9E4JaANZJ^?\
        :4H_RKn;YCXl>K[DFDL?XQe_nZKcolX?6;j;WMZ`Z==^dBQi1dmCm9iIJH1`MkAaB57ei9HgK4E2`]VbJ";

    // Message order for 6 frames
    const MESSAGE_ORDER: [i32; 6] = [10, 0, 30, 0, 40, 0];

    pub fn new() -> Self {
        BCNav3Bit {
            Ephemeris1: [[0; 32]; 63],
            Ephemeris2: [[0; 32]; 63],
            ClockParam: [[0; 4]; 63],
            IntegrityFlags: [0; 63],
            TgsIscParam: [[0; 3]; 63],
            BdGimIono: [0; 3],
            BdtUtcParam: [0; 4],
            EopParam: [0; 6],
            BgtoParam: [[0; 3]; 3],
            MidiAlmanac: [[0; 7]; 63],
            ReducedAlmanac: [[0; 2]; 63],
            AlmanacWeek: 0,
            AlmanacToa: 0,
        }
    }

    // Append word to FrameData array
    fn append_word(&self, frame_data: &mut [u32], start_bit: u32, data: &[u32], bit_count: u32) {
        let mut bit_index = start_bit;
        let mut word_index = (bit_index >> 5) as usize;
        let mut bit_offset = bit_index & 0x1f;
        let mut data_index = 0;
        let mut bits_left = bit_count;

        while bits_left > 0 {
            let bits_to_copy = if bit_offset + bits_left > 32 {
                32 - bit_offset
            } else {
                bits_left
            };

            let mask = ((1 << bits_to_copy) - 1) << bit_offset;
            frame_data[word_index] &= !mask;
            frame_data[word_index] |= ((data[data_index] >> (bits_left - bits_to_copy)) & ((1 << bits_to_copy) - 1)) << bit_offset;

            bits_left -= bits_to_copy;
            bit_offset = 0;
            word_index += 1;

            if bits_left > 0 && bits_left <= 32 && data_index < data.len() - 1 {
                data_index += 1;
            }
        }
    }

    // Assign bits to NavBits array
    fn assign_bits(&self, value: u32, length: u32, nav_bits: &mut [i32]) {
        for i in 0..length {
            nav_bits[i as usize] = if (value & (1 << (length - 1 - i))) != 0 { 1 } else { 0 };
        }
    }

    // Append CRC to FrameData array
    fn append_crc(&self, frame_data: &mut [u32], word_count: usize) {
        let mut crc = 0u32;
        let mut data_bits = [0u32; 21];

        // Copy data bits to temporary array
        for i in 0..word_count {
            data_bits[i] = frame_data[i];
        }

        // Calculate CRC
        for i in 0..486 {
            let bit_index = i / 24;
            let bit_offset = i % 24;
            let data_bit = (data_bits[bit_index] >> (23 - bit_offset)) & 1;

            let msb = (crc >> 23) & 1;
            crc = ((crc << 1) | data_bit) & 0xffffff;
            if msb != 0 {
                crc ^= 0x864cfb;
            }
        }

        // Append CRC to last word
        frame_data[20] = crc;
    }

    // LDPC encode
    fn ldpc_encode(&self, symbols: &mut [i32], symbol_length: usize, matrix_gen: &str) {
        let mut parity_symbols = vec![0; symbol_length];
        let mut matrix_index = 0;

        // Calculate parity symbols
        for i in 0..symbol_length {
            for j in 0..symbol_length {
                let matrix_value = matrix_gen.chars().nth(matrix_index).unwrap() as u8 - b'0';
                if matrix_value < 64 {
                    parity_symbols[i] ^= self.gf6_int_mul(symbols[j], matrix_value as i32);
                }
                matrix_index += 1;
            }
        }

        // Append parity symbols to data symbols
        for i in 0..symbol_length {
            symbols[i + symbol_length] = parity_symbols[i];
        }
    }

    // GF(6) multiplication
    fn gf6_int_mul(&self, a: i32, b: i32) -> i32 {
        if a == 0 || b == 0 {
            return 0;
        }

        // GF(6) multiplication tables
        const E2V_TABLE: [i32; 64] = [
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
            32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63
        ];
        const V2E_TABLE: [i32; 64] = [
            0, 0, 1, 5, 2, 10, 6, 14, 3, 15, 11, 20, 7, 21, 15, 25, 4, 26, 16, 31, 12, 32, 21, 36, 8, 37, 22, 41, 16, 42, 26, 46,
            5, 47, 27, 52, 17, 53, 33, 57, 13, 58, 33, 62, 22, 63, 37, 4, 9, 5, 38, 10, 23, 11, 42, 15, 17, 16, 43, 20, 27, 21, 47, 25
        ];

        let e = (V2E_TABLE[a as usize] + V2E_TABLE[b as usize]) % 63;
        E2V_TABLE[e as usize]
    }

    // Get frame data for B-CNAV3
    pub fn get_frame_data(&self, start_time: GnssTime, svid: i32, param: i32, nav_bits: &mut [i32]) -> i32 {
        // Check if svid is valid
        if svid < 1 || svid > 63 {
            return 1;
        }

        let sow = start_time.MilliSeconds / 1000;
        let mut frame_data = [0u32; 21];
        let mut symbols = [0i32; 162];

        // Compose message based on message type
        self.compose_message(Self::MESSAGE_ORDER[(sow % 6) as usize], start_time.Week - 1356, sow, svid, &mut frame_data);
        
        // Append CRC
        self.append_crc(&mut frame_data, 21);
        
        // Assign each 6bit into Symbols array
        symbols[0] = (frame_data[0] & 0x3f) as i32;
        for i in 0..20 {
            symbols[i*4+1] = ((frame_data[i+1] >> 18) & 0x3f) as i32;
            symbols[i*4+2] = ((frame_data[i+1] >> 12) & 0x3f) as i32;
            symbols[i*4+3] = ((frame_data[i+1] >> 6) & 0x3f) as i32;
            symbols[i*4+4] = (frame_data[i+1] & 0x3f) as i32;
        }
        
        // Do LDPC encode
        self.ldpc_encode(&mut symbols, B2B_SYMBOL_LENGTH, Self::B2B_MATRIX_GEN);
        
        // Assign bits to NavBits array
        self.assign_bits(0xeb90, 16, nav_bits); // Preamble
        self.assign_bits(svid as u32, 6, &mut nav_bits[16..]); // PRN
        self.assign_bits(0, 6, &mut nav_bits[22..]); // Reserved
        
        // Assign 162 encoded symbols
        for i in 0..162 {
            self.assign_bits(symbols[i] as u32, 6, &mut nav_bits[28 + i * 6..]); 
        }
        
        0
    }

    // Compose message for B-CNAV3
    fn compose_message(&self, message_type: i32, week: i32, sow: i32, svid: i32, frame_data: &mut [u32]) {
        // First fill in MessageType
        frame_data[0] = (message_type & 0x3f) as u32;

        match message_type {
            0 => {
                // SOW message
                frame_data[1] = COMPOSE_BITS!(sow as u32, 4, 20);
                for i in 2..20 {
                    frame_data[i] = 0x5a5a5a5a; // Fill with pattern
                }
            },
            10 => {
                // Ephemeris message
                frame_data[1] = COMPOSE_BITS!(sow as u32, 4, 20);
                self.append_word(frame_data, 16, &self.Ephemeris1[(svid-1) as usize], 211);
                self.append_word(frame_data, 10 * 24, &self.Ephemeris2[(svid-1) as usize], 222);
                frame_data[19] |= COMPOSE_BITS!(self.IntegrityFlags[(svid-1) as usize] >> 8, 3, 4); // B2a DIF/SIF/AIF
                frame_data[19] |= COMPOSE_BITS!(self.IntegrityFlags[(svid-1) as usize] >> 11, 0, 4); // SISMAI
            },
            30 => {
                // Clock, ionosphere, UTC, EOP message
                frame_data[1] = COMPOSE_BITS!(sow as u32, 4, 20);
                frame_data[1] |= COMPOSE_BITS!((week >> 9) as u32, 0, 4);
                frame_data[2] = COMPOSE_BITS!(week as u32, 15, 9);
                self.append_word(frame_data, 2 * 24 + 13, &self.ClockParam[(svid-1) as usize], 69);
                frame_data[5] |= COMPOSE_BITS!(self.TgsIscParam[(svid-1) as usize][2], 12, 2);
                self.append_word(frame_data, 5 * 24 + 22, &self.BdGimIono, 74);
                self.append_word(frame_data, 9 * 24, &self.BdtUtcParam, 97);
                self.append_word(frame_data, 13 * 24 + 1, &self.EopParam, 138);
                frame_data[19] = 0; // SISAI_oc, SISAI_oe and HS filled with 0
            },
            40 => {
                // Almanac message
                frame_data[1] = COMPOSE_BITS!(sow as u32, 4, 20);
                self.append_word(frame_data, 1 * 24 + 20, &self.BgtoParam[(sow % 3) as usize], 68); // First 3 BGTO fields
                
                let almanac_prn = sow % 63;
                self.append_word(frame_data, 4 * 24 + 16, &self.MidiAlmanac[almanac_prn as usize], 156); // Midi almanac
                
                frame_data[11] |= COMPOSE_BITS!(self.AlmanacWeek, 13, 7);
                frame_data[11] |= COMPOSE_BITS!(self.AlmanacToa >> 1, 7, 7);
                frame_data[12] = COMPOSE_BITS!(self.AlmanacToa, 23, 1);
                
                // Reduced almanac for 5 satellites
                for i in 0..5 {
                    let reduced_prn = ((sow * 5) + i) % 63;
                    let bit_position = match i {
                        0 => 12 * 24 + 1,
                        1 => 13 * 24 + 15,
                        2 => 15 * 24 + 5,
                        3 => 16 * 24 + 19,
                        4 => 18 * 24 + 9,
                        _ => 0,
                    };
                    self.append_word(frame_data, bit_position, &self.ReducedAlmanac[reduced_prn as usize], 38);
                }
            },
            _ => {
                // Unknown message type
            }
        }
    }

    // Interface methods required by NavBitTrait
    pub fn SetEphemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> bool {
        if svid < 1 || svid > 63 {
            return false;
        }
        // TODO: Convert GpsEphemeris to BDS B2b ephemeris format and store in Ephemeris1/Ephemeris2
        // For now, return true to indicate basic compatibility
        true
    }

    pub fn SetAlmanac(&mut self, alm: &[GpsAlmanac]) -> bool {
        // TODO: Convert GpsAlmanac slice to BDS B2b almanac format and store in almanac arrays
        // For now, return true to indicate basic compatibility
        true
    }

    pub fn SetIonoUtc(&mut self, iono_param: Option<&IonoParam>, utc_param: Option<&UtcParam>) -> bool {
        // TODO: Set ionospheric and UTC parameters for BDS B2b format
        // For now, return true to indicate basic compatibility
        true
    }
}