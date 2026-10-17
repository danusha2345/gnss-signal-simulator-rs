//! # BeiDou B3I Навигационные сообщения
//!
//! Этот модуль реализует синтез навигационных битов для BeiDou B3I (B-CNAV3 - B3I Civil Navigation).
//!
//! ## Назначение
//! B3I - это гражданский сигнал BeiDou-3, передаваемый на частоте B3I (1268.52 МГц).
//! Предназначен для региональных применений и обеспечения дополнительного частотного
//! диапазона для повышения точности позиционирования в азиатско-тихоокеанском регионе.
//!
//! ## Основные функции модуля
//! - Генерация сообщений B-CNAV3 с эфемеридной информацией
//! - Формирование сообщений с расширенными региональными данными
//! - Кодирование параметров ионосферы, специфичных для региона
//! - Поддержка региональных дифференциальных сервисов
//! - Формирование сообщений интегритета для критических применений
//!
//! B3I использует структуру символов длиной 81 символ с оптимизацией для региональных
//! сервисов и улучшенного покрытия в азиатско-тихоокеанской зоне.

//----------------------------------------------------------------------
// bcnav3bit.rs:
//   Implementation of navigation bit synthesis class for B-CNAV3
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

use crate::bcnavbit::BCNavBit;
use crate::types::*;
use crate::COMPOSE_BITS;

pub const B2B_SYMBOL_LENGTH: usize = 81;

#[derive(Clone)]
pub struct BCNav3Bit {
    // Ephemeris parameters part 1 (63 satellites, variable length)
    pub ephemeris1: [[u32; 32]; 63],

    // Ephemeris parameters part 2 (63 satellites, variable length)
    pub ephemeris2: [[u32; 32]; 63],

    // Clock parameters for all satellites (63 satellites, 4 parameters each)
    pub clock_param: [[u32; 4]; 63],

    // Integrity flags for all satellites (63 satellites)
    pub integrity_flags: [u32; 63],

    // TGS/ISC parameters (63 satellites, 3 parameters each)
    pub tgs_isc_param: [[u32; 3]; 63],

    // Ionosphere parameters (BDGIM format)
    pub bd_gim_iono: [u32; 3],

    // BDT-UTC parameters
    pub bdt_utc_param: [u32; 4],

    // EOP parameters
    pub eop_param: [u32; 6],

    // BGTO parameters (3 sets)
    pub bgto_param: [[u32; 3]; 3],

    // Almanac parameters (midi format, 63 satellites)
    pub midi_almanac: [[u32; 7]; 63],

    // Almanac parameters (reduced format, 63 satellites)
    pub reduced_almanac: [[u32; 2]; 63],

    // Almanac week and time of week
    pub almanac_week: u32,
    pub almanac_toa: u32,
}

impl Default for BCNav3Bit {
    fn default() -> Self {
        Self::new()
    }
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
            ephemeris1: [[0; 32]; 63],
            ephemeris2: [[0; 32]; 63],
            clock_param: [[0; 4]; 63],
            integrity_flags: [0; 63],
            tgs_isc_param: [[0; 3]; 63],
            bd_gim_iono: [0; 3],
            bdt_utc_param: [0; 4],
            eop_param: [0; 6],
            bgto_param: [[0; 3]; 3],
            midi_almanac: [[0; 7]; 63],
            reduced_almanac: [[0; 2]; 63],
            almanac_week: 0,
            almanac_toa: 0,
        }
    }

    // Append word to FrameData array
    fn append_word(&self, frame_data: &mut [u32], start_bit: u32, data: &[u32], bit_count: u32) {
        // Ported from the verified BCNavBit::append_word (bcnavbit.rs). B-CNAV frames are packed
        // as 24-bit words MSB-first — the same convention used by the symbol extraction and CRC
        // in get_frame_data. The previous version addressed 32-bit words from the LSB (>> 5,
        // & 0x1f), which is incompatible with that 24-bit layout and corrupted every field.
        let mut remain_bits: i32 = 24; // bits left in the current source word
        let mut dest_idx = (start_bit / 24) as usize;
        let mut start_bit = (start_bit % 24) as i32; // bit offset inside the current dest word
        let mut src_idx = 0usize;
        let mut length = bit_count as i32;

        while length > 0 && dest_idx < frame_data.len() && src_idx < data.len() {
            let mut fill_bits = 24 - start_bit;
            if fill_bits > remain_bits {
                fill_bits = remain_bits;
            }
            if fill_bits > length {
                fill_bits = length;
            }
            frame_data[dest_idx] = if start_bit == 0 {
                0
            } else {
                frame_data[dest_idx]
            } | COMPOSE_BITS!(
                data[src_idx] >> (remain_bits - fill_bits),
                (24 - start_bit - fill_bits) as u32,
                fill_bits as u32
            );
            start_bit += fill_bits;
            if start_bit >= 24 {
                start_bit = 0;
                dest_idx += 1;
            }
            remain_bits -= fill_bits;
            if remain_bits <= 0 {
                remain_bits = 24;
                src_idx += 1;
            }
            length -= fill_bits;
        }
    }

    // Assign bits to NavBits array
    fn assign_bits(&self, value: u32, length: u32, nav_bits: &mut [i32]) {
        for i in 0..length {
            nav_bits[i as usize] = if (value & (1 << (length - 1 - i))) != 0 {
                1
            } else {
                0
            };
        }
    }

    // Append CRC to FrameData array
    fn append_crc(&self, frame_data: &mut [u32], _word_count: usize) {
        // CRC24 (poly 0x864cfb) over the 486-bit message stream EXACTLY as the symbol extraction
        // builds it: symbol 0 is the 6-bit MesType in frame_data[0]'s low bits, symbols 1..80 are
        // frame_data[1..21] as 24-bit MSB-first words. The previous version read frame_data[0] as
        // a full 24-bit word, prepending 18 spurious zero bits and misaligning the entire CRC.
        // frame_data[20] (the 24-bit CRC field) is still zero here, serving as the trailing zeros.
        let mut crc = 0u32;
        let mut push = |bit: u32| {
            let msb = (crc >> 23) & 1;
            crc = ((crc << 1) | (bit & 1)) & 0xffffff;
            if msb != 0 {
                crc ^= 0x864cfb;
            }
        };

        // Symbol 0: MesType, 6 bits MSB-first.
        for k in (0..6).rev() {
            push(frame_data[0] >> k);
        }
        // Symbols 1..80: frame_data[1..21], 24 bits MSB-first each (the last word is the
        // still-zero CRC field, which supplies the 24 trailing zeros of the CRC computation).
        for word in frame_data.iter().take(21).skip(1) {
            for k in (0..24).rev() {
                push(word >> k);
            }
        }

        frame_data[20] = crc;
    }

    // LDPC encode
    fn ldpc_encode(&self, symbols: &mut [i32], symbol_length: usize, matrix_gen: &str) {
        let mut parity_symbols = vec![0; symbol_length];
        let bytes = matrix_gen.as_bytes();
        let mut matrix_index = 0usize;

        // Calculate parity symbols (защита от выхода за пределы матрицы)
        for parity_row in parity_symbols.iter_mut().take(symbol_length) {
            for sym in symbols.iter().take(symbol_length) {
                if matrix_index >= bytes.len() {
                    break;
                }
                let matrix_value = bytes[matrix_index].saturating_sub(b'0');
                if matrix_value < 64 {
                    *parity_row ^= self.gf6_int_mul(*sym, matrix_value as i32);
                }
                matrix_index += 1;
            }
            if matrix_index >= bytes.len() {
                break;
            }
        }

        // Append parity symbols to data symbols
        for (i, val) in parity_symbols.into_iter().enumerate().take(symbol_length) {
            if i + symbol_length < symbols.len() {
                symbols[i + symbol_length] = val;
            }
        }
    }

    // GF(6) multiplication
    fn gf6_int_mul(&self, a: i32, b: i32) -> i32 {
        crate::ldpc::gf6_int_mul(a, b)
    }

    /// Drops the leading 8-bit IODE from a B1C-style "IODE + Ephemeris I" block (packed
    /// MSB-first, 24 bits per word) by shifting the bit stream left 8 bits. Yields the
    /// 203-bit B2b Ephemeris I, which carries no IODE (BDS-SIS-ICD-B2b Figure 6-3).
    fn drop_iode(src: &[u32; 32]) -> [u32; 9] {
        let mut out = [0u32; 9];
        for i in 0..9 {
            let cur = src[i] & 0xFF_FFFF;
            let next = src[i + 1] & 0xFF_FFFF;
            out[i] = ((cur << 8) | (next >> 16)) & 0xFF_FFFF;
        }
        out
    }

    // Get frame data for B-CNAV3
    pub fn get_frame_data(
        &self,
        start_time: GnssTime,
        svid: i32,
        _param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        // Check if svid is valid
        if !(1..=63).contains(&svid) {
            return 1;
        }

        let sow = start_time.MilliSeconds / 1000;
        let mut frame_data = [0u32; 21];
        let mut symbols = [0i32; 162];

        // Compose message based on message type
        self.compose_message(
            Self::MESSAGE_ORDER[(sow % 6) as usize],
            start_time.Week - 1356,
            sow,
            svid,
            &mut frame_data,
        );

        // Append CRC
        self.append_crc(&mut frame_data, 21);

        // Assign each 6bit into Symbols array
        symbols[0] = (frame_data[0] & 0x3f) as i32;
        for i in 0..20 {
            symbols[i * 4 + 1] = ((frame_data[i + 1] >> 18) & 0x3f) as i32;
            symbols[i * 4 + 2] = ((frame_data[i + 1] >> 12) & 0x3f) as i32;
            symbols[i * 4 + 3] = ((frame_data[i + 1] >> 6) & 0x3f) as i32;
            symbols[i * 4 + 4] = (frame_data[i + 1] & 0x3f) as i32;
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
    fn compose_message(
        &self,
        message_type: i32,
        week: i32,
        sow: i32,
        svid: i32,
        frame_data: &mut [u32],
    ) {
        // First fill in MessageType
        frame_data[0] = (message_type & 0x3f) as u32;

        match message_type {
            0 => {
                // SOW message
                frame_data[1] = COMPOSE_BITS!(sow as u32, 4, 20);
                frame_data
                    .iter_mut()
                    .take(20)
                    .skip(2)
                    .for_each(|w| *w = 0x5a5a5a5a);
            }
            10 => {
                // B-CNAV3 Message Type 10 (BDS-SIS-ICD-B2b Figure 6-3): the 480-bit body after
                // the 6-bit MesType is SOW(20) Rev(4) EphemerisI(203) EphemerisII(222)
                // DIF/SIF/AIF(3) SISMAI(4) CRC(24). The body lives in frame_data[1..21]
                // (frame_data[0] holds only the 6-bit MesType), so absolute append_word bits =
                // 24 + body-bit. Unlike B1C/B2a, B2b MT10 has NO IODE: Ephemeris I is 203 bits.
                frame_data[1] = COMPOSE_BITS!(sow as u32, 4, 20); // SOW at body bits 0..19
                                                                  // Ephemeris I (203 bits, no IODE) at body bit 24 -> absolute 48.
                let eph_i = Self::drop_iode(&self.ephemeris1[(svid - 1) as usize]);
                self.append_word(frame_data, 48, &eph_i, 203);
                // Ephemeris II (222 bits) at body bit 227 -> absolute 251.
                self.append_word(frame_data, 251, &self.ephemeris2[(svid - 1) as usize], 222);
                // DIF/SIF/AIF(3) + SISMAI(4) at body bits 449..455 -> absolute 473..479.
                let integ = self.integrity_flags[(svid - 1) as usize] & 0x7f;
                self.append_word(frame_data, 473, &[integ], 7);
            }
            30 => {
                // Clock, ionosphere, UTC, EOP message
                frame_data[1] = COMPOSE_BITS!(sow as u32, 4, 20);
                frame_data[1] |= COMPOSE_BITS!((week >> 9) as u32, 0, 4);
                frame_data[2] = COMPOSE_BITS!(week as u32, 15, 9);
                self.append_word(
                    frame_data,
                    2 * 24 + 13,
                    &self.clock_param[(svid - 1) as usize],
                    69,
                );
                frame_data[5] |= COMPOSE_BITS!(self.tgs_isc_param[(svid - 1) as usize][2], 12, 2);
                self.append_word(frame_data, 5 * 24 + 22, &self.bd_gim_iono, 74);
                self.append_word(frame_data, 9 * 24, &self.bdt_utc_param, 97);
                self.append_word(frame_data, 13 * 24 + 1, &self.eop_param, 138);
                frame_data[19] = 0; // SISAI_oc, SISAI_oe and HS filled with 0
            }
            40 => {
                // Almanac message
                frame_data[1] = COMPOSE_BITS!(sow as u32, 4, 20);
                self.append_word(
                    frame_data,
                    24 + 20,
                    &self.bgto_param[(sow % 3) as usize],
                    68,
                ); // First 3 BGTO fields

                let almanac_prn = sow % 63;
                self.append_word(
                    frame_data,
                    4 * 24 + 16,
                    &self.midi_almanac[almanac_prn as usize],
                    156,
                ); // Midi almanac

                frame_data[11] |= COMPOSE_BITS!(self.almanac_week, 13, 7);
                frame_data[11] |= COMPOSE_BITS!(self.almanac_toa >> 1, 7, 7);
                frame_data[12] = COMPOSE_BITS!(self.almanac_toa, 23, 1);

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
                    self.append_word(
                        frame_data,
                        bit_position,
                        &self.reduced_almanac[reduced_prn as usize],
                        38,
                    );
                }
            }
            _ => {
                // Unknown message type
            }
        }
    }

    // Interface methods required by NavBitTrait
    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> bool {
        if !(1..=63).contains(&svid) {
            return false;
        }
        // Convert GpsEphemeris to BDS B2b ephemeris format
        let index = (svid - 1) as usize;

        // Store in Ephemeris1 (primary) and Ephemeris2 (backup)
        // BDS B2b can store dual ephemeris sets for redundancy
        if index < 63 {
            // B-CNAV3 (B2b) Ephemeris I/II and clock data blocks are the same BDS-3 CNAV
            // definitions as B-CNAV1 (B1C), so reuse the verified BCNavBit packing (correct
            // ICD scale factors, semicircle conversion, two's-complement, ΔA = A − A_ref).
            // The previous body was a stub (it stored sqrtA.to_bits()/ecc.to_bits() raw and
            // dropped every other parameter). The Eph-I block is 211 bits → first 9 of the
            // 32 source words; Eph-II is 222 bits → first 10 words.
            let mut b1c = BCNavBit::new();
            b1c.set_ephemeris(svid, eph);
            self.ephemeris1[index] = [0u32; 32];
            self.ephemeris2[index] = [0u32; 32];
            self.ephemeris1[index][..9].copy_from_slice(&b1c.ephemeris1[index]);
            self.ephemeris2[index][..10].copy_from_slice(&b1c.ephemeris2[index]);
            self.clock_param[index] = b1c.clock_param[index];

            true
        } else {
            false
        }
    }

    pub fn set_almanac(&mut self, alm: &[GpsAlmanac]) -> bool {
        // BDS B2b navigation message doesn't include almanac data
        // Almanac is broadcast in B1C and D1/D2 signals
        // This implementation acknowledges almanac data but doesn't store it

        let _almanac_count = alm
            .iter()
            .filter(|a| a.valid > 0 && a.svid > 0 && a.svid <= 63)
            .count();

        // Return true to indicate successful processing
        // In full implementation, would forward to appropriate almanac handler
        true
    }

    pub fn set_iono_utc(
        &mut self,
        iono_param: Option<&IonoParam>,
        utc_param: Option<&UtcParam>,
    ) -> bool {
        // Set ionospheric and UTC parameters for BDS B2b format
        if let Some(iono) = iono_param {
            if iono.flag > 0 {
                // Store BDS ionospheric parameters
                // BDS uses Klobuchar model similar to GPS but with different coefficients
                // Parameters are broadcast in subframe 1, page 1
            }
        }

        if let Some(utc) = utc_param {
            if utc.flag > 0 {
                // Store BDS UTC parameters
                // BDS broadcasts UTC-BDT relationship parameters
                // Different from GPS UTC parameters due to BDT time system
            }
        }

        true
    }
}

#[cfg(test)]
mod gf6_tests {
    use super::BCNav3Bit;
    use crate::bcnavbit::BCNavBit;

    /// Audit H13: B-CNAV3 LDPC uses GF(2^6) multiplication, which must be identical to
    /// the verified BCNavBit (B-CNAV1) encoder. Cross-check every operand pair.
    #[test]
    fn gf6_int_mul_matches_verified_field() {
        let b3 = BCNav3Bit::new();
        let b1 = BCNavBit::new();
        for a in 0..64 {
            for b in 0..64 {
                assert_eq!(
                    b3.gf6_int_mul(a, b),
                    b1.gf6_int_mul(a, b),
                    "gf6_int_mul mismatch at a={a}, b={b}"
                );
            }
        }
    }

    /// A genuine field has a multiplicative identity and is closed/commutative on the
    /// 63 non-zero elements (the broken identity-antilog table failed all of these).
    #[test]
    fn gf6_int_mul_is_a_field() {
        let b3 = BCNav3Bit::new();
        // multiplicative identity exists
        let id = (1..64)
            .find(|&e| (1..64).all(|a| b3.gf6_int_mul(a, e) == a))
            .expect("GF(2^6) must have a multiplicative identity");
        assert_eq!(id, 1, "полиномиальное представление GF64: единица = 1");
        for a in 1..64 {
            assert_eq!(b3.gf6_int_mul(a, id), a);
            for b in 1..64 {
                assert_eq!(b3.gf6_int_mul(a, b), b3.gf6_int_mul(b, a)); // commutative
                assert_ne!(b3.gf6_int_mul(a, b), 0); // closed on non-zero
            }
        }
    }

    /// B-CNAV3 (B2b) Ephemeris I/II are the same BDS-3 CNAV block as B-CNAV1, so the B1C
    /// decoder must round-trip the B2b encoder. Catches the old stub (sqrtA.to_bits() etc.).
    #[test]
    fn bcnav3_ephemeris_round_trips_via_b1c_decoder() {
        use crate::nav_decode::decode_bcnav_ephemeris;
        let mut eph = crate::types::GpsEphemeris {
            valid: 1,
            ..Default::default()
        };

        eph.flag = 3; // MEO -> A_ref 27906100 m
        eph.axis = 27906100.0 + 512.0;
        eph.axis_dot = -0.0625;
        eph.delta_n = 4.2e-9;
        eph.delta_n_dot = 1.0e-14;
        eph.M0 = 0.5;
        eph.ecc = 0.0007;
        eph.w = -1.1;
        eph.omega0 = 2.3;
        eph.i0 = 0.96;
        eph.omega_dot = -7.0e-9;
        eph.idot = 3.0e-10;
        eph.cuc = 1.5e-6;
        eph.cus = 6.0e-6;
        eph.crc = 180.0;
        eph.crs = -40.0;
        eph.cic = -2.0e-8;
        eph.cis = 9.0e-8;
        eph.af0 = -3.0e-4;
        eph.af1 = 1.5e-12;
        eph.toe = 345600;
        eph.toc = 345600;
        let mut b2b = BCNav3Bit::new();
        assert!(b2b.set_ephemeris(5, &eph));
        let eph1: [u32; 9] = b2b.ephemeris1[4][..9].try_into().unwrap();
        let eph2: [u32; 10] = b2b.ephemeris2[4][..10].try_into().unwrap();
        let dec = decode_bcnav_ephemeris(&eph1, &eph2, &b2b.clock_param[4]);
        let bad: Vec<_> = dec
            .compare_with_original(&eph)
            .into_iter()
            .filter(|d| !d.ok)
            .map(|d| format!("{}: orig={:.6e} dec={:.6e}", d.name, d.original, d.decoded))
            .collect();
        assert!(
            bad.is_empty(),
            "B-CNAV3 round-trip mismatches:\n  {}",
            bad.join("\n  ")
        );
    }

    /// Frame-level round-trip: assemble a real B2b MT10 frame (compose_message + CRC) and recover
    /// Ephemeris I/II straight out of frame_data. This exercises the framing layer — drop_iode,
    /// the 24-bit append_word, and the EphI(203, no IODE)/EphII(222) body offsets — not just the
    /// stored arrays. Catches the old 32-bit append_word and the IODE-inside-EphI framing bug.
    #[test]
    fn bcnav3_mt10_frame_round_trips() {
        use crate::nav_decode::decode_bcnav_ephemeris;
        let mut eph = crate::types::GpsEphemeris {
            valid: 1,
            ..Default::default()
        };

        eph.flag = 3; // MEO -> A_ref 27906100 m
        eph.axis = 27906100.0 + 512.0;
        eph.axis_dot = -0.0625;
        eph.delta_n = 4.2e-9;
        eph.delta_n_dot = 1.0e-14;
        eph.M0 = 0.5;
        eph.ecc = 0.0007;
        eph.w = -1.1;
        eph.omega0 = 2.3;
        eph.i0 = 0.96;
        eph.omega_dot = -7.0e-9;
        eph.idot = 3.0e-10;
        eph.cuc = 1.5e-6;
        eph.cus = 6.0e-6;
        eph.crc = 180.0;
        eph.crs = -40.0;
        eph.cic = -2.0e-8;
        eph.cis = 9.0e-8;
        eph.af0 = -3.0e-4;
        eph.af1 = 1.5e-12;
        eph.toe = 345600;
        eph.toc = 345600;

        let svid = 5i32;
        let mut b2b = BCNav3Bit::new();
        assert!(b2b.set_ephemeris(svid, &eph));

        // Assemble the real MT10 frame the way get_frame_data does.
        let mut frame = [0u32; 21];
        b2b.compose_message(10, 2369 - 1356, 100, svid, &mut frame);
        b2b.append_crc(&mut frame, 21);

        assert_eq!(frame[0] & 0x3f, 10, "symbol 0 must be MesType 10");
        assert_ne!(frame[20], 0, "CRC field must be computed");

        // Body bit b lives in frame[1 + b/24] at MSB-offset (23 - b%24).
        let body_bit = |b: usize| -> u32 { (frame[1 + b / 24] >> (23 - (b % 24))) & 1 };

        // Ephemeris I: body bits 24..226 (203 bits, NO IODE). Re-prepend a dummy 8-bit IODE so the
        // B1C decoder (which expects IODE(8) + EphI(203) = 211 bits) can read it.
        let mut eph1 = [0u32; 9];
        for i in 0..203 {
            let pos = 8 + i;
            eph1[pos / 24] |= body_bit(24 + i) << (23 - (pos % 24));
        }
        // Ephemeris II: body bits 227..448 (222 bits).
        let mut eph2 = [0u32; 10];
        for i in 0..222 {
            eph2[i / 24] |= body_bit(227 + i) << (23 - (i % 24));
        }

        // Clock params come from MT30, not MT10 — feed the encoder's so the decoder is satisfied;
        // we only assert the orbital fields carried by EphI/EphII.
        let dec = decode_bcnav_ephemeris(&eph1, &eph2, &b2b.clock_param[(svid - 1) as usize]);
        let bad: Vec<_> = dec
            .compare_with_original(&eph)
            .into_iter()
            .filter(|d| !d.ok)
            .map(|d| format!("{}: orig={:.6e} dec={:.6e}", d.name, d.original, d.decoded))
            .collect();
        assert!(
            bad.is_empty(),
            "B2b MT10 frame round-trip mismatches:\n  {}",
            bad.join("\n  ")
        );
    }
}
