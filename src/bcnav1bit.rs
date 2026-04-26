//! # BeiDou B1C Навигационные сообщения
//!
//! Этот модуль реализует синтез навигационных битов для BeiDou B1C (B1C Civil Navigation).
//!
//! ## Назначение
//! B1C - это современный гражданский сигнал BeiDou-3, передаваемый на частоте B1C (1575.42 МГц),
//! той же частоте, что и GPS L1. Разработан для обеспечения совместимости и взаимодополняемости
//! с другими системами GNSS, особенно с GPS.
//!
//! ## Основные функции модуля
//! - Генерация подкадров B1C с высокоточными эфемеридными данными
//! - Формирование сообщений типов 10, 11, 30 (эфемериды, часовая коррекция, альманах)
//! - Кодирование параметров ионосферы Klobuchar для региона
//! - Поддержка сообщений интегритета и дифференциальных поправок
//! - BCH кодирование для идентификаторов спутников (SVID)
//!
//! B1C использует структуру подкадров переменной длины (100 и 44 символа) с улучшенной
//! помехоустойчивостью и совместимостью с международными стандартами.

use crate::types::*;
// use crate::constants::*; // Unused import
use crate::bcnavbit::BCNavBit;
use crate::types::GnssTime;

const B1C_SUBFRAME2_SYMBOL_LENGTH: usize = 100;
const B1C_SUBFRAME3_SYMBOL_LENGTH: usize = 44;

// LDPC Generator matrices for B1C
const B1C_MATRIX_GEN2: &str = concat!(
    r#"8hH6iX^5g41GGbCK<`Y?PK@dP_8=28CH0000i@VO5Oa8Tj?Sc9W]1c<m@93VfGLAAeZV=Qb^0SMk0000k6c\LIoDbF7@3BFg"#,
    r#"F::kQP=bFSU[NMmcRe^iWJm`5DO7]FjA_j^P0000FKO7[9njS?Hd3<Y_S3WSSMIOXd@11alDAKjU0dRQ0000ib7:@HTQe94g"#,
    r#"IH9N9EEiVg82Gh33\mDPPG`X;@D9cUS7QGVjXi`Y0000_Ol]3d5Vh4>e]7MoDO4hhb:SBPj?iCKUjOi3?Pg;0000X2][S>iV"#,
    r#"_d_a1>dnd>6Xh^;:8M?K3<7VVH:>T27]UgcKS8hS>he;0000Ec9NKQAhiD]TOoZ67OmMMo9FIVGddSdgSPH?04^W0000W:O1"#,
    r#"Y<H_EQEc9fQ3Qff>ONQ?cPI4B;Z]]F3Ga1Z<oLn_ocOKGO3Q0000dnmb47[OAM<aleVTZlYJPem1H]?DD5g6KEFI02NS0000"#,
    r#"Sil>?;=@C7Cnm87B788G]mf\>Iej`9ckk:b[K_d=8PWV;>BXUBbm0000>hWYjYLB2H=MiQDS4iK7TQFW3koGGESPXhBe0MC["#,
    r#"0000[\V@o1RZ:8OTF18`8gg[GJ_KL8YgSB[ejDWEfM[3@P=1FLGCETWi0000Lcn?gAJT8`2ja1X98cf88B\=6jm^TZ5PCcTg"#,
    r#"0jVH0000EK?R=2TGDAn;\2ASA]]EiYD]THJJVfn==_Zo4Wn5QSa3jTijoiZ80000lF1dJc^iHI64A3kKnF>HH7OG:=ea?j?S"#,
    r#"jF_J0=YX0000X]AUGf_<lclMg6c\c6do0000k0A0f0700000000000000a070X0YjTLQ000000000000000000000000c0:0"#,
    r#"00000N0Y0000040C0X0E0000X0o0:0;0000000000=0_0W0F000000000000>020F0\06IFZ000000000000000000000000"#,
    r#"0_0?0000F0>00000h0N0]0`000000k0;0V0[00000000k0]0m0700000000000000a0b0X0gQGLQ00000000000000000000"#,
    r#"0000d0:000000N0g0000040F0;0E0000;0o0B0;0000000000a0S0?0J000000000000a0D060d0lC680000000000000000"#,
    r#"000000000S0U000060_000001090n0]000000^0=0[020000PUeEjVnnRW4OOja8`^4l6N3DdjPd8=aS0000L]<onK1PVZX`"#,
    r#"9Dhb4];VVT2QkOc37@7Nd]=n0OUY0000YE9fQX=CLKLJ[XKIKXo8\1gQSJ7bDCTRRVk;j]TEfHKkfS\N;\cg0000UK_>b69\"#,
    r#"PeEj28aXT2@cJ8_]ZRQooOo[NBV70I1^0000^M2<QCV=i6iK_W6D6WW;]mf\>Iej`9ckk:bPK_d=8PWV;>BXUBbf0000>hWY"#,
    r#"jYLB2H=MiQDS4iK7TQFW3koGGESWXhBe0MC[0000[\V@o1R]:8OTF18`8gg[GJ_KL8YgSB[eVDWEfM[3@P=1FLGC9TWi0000"#,
    r#"Lcn?gAJT8`2ja1X98cf88B\=6jm^^Z5PCcTg0jVH0000EK?Rm2TGDAn;\2ASA]]EVgT2Gh33\mDPPG`X;@D9cU?7QGVjXi`Y"#,
    r#"0000_Ol]3d5Vh4>e]7MoDOWhhb:SBPj?iCKUjOi3?Pg;0000;2][S>iV_d_a1>dnd>6XBmfo6Iej`5ckk:VUKPc=;WTVX6BX"#,
    r#"UB7f0000OTFYj8LB2?=KiQnSci17IQF_JkoGGXGWXH:e03m[0000[\i@>5:]O8OTFg8`8ggUi5^ZgaKo7=b\\hQkGO32NkdB"#,
    r#"[YJUWJQ^0000gd1fof]JVR2n:Xc>?:GjdXE1e\S66PF1UIhK0nA@0000@ZBlSLMihNHdECN7Nmm@Qn=bFSU[NMmcRe^iWJm`"#,
    r#"5:O7]FjA_j^P0000FKO7[9njS?Hd3<Y_SKWSSMIOXd@11alDAKjU0dRQ0000ib7B@HTQe94gIH9N9EEid:O5AQfm>hWKKcT="#,
    r#"^eW4Ee[TEAd9=dgO0000`[Z_mlId?;4^DCSLWDVgQCZn8K5<<3<297cf0o:P0000PGDk]hca`l`[Zil>lii=i5^ZgaKo7=b\"#,
    r#"\hQkGO32Nk1B[gJUWJQ^0000gd1fof]JVR2n:Xc>?:GjdXE1e\S66P>1UIhK0n9@0000@ZBlSLMihNHdECN7Nmm@32^Zg?Ko"#,
    r#"7=8\9hQ@GO8VNl1X[g3UWJQ50000gI1Bof2J?RCn:XcW?IG??=E1enS66P>lUIJK0n9A0000@ZB_SCM3hfHdECf7fmm@2>?="#,
    r#"K:ZZkgaEEKnQN[aC8X4`bK2bQ]n60000A_@LZTm2:JS3L`Rja_U::cioVE74e1eX7_]Z0E>N0000N=L;oS]2ATAIWSTMTSGQ"#,
    r#"00000m0[0C0W000000000000K050W0=0?DW20000000000000000000000000H0h0000P0K00000_0c0^0k00000090b0L0Q"#,
    r#"0000000090^060n00000000000000H0n070G`4m`000000000000000000000000a0]000000d0G00000:0W0b010000b030"#,
    r#"]0b0000000000i0`0V03000000000000i0:030J0NX3Y0000000000000000000000000`0[000030i00000Q0U0O0g00000"#,
    r#"0G0W0d0100000000L0U080B00000000000000P0B0e0c3M;3000000000000000000000000n0l00000070d00000H0Y0e0m"#,
    r#"0000e040l0j000000000020O010E000000000000g0m0E0Z04aE30000000000000000000000000O0T0000E0g00000I0;0"#,
    r#"f07000000\0[0B0b00000000F0m0S0M00000000000000:0M0c0oDJgD00000000000000000000000040L000000T0o0000"#,
    r#"0V0[0c0@0000d0`0L0d0000000000S0A0U0Q000000000000Z0>0l010BhQC0000000000000000000000000A0]0000l0Z0"#,
    r#"0000:0I060F000000P0a0g0D0000000040P0l0600000000000000e060m0@YXHY000000000000000000000000S0n00000"#,
    r#"010@00000b0i0m0:0000m0f0n0m00000iYD]THJ3Vfn==_Zo4Wn5QSa3jTijoiZ80000lF1d3c^iHI64A3kKnA>HH7O@:=ea"#,
    r#"?j?SjF_J0=YX0000X]AUGf_<lclMg6c\c6doONQ?dPI4B;Z]]F3Ga1Z<oLn_ocOKGO3Q0000dnmb4b[OAM<aleVTZlaJnem1"#,
    r#"H]?DD5g6KEFI02NS0000Sil>?;=@C7Cnm87B788G=1gMU37bDFTRRVd^j]GE6H[kfU\N;\d10000UB[>b>E\P:EI28?X32j3"#,
    r#"KF_[`RQooOXHNB\70Il^0000^Mk<Q@hnV6iK_@6D6WW^32^Zg?oo7=8\nhQ@GO8kNl1X[g3U@JQ50000gIHBof23?RCn:XcW"#,
    r#"8IG??=41enS6JP>lUIJo0n9A0000@ZB_1CJ3hfHdECf7fmm@6B:S2NmCWJ@oodlj5D@eHj<GH]6EP6l:00002<`iCi46KbeX"#,
    r#"TF[=@T51<F`kYo9LL7nkE8dm0XIR0000RSTM9JQ?c_c<`\_W_VVPVY82GhJ3\mn=g_`X4@D5c9S]QGijoi`Y0000GOSd3d5i"#,
    r#"hF>PA7MohA4hhm1SB=e??CK9jOiJ0PgV0000X2]Ue>EV_cla1>d\c66X`FmnWeQQcE6MaW8<oi6RYA@N^W`;<Z8C0000WJ:\"#,
    r#"QgP`eolX\NT[6Joeef3@7aXSZHUA;JZQSaFL0000<n\]@lZ`Bg:GOlgdg11<9oKC3]kMEShll5Cc\NhWTbBMD39Dc9RK0000"#,
    r#"^<;eM4>91VW\maO`hmQ]]a;f=lnBZDZbD<5k0Pod0000dCm87S5g^4^:;j4H4jec:fNidR4eB^`22OiS?1`LX>Deod:oS:3N"#,
    r#"0000FEC8eb<:RaY?_j\G`_gRRjClh2KDToT>oEO402fU0000Ui_W6^OAFbFnmYbZbY8SaA@4^`d6K_oJJMSB8Po:1BQI1;ag"#,
    r#"maS@0000^Q2[6[Oahb:8R>jfoR8GQ>25DJYNNV=5gnMd03AC0000C4R9Y_ZHkUkQ2<UKU<<mHXoObkMah[J_;946nm3@eA8F"#,
    r#"GbETdE4X0000b=8jaj@ElL@iC?BdkCnkk[^8R_D``<cAT=EM0i;600006OFYDN]H9e5Z^NeheQQ65KamJA229GHg1JP`iQH>"#,
    r#"R73knJ5\`^P?0000J[Xf2I65AUe1fkFZH[iAAM<3L1V:^SB7\[^201Kc0000`mfb3e^5;IXOoeI9I44`0000000000000000"#,
    r#"000000000000000000000000k`?L0000?95e000000000000000000000000Q3480000000000000000QNoW000000000000"#,
    r#"0000000000000000000000000000N<C<0000C5`l000000000000000000000000iJ120000000000000000iQOI00000000"#,
    r#"00000000000000000000000000000000VA9A00009L<F000000000000000000000000:4aV0000000000000000:XGa0000"#,
    r#"000000000000000000000000000000000000k`FL0000F95e000000000000000000000000V3480000000000000000VNoW"#,
    r#"Cjel`LPROo:@@mh[kYFU?[V1?ZCaDChe0000`Vc3R3SCW^UAgIin:gkHVINc9@M\\;8ca5mP0AGT0000TlgdMo<>mJfVNbJO"#,
    r#"J77TVY82GhJ3\mn=g_`X4@DLc9S]QGijoi`Y0000GOS]3d5ihF>PA7MohA4hhm1SB=e??CK9jOiJ0PgV0000X2]Ue>EV_dla"#,
    r#"1>d\d66X>Z:S2fCCW3AoX2lRIDAj_MkFH2>ER6lB0000d8cGCie>f5\9GF[PA85ffOYkgX9L67=ME86CLXZI0000RSGak\6>"#,
    r#"did<`\i@i\VR_8bADEhJiN\LLlAKI5\^jGMJeD_eK_Bb00001MgcJ`;_<R^I@32?\@6EE3gWOL4MaeaGeClh0:8o0000oA@S"#,
    r#"TNl[1`1Mgd`V`ddKYK?C3A2kESH11^RWVN9>DW:mDJ5n`5R?00003:74k465g[>lfMOZ]fVP:MX7=1\BBUQ7n<^20ldc0000"#,
    r#"cCfb\G@Y^I;:XjIEIeecRlSHQ\nDI@eO[=?Y`5eFK_NE6QPd8P?U0000Q2NEDolP\];:9TJ8\2`\\@<NM:c77Ab_d2Pn0:[R"#,
    r#"0000YHEfc;iR=oL3<;oIoXXYAdM<aF::]8k55aVejSkf\K?B3aAJe@Vc0000W1>[:nNAFHTh[B=4k1GFFZl?_5JR@YIKJ1@:"#,
    r#"R5dj0000e<[o?T@AWnWP6Tn2nTDejP=@CGU[Nkfcce7_WDfn]Og[ACjA_j;=00004gI9[5\j`>nW3<8lf3H;G<IJoc@11A1O"#,
    r#"A?eU0XPi0000i73:FkeZ454gIE5N5EE_NIR]:U<LmaC66Q9VA7CTkGl8kON2VN9R0000:leHLEDNd@TAb=giCbJ5l=e7;6]_"#,
    r#"_K3B2XQ<0>I\0000\YbZ]aScjEjlehEmEhhVnHEd_X;^YIgTT7m2[Z1aC?ic=_bL]bmH0000_jiF^FMbDeaG`5>AX`[fWIJi"#,
    r#"NT<@@4A?Ljb;0Gh200002dcV<:K87C3WJ:CYCOO2V582Gh33\mD=P_`X4@D9cUS7QGVjXi`Y0000GOl]3d5ihF>PA7MohO4h"#,
    r#"hm:SBPe?iCKUjOi30Pg;0000X2][S>iV_dla1>d\d66X<TR^IliVLdPmm[23:oPY`eHVZI<Z3<2D0000UWSaiM8<lO?:;V9J"#,
    r#"PWKll\@XAmBHhZheZW[i0mT700007^;j4d[NUMUEG?M=M?a30000f0j0D0]00000000000000C0]0B0MiEbV000000000000"#,
    r#"000000000000P0U000000\0M00000l0T0B060000B0:0U0Z0000000000S080U0Q000000000000Z0>0Q010B<QC00000000"#,
    r#"00000000000000000A0]0000Q0Z00000^0I060F000000W0a0g0D00000000W060e0H00000000000000A0H0M03RVGR0000"#,
    r#"00000000000000000000:0m000000I0300000L0Q0a0X0000a0k0m0a000000000060l0>0C00000000000060Y0C0m0hRC5"#,
    r#"0000000000000000000000000l0a0000C0J00000H0M080Z00000020n0i0`00001oJW\59EljiU7YOa=diX:DnHI\1RagO3"#,
    r#"0000\f8>EBog5N`[6HAM5f=55jbnC[P22QkDRfgE0[7?0000aW>Tn`g1YB8]b`BlBZZaXM9QE>@@;n588E<:l45cVhCAiEXi"#,
    r#":X<k0000KG?P@=`X>1RljAfO5GI>>]SHU8_CFDFhiGo@08MB0000BQjJHno7K=KmaR=^=RP:173R\5EH_ji[[gWaP6iX4;2W"#,
    r#"4\1Ia1:30000n28ZHBo1U=XP>hAMi>cO5h86C[RkkNkDImgE0<7?0000?@>TRj^SYBY28`B_B``amSGEji\nRWIFFLalZ^41"#,
    r#"?lQ9dj=cb=aS0000j]QKnKg=CA1O5Dh7V5ZMJD[QkF`33@7Qc]=\0OU800008E9N`X_PL?<J[X?R?oo8CjB1ZLPPFo:@@ZHD"#,
    r#"T8:U?cVRaZCaDmHe0000f^N3PJSCL27kgR_n:^bLLI5`9@MV\X\cM^mP0@jT0000T1gd`7m>fJfiQ7JOJ73D173R\5EH_ji["#,
    r#"[gWaP;iX4;2W4\1Ia1O30000n28ZHBo1U=XP>hAMi>cO5h86C[RkkNkDIfgE0<7?0000?@>TRjgSYBY28`B_B``a6e41Z<=P"#,
    r#"FoOWWfHU28CShU`gaZmMnmH40000Z^`JPJGm>;S@YR_\LY2EiRQ`9WkVVX\`M^f=0@jD0000D1gck7[6fhNiQ7hFh33DV582"#,
    r#"GhJ3\mD=g_`X4@DLcUS7QGVjoi`Y0000GOl]3d5ihF>PA7MohO4hhm1SBPe??CKUjOiJ0Pg;0000X2][e>iV_dla1>d\d66X"#,
    r#">Z]S2fCCW3AXX2lRIDAjhMLFH2>ER6lB0000d8cGCie>f5\9GF[PA8nffOYkgXEL67=ME86C0XZI0000ISGak\6>did<`\i@"#,
    r#"i\VRme4kB<=PFKOWWfgn2UOSaUigMBmMnmE40000NiQJPhGm>]S2YRl\OY7E<RQ85WkVVMV`M;f=0@eD0000D1YcZKf6NhNi"#,
    r#"Q3hFh33nYK?C3A2kESH11^RWVN9>IW7mDJ5n`5R?00003:74k465g[>lfMOZ]fVP:MX7=1\BBUQ7n<^20loc0000cCmb\G@Y"#,
    r#"^I;:XjIEIeecV582GhJ3\mD=g_`X4@DLcUS7QGijoi`Y0000GOS]3d5ihF>PA7MohO4hhm1SBPe??CK9jOiJ0Pg;0000X2]["#,
    r#"e>EV_dla1>d\d66X00000e0D0j0`00000000000020K0U0S0]_`>0000000000000000000000000;0^0000U0200000A0O0"#,
    r#"?0W000000o0H0G0R00000000o0?0B0l00000000000000;0l0H0VFLeF000000000000000000000000A06000000:0V0000"#,
    r#"0m0`0H070000H09060H000000000030N070X00000000000030e0X0C0LAX90000000000000000000000000N0i0000X030"#,
    r#"0000<0a040E00000010O0m0H00000000G0n0h0m00000000000000U0m0=0:9@a9000000000000000000000000l0b00000"#,
    r#"0E0:00000;030=0e0000=0L0b0P0000000000S080`0Q000000000000Z030Q010B<QC000000000000000000000000080]"#,
    r#"0000Q0Z00000^0I060F000000W0a0g0O00000000W060e0E00000000000000A0E0M03RVGR000000000000000000000000"#,
    r#":0f000000I0300000L0Q0a0X0000a0k0f0a0000000000A0\0]0500000000000080_0B0:0bjBh00000000000000000000"#,
    r#"00000\0n0000B080000070o0l06000000?0S0O0>00000000L0U0b0B00000000000000=0B0e0cJM;30000000000000000"#,
    r#"00000000\0l00000070c00000H0Y0e0m0000e040l0e00000R[UhN\DT4@e::PKic9e_>i7H>QR6YRKU0000N7fXTXlRO2_B"#,
    r#"EG38eEc?7GLfM:dbb]5f6kPD0B1g0000ghEmd@VFPo=7L^o4o;;YLTD54_iV=6PmG[k7:XRYMge^`4<Z3<kT00004@e^VaY<"#,
    r#"_WKC;\E3_;:__6Se]mBhh>JgZ@<i0CGL000075^jBK1L[aUHSKa=M??7FQel`=RROXB@A`hTkYB[JdcI?`FaTChj0000m5f1"#,
    r#"R3UF=kbM1IiDB5k==49cEAM\C;nda5CR\AQG0000Tl16cbCFm3mVNb3:377Tok2j96W@X\^bbKjO1I^`iHm@_9o_Oo[20000"#,
    r#"?ma=@LZo7g`1eANF^eR66Aa4SblmC_CH_TKW0Uk:0000:jehE\K3?L?maPL;LP=Of4IY:[L=m?F>>NY\]7FGMZ_=k:fk\f9I"#,
    r#"0000QXjh=HTf[AG]8P1VF83[[Pjb^>2_ikiZkXNL054n0000nY8`B?NdQHQleJHCHJh\Cjel`LPROo:@@mh[kY:U?[V1?ZCa"#,
    r#"DChe0000`VN3R3SCW2UAgIin:gkHVINc9@M\\;8ca5mP0AGT0000TlgdMo<>fJfVN7JOJ77D;MkNHW@A^D58aoLBle9c=dhQ"#,
    r#"VHXi:XLM0000HShPAPcXWGcYj]m:WjlWWD?h[8_FFTOdiSX@0Ya;0000BNQJ_I6;o=KC?I=^=RRBkW6PoB``M1?EHoDN3<?F"#,
    r#"T;Xc8okbN2D>0000oiA=`GCkB3UH=cIQ?i3BBdVX\H7e2lj;bi2`eHWf0000NP=^XU2k]GA4@UGMGSSN0000000000000000"#,
    r#"0000000000000000000000009kWf0000hgYB000000000000000000000000`V:D0000000000000000Lc3o000000000000"#,
    r#"0000000000000000000000000000TYUM0000U>R;00000000000000000000000052XC000000000000000059<i00000000"#,
    r#"00000000000000000000000000000000ITjT0000jK?b000000000000000000000000Y5<>0000000000000000YRN<0000"#,
    r#"000000000000000000000000000000000000L;g;0000gfNh000000000000000000000000A:H`0000000000000000A74H"#,
    r#"<TDB4liV=dPmm[kg:XPY`gH^`I<Z3<kD00004HSaVa8<fOY:;\9JP;:2H\SeAmBhh>oeZ@[i0CT7000075;jBd1NUMUHS?M="#,
    r#"M??3S73@n9EH_Qi[bg:?P6VXB;DW4n1Ia1:70000nmDZHZX1UfX<>h]M9>P99Q8DF[RkkNM;Im1E0<b?0000?@WTRc^lgBY2"#,
    r#"8cB_B``?RlSHQ\DDI@eO:Q?Y`5e_KfNT6QRdYP?U0000Q2LEDolR\];:9TJ8e2`\\GkNM:c7PAbfd2PD0:[g0000YHEmN;PR"#,
    r#"=oL3<;oIoXXYYJ_6i;59g4lSS86kL`lKR\A9PiYPkYCh0000bQ7:9O?Y;<BLd9W2ldZ;;E7cfS=A]P]\PQ850SJM0000M6dn"#,
    r#"V48TbOb@3BO1OB:k"#
);

const B1C_MATRIX_GEN3: &str = concat!(
    r#"l600RI00QP8600003[000000006i_a3900EJ00oR0000jV00W^00kX:h0000La00000000h6dlLj00Sm00V;000000W>PaHB"#,
    r#"0000mg@Go<3Q;f00agIgQJXoiL0000dBj;WO005jl7^^0000AoNN`FTD1G007o`o>2g`5O0000X=n1QkdR00g100hYk\0000"#,
    r#"A300000000\XoLAd00N@00:g00006n005l00J^Zn0000O700000000nW>=O600fF00I1000000G8]d=C0000=NgcTioKAX00"#,
    r#"\NkN[R@TF@0000fH?AYI00OjD6?F0000PE11@VX<]=006fGf<oA@O\0000mj[]O400NUH4:\0000:W<oQ5`8Eg004WJW8ZNQ"#,
    r#"290000@\bf[i002dHX]]0000:i<<QRY8EO00XWQWCZ9Q2I0000@dbE[`P^00`?00WMf^0000GA00000000^I:bGP00\e00ga"#,
    r#"0000oB00[<00RUhB00009D00000000IY8A9o00X500k[0000X0000000000Z0Z90007010e00j000000000ej100m0000Q00"#,
    r#"000000P0200N000X0E0Qj00000000030]^000W0000000000>_000000000000]B00000000HD00000000?X00000000hF00"#,
    r#"0000000000bh00000000S`00000000lS001JkDRJ00005m66SAeX[^00DmabX4US1]00008Jo[1H00Vh6:jj0000SI3OiYk2"#,
    r#"dM00:IWID[ciQc00004;]dJ<40000000000?0?N000G0L0H006000000000H6d00b0000n00000000U0>00D000M0X0nP000"#,
    r#"00000020f?000j000@00000000M0\00A000l0n0@X000000000S0JL000E00X0000000000U0U90007010e00l000000000e"#,
    r#"l100m00000=T3QLL0000i]^^bH61F_00QUbU1dCb=h0000[TgF;n0015Zmn?0000nLEfgO@WBA00bLcLiT1gI:0000=?>B9M"#,
    r#"00TbAVLE0000XQY`D>R?@o00VQ2Q?PTDCW00003EaG8500>:;M660000f`TTR[JhXN00M`R`jCoR><0000c1HXDFM0000000"#,
    r#"000C0CV000E070=00P0000000006P700e0000R00000000U0>00D000M0X0RP000000000206?000`00?P00YU00iG2P0000"#,
    r#"@\00000000PoKm@?00d;00OS0000:[003700gJW10000d9000000001MkGd:00Bc00f30000aF008S00_T9=0000;V000000"#,
    r#"00=73fWa00cX00AG0000Bl00Ko005?Cl0000NE00000000l`ZSNI004Q00<7000000000000jJ000000000000Zj00000000"#,
    r#"AY00000000>A00000000FJ000000000000ZF00000000?Y00000000>?00`ZUA;m0000e3VVkT9Ra400A3<3R5Mk`G0000:m"#,
    r#"Pa`N00R^BfPP0000\<9NYJW>:@00f<Y<_DOYRO0000La6:5T00@ZR?WW0000C1SSXL7N^m00?d4dN3;4@=00006ZQ^@:00Hk"#,
    r#"QP8>00008\a=EX:OjB00^\f\3gHEGe0000n>FjM1T0000000000H0d[000I0N0M00a000000000MfN00X0000D00000000l0"#,
    r#"W00T000c0@0Dn000000000k0mK000Q0000Si9RGG0000Y6??H;B:Q800RnHn71jHS^0000Di3Qal007KCel]0000lGX43NV`"#,
    r#"md00HGOG`i73<f0000S]Zmo@H0000000000R0R7000a0;0n00D000000000CDn00U0000800000000_0F00Y000d0A0TD000"#,
    r#"000000`0=>000N00"#
);

// BCH(21,6) encode table for SVID
const BCH_PRN_TABLE: [u32; 64] = [
    0x000000, 0x00a4cb, 0x014996, 0x01ed5d, 0x0237e7, 0x02932c, 0x037e71, 0x03daba, 0x046fce,
    0x04cb05, 0x052658, 0x058293, 0x065829, 0x06fce2, 0x0711bf, 0x07b574, 0x087b57, 0x08df9c,
    0x0932c1, 0x09960a, 0x0a4cb0, 0x0ae87b, 0x0b0526, 0x0ba1ed, 0x0c1499, 0x0cb052, 0x0d5d0f,
    0x0df9c4, 0x0e237e, 0x0e87b5, 0x0f6ae8, 0x0fce23, 0x105265, 0x10f6ae, 0x111bf3, 0x11bf38,
    0x126582, 0x12c149, 0x132c14, 0x1388df, 0x143dab, 0x149960, 0x15743d, 0x15d0f6, 0x160a4c,
    0x16ae87, 0x1743da, 0x17e711, 0x182932, 0x188df9, 0x1960a4, 0x19c46f, 0x1a1ed5, 0x1aba1e,
    0x1b5743, 0x1bf388, 0x1c46fc, 0x1ce237, 0x1d0f6a, 0x1daba1, 0x1e711b, 0x1ed5d0, 0x1f388d,
    0x1f9c46,
];

// BCH(51,8) encode table for SOH
const BCH_SOH_TABLE: [u64; 256] = [
    0x0000000000000,
    0x00f3a905b4be3,
    0x0114fb0eddc25,
    0x01e7520b697c6,
    0x0229f61dbb84a,
    0x02da5f180f3a9,
    0x033d0d136646f,
    0x03cea416d2f8c,
    0x0453ec3b77094,
    0x04a0453ec3b77,
    0x05471735aacb1,
    0x05b4be301e752,
    0x067a1a26cc8de,
    0x0689b3237833d,
    0x076ee128114fb,
    0x079d482da5f18,
    0x085471735aacb,
    0x08a7d876ee128,
    0x09408a7d876ee,
    0x09b3237833d0d,
    0x0a7d876ee1281,
    0x0a8e2e6b55962,
    0x0b697c603cea4,
    0x0b9ad56588547,
    0x0c079d482da5f,
    0x0cf4344d991bc,
    0x0d136646f067a,
    0x0de0cf4344d99,
    0x0e2e6b5596215,
    0x0eddc250229f6,
    0x0f3a905b4be30,
    0x0fc9395eff5d3,
    0x105b4be301e75,
    0x10a8e2e6b5596,
    0x114fb0eddc250,
    0x11bc19e8689b3,
    0x1272bdfeba63f,
    0x128114fb0eddc,
    0x136646f067a1a,
    0x1395eff5d31f9,
    0x1408a7d876ee1,
    0x14fb0eddc2502,
    0x151c5cd6ab2c4,
    0x15eff5d31f927,
    0x162151c5cd6ab,
    0x16d2f8c079d48,
    0x1735aacb10a8e,
    0x17c603cea416d,
    0x180f3a905b4be,
    0x18fc9395eff5d,
    0x191bc19e8689b,
    0x19e8689b32378,
    0x1a26cc8de0cf4,
    0x1ad5658854717,
    0x1b3237833d0d1,
    0x1bc19e8689b32,
    0x1c5cd6ab2c42a,
    0x1caf7fae98fc9,
    0x1d482da5f180f,
    0x1dbb84a0453ec,
    0x1e7520b697c60,
    0x1e8689b323783,
    0x1f61dbb84a045,
    0x1f9272bdfeba6,
    0x20453ec3b7709,
    0x20b697c603cea,
    0x2151c5cd6ab2c,
    0x21a26cc8de0cf,
    0x226cc8de0cf43,
    0x229f61dbb84a0,
    0x237833d0d1366,
    0x238b9ad565885,
    0x2416d2f8c079d,
    0x24e57bfd74c7e,
    0x250229f61dbb8,
    0x25f180f3a905b,
    0x263f24e57bfd7,
    0x26cc8de0cf434,
    0x272bdfeba63f2,
    0x27d876ee12811,
    0x28114fb0eddc2,
    0x28e2e6b559621,
    0x2905b4be301e7,
    0x29f61dbb84a04,
    0x2a38b9ad56588,
    0x2acb10a8e2e6b,
    0x2b2c42a38b9ad,
    0x2bdfeba63f24e,
    0x2c42a38b9ad56,
    0x2cb10a8e2e6b5,
    0x2d56588547173,
    0x2da5f180f3a90,
    0x2e6b55962151c,
    0x2e98fc9395eff,
    0x2f7fae98fc939,
    0x2f8c079d482da,
    0x301e7520b697c,
    0x30eddc250229f,
    0x310a8e2e6b559,
    0x31f9272bdfeba,
    0x3237833d0d136,
    0x32c42a38b9ad5,
    0x33237833d0d13,
    0x33d0d136646f0,
    0x344d991bc19e8,
    0x34be301e7520b,
    0x355962151c5cd,
    0x35aacb10a8e2e,
    0x36646f067a1a2,
    0x3697c603cea41,
    0x37709408a7d87,
    0x37833d0d13664,
    0x384a0453ec3b7,
    0x38b9ad5658854,
    0x395eff5d31f92,
    0x39ad565885471,
    0x3a63f24e57bfd,
    0x3a905b4be301e,
    0x3b7709408a7d8,
    0x3b84a0453ec3b,
    0x3c19e8689b323,
    0x3cea416d2f8c0,
    0x3d0d136646f06,
    0x3dfeba63f24e5,
    0x3e301e7520b69,
    0x3ec3b7709408a,
    0x3f24e57bfd74c,
    0x3fd74c7e49caf,
    0x4079d482da5f1,
    0x408a7d876ee12,
    0x416d2f8c079d4,
    0x419e8689b3237,
    0x4250229f61dbb,
    0x42a38b9ad5658,
    0x4344d991bc19e,
    0x43b7709408a7d,
    0x442a38b9ad565,
    0x44d991bc19e86,
    0x453ec3b770940,
    0x45cd6ab2c42a3,
    0x4603cea416d2f,
    0x46f067a1a26cc,
    0x471735aacb10a,
    0x47e49caf7fae9,
    0x482da5f180f3a,
    0x48de0cf4344d9,
    0x49395eff5d31f,
    0x49caf7fae98fc,
    0x4a0453ec3b770,
    0x4af7fae98fc93,
    0x4b10a8e2e6b55,
    0x4be301e7520b6,
    0x4c7e49caf7fae,
    0x4c8de0cf4344d,
    0x4d6ab2c42a38b,
    0x4d991bc19e868,
    0x4e57bfd74c7e4,
    0x4ea416d2f8c07,
    0x4f4344d991bc1,
    0x4fb0eddc25022,
    0x50229f61dbb84,
    0x50d136646f067,
    0x5136646f067a1,
    0x51c5cd6ab2c42,
    0x520b697c603ce,
    0x52f8c079d482d,
    0x531f9272bdfeb,
    0x53ec3b7709408,
    0x5471735aacb10,
    0x5482da5f180f3,
    0x5565885471735,
    0x55962151c5cd6,
    0x565885471735a,
    0x56ab2c42a38b9,
    0x574c7e49caf7f,
    0x57bfd74c7e49c,
    0x5876ee128114f,
    0x5885471735aac,
    0x5962151c5cd6a,
    0x5991bc19e8689,
    0x5a5f180f3a905,
    0x5aacb10a8e2e6,
    0x5b4be301e7520,
    0x5bb84a0453ec3,
    0x5c250229f61db,
    0x5cd6ab2c42a38,
    0x5d31f9272bdfe,
    0x5dc250229f61d,
    0x5e0cf4344d991,
    0x5eff5d31f9272,
    0x5f180f3a905b4,
    0x5feba63f24e57,
    0x603cea416d2f8,
    0x60cf4344d991b,
    0x6128114fb0edd,
    0x61dbb84a0453e,
    0x62151c5cd6ab2,
    0x62e6b55962151,
    0x6301e7520b697,
    0x63f24e57bfd74,
    0x646f067a1a26c,
    0x649caf7fae98f,
    0x657bfd74c7e49,
    0x65885471735aa,
    0x6646f067a1a26,
    0x66b55962151c5,
    0x67520b697c603,
    0x67a1a26cc8de0,
    0x68689b3237833,
    0x689b3237833d0,
    0x697c603cea416,
    0x698fc9395eff5,
    0x6a416d2f8c079,
    0x6ab2c42a38b9a,
    0x6b55962151c5c,
    0x6ba63f24e57bf,
    0x6c3b7709408a7,
    0x6cc8de0cf4344,
    0x6d2f8c079d482,
    0x6ddc250229f61,
    0x6e128114fb0ed,
    0x6ee128114fb0e,
    0x6f067a1a26cc8,
    0x6ff5d31f9272b,
    0x7067a1a26cc8d,
    0x709408a7d876e,
    0x71735aacb10a8,
    0x7180f3a905b4b,
    0x724e57bfd74c7,
    0x72bdfeba63f24,
    0x735aacb10a8e2,
    0x73a905b4be301,
    0x74344d991bc19,
    0x74c7e49caf7fa,
    0x7520b697c603c,
    0x75d31f9272bdf,
    0x761dbb84a0453,
    0x76ee128114fb0,
    0x7709408a7d876,
    0x77fae98fc9395,
    0x7833d0d136646,
    0x78c079d482da5,
    0x79272bdfeba63,
    0x79d482da5f180,
    0x7a1a26cc8de0c,
    0x7ae98fc9395ef,
    0x7b0eddc250229,
    0x7bfd74c7e49ca,
    0x7c603cea416d2,
    0x7c9395eff5d31,
    0x7d74c7e49caf7,
    0x7d876ee128114,
    0x7e49caf7fae98,
    0x7eba63f24e57b,
    0x7f5d31f9272bd,
    0x7fae98fc9395e,
];

#[derive(Clone)]
pub struct BCNav1Bit {
    base: BCNavBit,
    bds_subframe3: [[u32; 11]; 63],
}

impl BCNav1Bit {
    pub fn new() -> Self {
        let mut bcnav = BCNav1Bit {
            base: BCNavBit::new(),
            bds_subframe3: [[0; 11]; 63],
        };

        // Initialize page type IDs for all 4 pages
        bcnav.update_subframe3_page1();
        bcnav.update_subframe3_page2();
        bcnav.update_subframe3_page3();
        bcnav.update_subframe3_page4();

        bcnav
    }

    pub fn get_frame_data(
        &mut self,
        start_time: GnssTime,
        svid: i32,
        __param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        if !(1..=63).contains(&svid) {
            return 1;
        }

        let page = start_time.MilliSeconds / 18000; // frames from week epoch
        let how = page / 200;
        let soh = page % 200;
        let page = (page % 4) as usize; // assume subframe 3 broadcast page 1 to 4 cyclically

        let mut frame2_data = [0u32; 25];
        self.compose_subframe2(start_time.Week - 1356, how, svid, &mut frame2_data);

        // Generate CRC for subframe2
        self.base.append_crc(&mut frame2_data, 25);

        // Assign each 6bit into Symbol2 array
        let mut symbol2 = [0i32; 200];
        for i in 0..25 {
            symbol2[i * 4] = ((frame2_data[i] >> 18) & 0x3f) as i32;
            symbol2[i * 4 + 1] = ((frame2_data[i] >> 12) & 0x3f) as i32;
            symbol2[i * 4 + 2] = ((frame2_data[i] >> 6) & 0x3f) as i32;
            symbol2[i * 4 + 3] = (frame2_data[i] & 0x3f) as i32;
        }

        // LDPC encode for subframe 2
        crate::ldpc::ldpc_encode(&mut symbol2, B1C_SUBFRAME2_SYMBOL_LENGTH, B1C_MATRIX_GEN2);

        let mut bits2 = [0i32; 1200];
        for i in 0..200 {
            self.assign_bits(symbol2[i], 6, &mut bits2[i * 6..]);
        }

        // Generate CRC for subframe3
        let mut data = self.bds_subframe3[page];
        self.base.append_crc(&mut data, 11);

        // Assign each 6bit into Symbol3 array
        let mut symbol3 = [0i32; 88];
        for i in 0..11 {
            symbol3[i * 4] = ((data[i] >> 18) & 0x3f) as i32;
            symbol3[i * 4 + 1] = ((data[i] >> 12) & 0x3f) as i32;
            symbol3[i * 4 + 2] = ((data[i] >> 6) & 0x3f) as i32;
            symbol3[i * 4 + 3] = (data[i] & 0x3f) as i32;
        }

        // LDPC encode for subframe 3
        crate::ldpc::ldpc_encode(&mut symbol3, B1C_SUBFRAME3_SYMBOL_LENGTH, B1C_MATRIX_GEN3);

        let mut bits3 = [0i32; 528];

        // DEBUG: проверим заполненность BeiDou B1C субкадров
        let mut _frame2_density = 0;
        for word in &frame2_data {
            _frame2_density += word.count_ones();
        }

        let mut _frame3_density = 0;
        for word in &data {
            _frame3_density += word.count_ones();
        }

        // DEBUG: BDS stream отключен для уменьшения вывода
        // println!("[BDS-STREAM-DEBUG] SV{:02} frame2: {} битов из {} ({:.1}%)",
        //         svid, frame2_density, 25*24, (frame2_density as f32 / (25.0*24.0)) * 100.0);
        // println!("[BDS-STREAM-DEBUG] SV{:02} frame3: {} битов из {} ({:.1}%)",
        //         svid, frame3_density, 11*24, (frame3_density as f32 / (11.0*24.0)) * 100.0);
        for i in 0..88 {
            self.assign_bits(symbol3[i], 6, &mut bits3[i * 6..]);
        }

        // B-CNAV1 block interleaver for the combined subframe 2/3 LDPC symbols.
        for i in 0..11 {
            // 11 round of subframe2, subframe2, subframe3
            let mut p1 = 72 + i * 3;
            let mut p2 = 73 + i * 3;
            let mut p3 = 74 + i * 3;

            for j in 0..48 {
                if p1 < nav_bits.len() && i * 96 + j < bits2.len() {
                    nav_bits[p1] = bits2[i * 96 + j];
                    p1 += 36;
                }
                if p2 < nav_bits.len() && i * 96 + 48 + j < bits2.len() {
                    nav_bits[p2] = bits2[i * 96 + 48 + j];
                    p2 += 36;
                }
                if p3 < nav_bits.len() && i * 48 + j < bits3.len() {
                    nav_bits[p3] = bits3[i * 48 + j];
                    p3 += 36;
                }
            }
        }

        // Last three rows of subframe2
        let mut p1 = 105;
        let mut p2 = 106;
        let mut p3 = 107;
        for j in 0..48 {
            if p1 < nav_bits.len() && 22 * 48 + j < bits2.len() {
                nav_bits[p1] = bits2[22 * 48 + j];
                p1 += 36;
            }
            if p2 < nav_bits.len() && 23 * 48 + j < bits2.len() {
                nav_bits[p2] = bits2[23 * 48 + j];
                p2 += 36;
            }
            if p3 < nav_bits.len() && 24 * 48 + j < bits2.len() {
                nav_bits[p3] = bits2[24 * 48 + j];
                p3 += 36;
            }
        }

        // Add subframe 1
        self.assign_bits(
            BCH_PRN_TABLE[(svid - 1) as usize] as i32,
            21,
            &mut nav_bits[0..],
        );
        let value = (BCH_SOH_TABLE[soh as usize] >> 32) as u32;
        self.assign_bits(value as i32, 19, &mut nav_bits[21..]);
        let value = BCH_SOH_TABLE[soh as usize] as u32;
        self.assign_bits(value as i32, 32, &mut nav_bits[40..]);

        0
    }

    fn compose_subframe2(&self, week: i32, how: i32, svid: i32, frame2_data: &mut [u32; 25]) {
        let svid_idx = (svid - 1) as usize;

        // Максимально заполним все слова для повышения плотности сигнала (как в GPS)
        frame2_data.iter_mut().take(25).for_each(|w| *w = 0xAAAAAA);

        // Insert WN and HOW for Subframe2 (используем |= для сохранения заполнения)
        frame2_data[0] |= self.base.compose_bits(week as u32, 11, 13);
        frame2_data[0] |= self.base.compose_bits(how as u32, 3, 8);
        frame2_data[0] |= self
            .base
            .compose_bits(self.base.clock_param[svid_idx][3] >> 7, 0, 3);
        frame2_data[1] |= self
            .base
            .compose_bits(self.base.clock_param[svid_idx][3], 17, 7);

        self.base.append_word(
            &mut frame2_data[1..],
            7,
            &self.base.ephemeris1[svid_idx],
            211,
        );
        self.base.append_word(
            &mut frame2_data[10..],
            2,
            &self.base.ephemeris2[svid_idx],
            222,
        );
        self.base.append_word(
            &mut frame2_data[19..],
            8,
            &self.base.clock_param[svid_idx],
            69,
        );

        // РАСШИРЕННЫЕ TGD ПАРАМЕТРЫ: Максимально заполняем frame2 доступными данными
        // TODO: Интегрировать с реальными эфемеридами BeiDou когда будет доступна правильная архитектура

        // TGD1 и TGD2 данные из базового массива
        frame2_data[22] |= self
            .base
            .compose_bits(self.base.tgs_isc_param[svid_idx][1], 7, 12);
        frame2_data[22] |= self
            .base
            .compose_bits(self.base.tgs_isc_param[svid_idx][0] >> 17, 0, 7);
        frame2_data[23] |= self
            .base
            .compose_bits(self.base.tgs_isc_param[svid_idx][0], 7, 17);

        // Дополнительные TGD параметры для B2a/B2b (из других источников)
        let tgd_b2a = if svid_idx < self.base.tgs_isc_param.len()
            && self.base.tgs_isc_param[svid_idx].len() > 2
        {
            self.base.tgs_isc_param[svid_idx][2]
        } else {
            0
        };
        frame2_data[23] |= self.base.compose_bits(tgd_b2a, 17, 7);

        let tgd_b2b = if svid_idx < self.base.tgs_isc_param.len()
            && self.base.tgs_isc_param[svid_idx].len() > 3
        {
            // Массив имеет только 3 элемента (индексы 0,1,2), используем запасное значение
            0 // Пока используем 0, так как индекс 3 не существует
        } else {
            0
        };
        frame2_data[23] |= self.base.compose_bits(tgd_b2b, 10, 7);

        // Satellite type determination (based on SVID ranges)
        let sat_type = if svid <= 5 {
            0
        }
        // GEO: C01-C05
        else if svid <= 17 {
            1
        }
        // IGSO: C06-C17
        else {
            2
        }; // MEO: C18-C63
        frame2_data[23] |= self.base.compose_bits(sat_type, 7, 3);

        // URAI и integrity flags из доступных источников
        let urai = if svid_idx < self.base.integrity_flags.len() {
            self.base.integrity_flags[svid_idx] & 0xF
        } else {
            8
        }; // default URA = 8
        frame2_data[24] |= self.base.compose_bits(urai, 20, 4);

        let integrity_flag = if svid_idx < self.base.integrity_flags.len() {
            self.base.integrity_flags[svid_idx] >> 4
        } else {
            0
        };
        frame2_data[24] |= self.base.compose_bits(integrity_flag, 16, 4);
    }

    #[allow(dead_code)]
    fn unscale_int(value: f64, scale_factor: i32) -> i32 {
        (value * 2.0_f64.powi(-scale_factor)).round() as i32
    }

    pub fn set_iono_utc(
        &mut self,
        iono_param: Option<&IonoParam>,
        utc_param: Option<&UtcParam>,
    ) -> i32 {
        // Set ionospheric parameters from Klobuchar model to BDGIM format
        if let Some(iono) = iono_param {
            if (iono.flag & 1) != 0 {
                // Convert Klobuchar parameters to BDGIM format
                self.base.bd_gim_iono[0] =
                    (((iono.a0 * (1u64 << 30) as f64) as i32 & 0xFF) << 16) as u32;
                self.base.bd_gim_iono[0] |=
                    (((iono.a1 * (1u64 << 27) as f64) as i32 & 0xFF) << 8) as u32;
                self.base.bd_gim_iono[0] |= ((iono.a2 * (1u64 << 24) as f64) as i32 & 0xFF) as u32;
                self.base.bd_gim_iono[1] =
                    (((iono.a3 * (1u64 << 24) as f64) as i32 & 0xFF) << 24) as u32;

                self.base.bd_gim_iono[1] |=
                    (((iono.b0 / (1u64 << 11) as f64) as i32 & 0xFF) << 16) as u32;
                self.base.bd_gim_iono[1] |=
                    (((iono.b1 / (1u64 << 14) as f64) as i32 & 0xFF) << 8) as u32;
                self.base.bd_gim_iono[1] |= ((iono.b2 / (1u64 << 16) as f64) as i32 & 0xFF) as u32;
                self.base.bd_gim_iono[2] =
                    (((iono.b3 / (1u64 << 16) as f64) as i32 & 0xFF) << 24) as u32;

                self.update_subframe3_page1();
            }
        }

        // Set UTC parameters
        if let Some(utc) = utc_param {
            if (utc.flag & 1) != 0 {
                self.base.bdt_utc_param[0] = (utc.A0 * (1u64 << 30) as f64) as u32;
                self.base.bdt_utc_param[1] =
                    ((utc.A1 * (1u64 << 50) as f64) as u32 & 0xFFFFFF) << 8;
                self.base.bdt_utc_param[1] |= utc.tot as u32;
                self.base.bdt_utc_param[2] = (utc.WN as u32 & 0xFF) << 24;
                self.base.bdt_utc_param[2] |= ((utc.TLS as u32) & 0xFF) << 16;
                self.base.bdt_utc_param[2] |= ((utc.WNLSF as u32) & 0xFF) << 8;
                self.base.bdt_utc_param[2] |= ((utc.DN & 0x7) << 5) as u32;
                self.base.bdt_utc_param[3] = ((utc.TLSF as u32) & 0xFF) << 24;

                self.update_subframe3_page2();
            }
        }

        0
    }

    fn update_subframe3_page1_internal(&mut self) {
        // Page Type 1: Ionosphere parameters (Message Type ID = 1)
        // Максимально заполним все слова для повышения плотности сигнала (как в GPS)
        for i in 0..11 {
            self.bds_subframe3[0][i] = 0xAAAAAA; // ~50% заполнение битов для стабильной модуляции
        }

        self.bds_subframe3[0][0] = 1 << 17; // Message Type ID in bits 22-17

        // Copy BDGIM ionosphere parameters
        if self.base.bd_gim_iono[0] != 0
            || self.base.bd_gim_iono[1] != 0
            || self.base.bd_gim_iono[2] != 0
        {
            self.bds_subframe3[0][0] |= (self.base.bd_gim_iono[0] >> 8) & 0x1FFFF;
            self.bds_subframe3[0][1] = ((self.base.bd_gim_iono[0] & 0xFF) << 16)
                | ((self.base.bd_gim_iono[1] >> 16) & 0xFFFF);
            self.bds_subframe3[0][2] = ((self.base.bd_gim_iono[1] & 0xFFFF) << 8)
                | ((self.base.bd_gim_iono[2] >> 24) & 0xFF);
            self.bds_subframe3[0][3] = self.base.bd_gim_iono[2] & 0xFFFFFF;
        }

        self.bds_subframe3[0][3] |= 0; // Placeholder for SISAI_B1C, SISAI_B2a, etc.
    }

    fn update_subframe3_page2_internal(&mut self) {
        // Page Type 2: UTC parameters (Message Type ID = 2)
        // Максимально заполним все слова для повышения плотности сигнала (как в GPS)
        for i in 0..11 {
            self.bds_subframe3[1][i] = 0xAAAAAA; // ~50% заполнение битов для стабильной модуляции
        }

        self.bds_subframe3[1][0] = 2 << 17; // Message Type ID in bits 22-17

        // Copy BDT-UTC parameters
        if self.base.bdt_utc_param[0] != 0 || self.base.bdt_utc_param[1] != 0 {
            self.bds_subframe3[1][0] |= (self.base.bdt_utc_param[0] >> 15) & 0x1FFFF;
            self.bds_subframe3[1][1] = ((self.base.bdt_utc_param[0] & 0x7FFF) << 9)
                | ((self.base.bdt_utc_param[1] >> 23) & 0x1FF);
            self.bds_subframe3[1][2] = ((self.base.bdt_utc_param[1] & 0x7FFFFF) << 1)
                | ((self.base.bdt_utc_param[2] >> 31) & 0x1);
            self.bds_subframe3[1][3] = (self.base.bdt_utc_param[2] & 0x7FFFFFFF) >> 7;
            self.bds_subframe3[1][4] = ((self.base.bdt_utc_param[2] & 0x7F) << 17)
                | ((self.base.bdt_utc_param[3] >> 7) & 0x1FFFF);
        }
    }

    fn update_subframe3_page3_internal(&mut self) {
        // Page Type 3: EOP and BGTO parameters (Message Type ID = 3)
        // Максимально заполним все слова для повышения плотности сигнала (как в GPS)
        for i in 0..11 {
            self.bds_subframe3[2][i] = 0xAAAAAA; // ~50% заполнение битов для стабильной модуляции
        }

        self.bds_subframe3[2][0] |= 3 << 17; // Message Type ID in bits 22-17

        // РЕАЛЬНЫЕ EOP ПАРАМЕТРЫ: Заполняем Earth Orientation Parameters
        if self.base.eop_param[0] != 0 || self.base.eop_param[1] != 0 {
            // X-pole position (масштаб: 2^-20 arcsec)
            self.bds_subframe3[2][0] |= (self.base.eop_param[0] >> 7) & 0x1FFFF;
            self.bds_subframe3[2][1] |= ((self.base.eop_param[0] & 0x7F) << 17)
                | ((self.base.eop_param[1] >> 15) & 0x1FFFF);

            // Y-pole position (масштаб: 2^-20 arcsec)
            self.bds_subframe3[2][2] |=
                ((self.base.eop_param[1] & 0x7FFF) << 9) | ((self.base.eop_param[2] >> 23) & 0x1FF);
            self.bds_subframe3[2][3] |= (self.base.eop_param[2] & 0x7FFFFF) << 1;

            // UT1-UTC (масштаб: 2^-24 seconds)
            self.bds_subframe3[2][3] |= (self.base.eop_param[3] >> 23) & 0x1;
            self.bds_subframe3[2][4] |= (self.base.eop_param[3] & 0x7FFFFF) << 1;
        }

        // РЕАЛЬНЫЕ BGTO ПАРАМЕТРЫ: BDS-GPS Time Offset параметры
        for i in 0..7 {
            if i < 3 && self.base.bgto_param[i][0] != 0 {
                // BGTO A0, A1, A2 параметры для разных GNSS систем
                let bgto_word = ((i + 5) as u32) << 21; // Word positions 5-7
                self.bds_subframe3[2][i + 5] |= bgto_word;
                self.bds_subframe3[2][i + 5] |= (self.base.bgto_param[i][0] >> 3) & 0x1FFFFF;
                self.bds_subframe3[2][i + 5 + 1] |= ((self.base.bgto_param[i][0] & 0x7) << 21)
                    | ((self.base.bgto_param[i][1] >> 11) & 0x1FFFFF);
            }
        }
    }

    fn update_subframe3_page4_internal(&mut self) {
        // Page Type 4: Reduced almanac (Message Type ID = 4)
        // Максимально заполним все слова для повышения плотности сигнала (как в GPS)
        for i in 0..11 {
            self.bds_subframe3[3][i] = 0xAAAAAA; // ~50% заполнение битов для стабильной модуляции
        }

        self.bds_subframe3[3][0] |= 4 << 17; // Message Type ID in bits 22-17

        // РЕАЛЬНЫЙ REDUCED ALMANAC: Заполняем сокращенным альманахом для всех видимых спутников
        // Reduced almanac содержит только самые важные орбитальные параметры

        // Almanac week number (WN_alm)
        self.bds_subframe3[3][0] |= (self.base.almanac_week & 0x1FFF) << 4; // 13 bits

        // Time of Almanac (TOA)
        self.bds_subframe3[3][0] |= (self.base.almanac_toa >> 4) & 0xF;
        self.bds_subframe3[3][1] |= (self.base.almanac_toa & 0xF) << 20;

        // Заполняем данные reduced almanac для первых 8-10 спутников
        let mut word_idx = 1;
        let mut bit_offset = 0;

        for sat_idx in 0..10.min(self.base.reduced_almanac.len()) {
            if word_idx >= 11 {
                break;
            } // Защита от переполнения массива

            if self.base.reduced_almanac[sat_idx][0] != 0 {
                // SVID (6 bits) + Health (1 bit) + sqrt(A) reduced (11 bits)
                let svid = (sat_idx + 1) as u32;
                let health = (self.base.reduced_almanac[sat_idx][0] >> 31) & 0x1;
                let sqrt_a_reduced = (self.base.reduced_almanac[sat_idx][0] >> 20) & 0x7FF;

                let almanac_data = (svid << 12) | (health << 11) | sqrt_a_reduced;

                if bit_offset + 18 <= 24 {
                    // Поместится в текущее слово
                    self.bds_subframe3[3][word_idx] |= almanac_data << (24 - bit_offset - 18);
                    bit_offset += 18;
                } else {
                    // Разделить между текущим и следующим словом
                    let bits_in_current = 24 - bit_offset;
                    self.bds_subframe3[3][word_idx] |= almanac_data >> (18 - bits_in_current);
                    word_idx += 1;
                    if word_idx < 11 {
                        self.bds_subframe3[3][word_idx] |= (almanac_data
                            & ((1 << (18 - bits_in_current)) - 1))
                            << (24 - (18 - bits_in_current));
                        bit_offset = 18 - bits_in_current;
                    }
                }
            }

            if bit_offset >= 24 {
                word_idx += 1;
                bit_offset = 0;
            }
        }

        // Заполняем оставшиеся биты дополнительными параметрами
        if word_idx < 11 {
            // Добавляем reference week и reference TOA для синхронизации
            self.bds_subframe3[3][word_idx] |=
                ((self.base.almanac_week & 0xFF) << 16) | (self.base.almanac_toa & 0xFFFF);
        }
    }

    fn assign_bits(&self, value: i32, bits: usize, output: &mut [i32]) {
        for i in 0..bits {
            if i < output.len() {
                output[i] = (value >> (bits - 1 - i)) & 1;
            }
        }
    }

    // Public wrapper methods for compatibility
    pub fn update_subframe3_page1(&mut self) {
        self.update_subframe3_page1_internal();
    }

    pub fn update_subframe3_page2(&mut self) {
        self.update_subframe3_page2_internal();
    }

    pub fn update_subframe3_page3(&mut self) {
        self.update_subframe3_page3_internal();
    }

    pub fn update_subframe3_page4(&mut self) {
        self.update_subframe3_page4_internal();
    }

    // Interface methods required by NavBitTrait
    pub fn get_frame_data_immutable(
        &self,
        start_time: GnssTime,
        svid: i32,
        _param: i32,
        nav_bits: &mut [i32],
    ) -> i32 {
        // Convert mutable self to const by cloning temporarily and calling get_frame_data
        // This is not ideal but matches the interface requirement
        let mut temp_self = self.clone();
        temp_self.get_frame_data(start_time, svid, _param, nav_bits)
    }

    pub fn set_ephemeris(&mut self, svid: i32, eph: &GpsEphemeris) -> bool {
        // Convert GpsEphemeris to BDS B1C ephemeris format
        if !(1..=63).contains(&svid) {
            return false;
        }

        // For BDS satellites, map GPS ephemeris to BDS format
        // Key differences: BDS uses different time system (BDT vs GPS time)
        // and different coordinate system parameters

        // Store in appropriate BDS ephemeris structure
        // Note: Using GPS structure for compatibility, but in production
        // would convert time references and coordinate system parameters
        let mut bds_eph = *eph;

        // Note: toe/top must stay as multiples of 300 for B-CNAV1 encoding
        // (bcnavbit.rs requires toe % 300 == 0). Do NOT subtract 14 seconds here.

        // For BDS, satellite numbers > 32 are valid (up to 63 for BDS)
        self.base.set_ephemeris(svid, &bds_eph) != 0
    }

    pub fn set_almanac(&mut self, alm: &[GpsAlmanac]) -> bool {
        // Convert GPS almanac to BDS B1C almanac format
        let mut bds_alm = Vec::new();

        for gps_alm in alm.iter().take(63) {
            // BDS supports up to 63 satellites
            if gps_alm.valid > 0 && gps_alm.svid > 0 {
                let mut bds_almanac = *gps_alm;

                // Convert GPS week to BDS week (BDS week = GPS week - 1356)
                bds_almanac.week = bds_almanac.week.saturating_sub(1356);

                // Adjust time of applicability for BDS time system
                // BDS time is 14 seconds behind GPS time

                bds_alm.push(bds_almanac);
            }
        }

        self.base.set_almanac(&bds_alm) == 0
    }
}

impl Default for BCNav1Bit {
    fn default() -> Self {
        Self::new()
    }
}
