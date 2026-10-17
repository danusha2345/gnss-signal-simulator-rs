#!/usr/bin/env python3
# Воспроизводит ложное сообщение CRC на исходном split_page GNSS-SDR 0.0.20.
from pathlib import Path
import subprocess,sys,tempfile,hashlib
src=Path(sys.argv[1]).read_text();assert hashlib.sha256(src.encode()).hexdigest()=='4999596681c1afffd02f95af4040a6013d7f272bbcd83fcdf5f3d773a97a9dd5';a=src.index('void Galileo_Inav_Message::split_page(');b=src.index('\n}\n',a)+2
header=r'''#include <string>
#include <array>
#include <bitset>
#include <sstream>
#include <cstdint>
#include <iostream>
#include <cassert>
const int GALILEO_DATA_FRAME_BITS=196;
struct Galileo_Inav_Message {
 bool flag_CRC_test=false,next_crc=true;int calls=0,page_position_in_inav_subframe=255;
 std::string page_Even;
 std::array<int8_t,15> nma_position_filled{};
 struct {std::array<uint32_t,15> mack{};std::array<uint8_t,15> hkroot{};}nma_msg;
 uint8_t hkroot_sis=0;uint32_t mack_sis=0;
 bool CRC_test(const std::bitset<196>&,uint32_t){calls++;return next_crc;}
 void page_jk_decoder(const char*){}
 void split_page(std::string,int32_t);
};
'''
tail=r'''
int main(){Galileo_Inav_Message m;std::string odd(120,'0');odd[0]='1';std::string even(120,'0');
m.split_page(odd,0);std::cout<<"first odd: CRC calls="<<m.calls<<", flag="<<m.flag_CRC_test<<" -> upstream prints CRC error\n";assert(m.calls==0 && !m.flag_CRC_test);
m.split_page(even,0);m.split_page(odd,1);assert(m.calls==1 && m.flag_CRC_test);
m.next_crc=false;m.split_page(even,0);m.split_page(odd,1);assert(m.calls==2 && !m.flag_CRC_test);
std::cout<<"complete pair: real CRC invoked; valid and invalid pairs distinguished\n";}
'''
with tempfile.TemporaryDirectory(prefix='inav-repro-',dir='.scratch') as folder:
 p=Path(folder)/'repro.cc';binary=Path(folder)/'repro';p.write_text(header+src[a:b]+tail)
 subprocess.run(['c++','-std=c++17',str(p),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
