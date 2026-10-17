# Независимый разбор BCNAV1: разрядности из GNSS-SDR parse_subframe2 / ICD.
from pathlib import Path
import math,json,collections,sys
folder=Path(sys.argv[1]);text=(folder/'nav.log').read_text().replace('\\r\\n','\n');eph={};obs=collections.defaultdict(list);pi=math.pi
for line in text.splitlines():
 p=line.split(',')
 if p[0]=='$NAV' and p[3]=='B1CD':
  bits=bin(int(p[7],16))[2:].zfill(len(p[7])*4);pos=14
  def read(n,scale=1,signed=False):
   global pos
   v=int(bits[pos:pos+n],2);pos+=n
   if signed and v>>(n-1):v-=1<<n
   return v*scale
  week=read(13);how=read(8);iodc=read(10);iode=read(8);toe=read(11,300);typ=read(2);A=(42162200 if typ in [1,2] else 27906100)+read(26,2**-9,True);Adot=read(25,2**-21,True);dn=read(17,pi*2**-44,True);dndot=read(23,pi*2**-57,True);M0=read(33,pi*2**-32,True);ecc=read(33,2**-34);omega=read(33,pi*2**-32,True);O0=read(33,pi*2**-32,True);i0=read(33,pi*2**-32,True);Od=read(19,pi*2**-44,True);idot=read(15,pi*2**-44,True);cis=read(16,2**-30,True);cic=read(16,2**-30,True);crs=read(24,2**-8,True);crc=read(24,2**-8,True);cus=read(21,2**-30,True);cuc=read(21,2**-30,True);toc=read(11,300);af0=read(25,2**-34,True);af1=read(22,2**-50,True);af2=read(11,2**-66,True)
  assert Adot==0 and dndot==0,'legacy RTKLIB orbit adapter requires zero rates'
  sv=int(p[4]);eph.setdefault(sv,[week,toe,toc,A,ecc,i0,O0,omega,M0,dn,Od,idot,crc,crs,cuc,cus,cic,cis,af0,af1,af2])
 elif p[0]=='$OBS' and p[9]=='1D':
  # $OBS,t,yyyy,mm,dd,hh,mm,ss,sat,code,CN0,P,L,D,...
  obs[float(p[1])].append((int(p[8][1:]),float(p[11]),float(p[10])))
with (folder/'rtklib_input.txt').open('w') as f:
 f.write(str(len(eph))+'\n')
 for sv,e in eph.items():f.write(' '.join(map(str,[sv]+e))+'\n')
 for t,rows in sorted(obs.items()):
  if t<46:continue
  f.write(f'{t} {len(rows)}\n')
  for r in rows:f.write(' '.join(map(str,r))+'\n')
(folder/'decoded_ephemerides.json').write_text(json.dumps(eph,indent=2));print('decoded ephemerides',len(eph),'observation epochs',len(obs))
