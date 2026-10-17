#include "pocket_sdr.h"
#include <stdio.h>
#include <assert.h>
int test_CRC(const uint8_t *bits, int len_bits);

int main(int argc,char**argv){
if (argc != 2) return 1;

FILE*f=fopen(argv[1],"r");
if (!f) return 2;
uint8_t raw[1800],rows[1728],a[1200],b[528],da[600],db[264];
for(int i=0;
i<1800;
i++)raw[i]=fgetc(f)=='1'?0:255;
fclose(f);
for(int i=0;
i<36;
i++)for(int j=0;
j<48;
j++)rows[i*48+j]=raw[72+j*36+i];
for(int i=0;
i<11;
i++){
memcpy(a+i*96,rows+i*3*48,48);
memcpy(a+i*96+48,rows+(i*3+1)*48,48);
memcpy(b+i*48,rows+(i*3+2)*48,48);
}
for(int i=22;
i<25;
i++)memcpy(a+i*48,rows+(i+11)*48,48);
int ea=sdr_decode_LDPC("BCNV1_SF2",a,1200,da),eb=sdr_decode_LDPC("BCNV1_SF3",b,528,db);
printf("LDPC corrected/errors SF2=%d SF3=%d; CRC SF2=%d SF3=%d\n",ea,eb,test_CRC(da,600),test_CRC(db,264));
assert(ea==0&&eb==0&&test_CRC(da,600)&&test_CRC(db,264));
}
