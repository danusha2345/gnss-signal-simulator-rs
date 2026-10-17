#include "rtklib.h"
#include <stdio.h>
#include <stdlib.h>
/* Решение из декодированных SF2 и измерений PocketSDR, без исходного RINEX. */
int main(int argc,char **argv){
if (argc != 2) return 1;


 FILE*f=fopen(argv[1],"r");
nav_t nav={
0}
;
int n;
if(!f||fscanf(f,"%d",&n)!=1)return 1;

 nav.eph=calloc(n,sizeof(eph_t));
nav.n=nav.nmax=n;

 for(int i=0;
i<n;
i++){
int sv;
double v[21];
if(fscanf(f,"%d",&sv)!=1)return 2;
for(int j=0;
j<21;
j++)if(fscanf(f,"%lf",v+j)!=1)return 3;
eph_t*e=nav.eph+i;
e->sat=satno(SYS_CMP,sv);
e->week=v[0];
e->toes=v[1];
e->toe=bdt2gpst(bdt2time(v[0],v[1]));
e->toc=bdt2gpst(bdt2time(v[0],v[2]));
e->ttr=e->toe;
e->A=v[3];
e->e=v[4];
e->i0=v[5];
e->OMG0=v[6];
e->omg=v[7];
e->M0=v[8];
e->deln=v[9];
e->OMGd=v[10];
e->idot=v[11];
e->crc=v[12];
e->crs=v[13];
e->cuc=v[14];
e->cus=v[15];
e->cic=v[16];
e->cis=v[17];
e->f0=v[18];
e->f1=v[19];
e->f2=v[20];
}

 prcopt_t opt=prcopt_default;
opt.navsys=SYS_CMP;
opt.mode=PMODE_SINGLE;
opt.ionoopt=IONOOPT_OFF;
opt.tropopt=TROPOPT_OFF;
opt.elmin=5*D2R;

 sol_t sol={
0}
;
double t;
int m;
while(fscanf(f,"%lf %d",&t,&m)==2){
obsd_t obs[64]={
{
0}
}
;
for(int i=0;
i<m;
i++){
int sv;
double p,cn0;
if(fscanf(f,"%d %lf %lf",&sv,&p,&cn0)!=3)return 4;
obs[i].time=timeadd(gpst2time(2369,381948),t);
obs[i].sat=satno(SYS_CMP,sv);
obs[i].rcv=1;
obs[i].code[0]=obs2code("1D");
obs[i].P[0]=p;
obs[i].SNR[0]=cn0/SNR_UNIT;
}

 char msg[256]={
0}
;
double pos[3];
if(pntpos(obs,m,&nav,&opt,&sol,NULL,NULL,msg)){
ecef2pos(sol.rr,pos);
printf("%.3f %.10f %.10f %.4f %d\n",t,pos[0]*R2D,pos[1]*R2D,pos[2],sol.ns);
}
else fprintf(stderr,"%.3f %s\n",t,msg);

 }
free(nav.eph);
fclose(f);
return 0;

}
