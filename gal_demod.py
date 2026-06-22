import numpy as np, sys, os
import verify_signal_enhanced as v

def demod(fname, svid, sr, dur_ms=8000):
    SPM=int(sr*1e-3); SPC=4*SPM; FL1=v.F_L1; SYNC=np.array([0,1,0,1,1,0,0,0,0,0])
    # acquire data code -> doppler50, cp
    sig0=v.load_iq8(fname,sr,0,1600)
    res=v.acquire_system(sig0,"E1",[svid],v.generate_e1_boc,4,sr,v.NON_COHERENT_GAL,doppler_step=25,carrier_freq=FL1)
    sv=res['all'][0]; d0=float(sv['doppler']); cp=float(sv['code_phase'])
    print(f"  acq SV{svid}: z={sv['zscore']:.0f} dopp0={d0:+.0f} cp={cp:.0f}")
    sig=v.load_iq8(fname,sr,0,dur_ms)
    dataR=1.0-2.0*v.resample_code(v.generate_e1_boc(svid),SPC).astype(np.float64)
    pilC=v.extract_e1_code(svid,use_pilot=True)
    pilR=1.0-2.0*v.resample_code(v.apply_boc11(pilC),SPC).astype(np.float64)
    nsym=(len(sig)-int(cp))//SPC-1
    # refine doppler: maximize pilot |sum over periods of per-period corr|^2 coherence via squared sum
    def corr_streams(dopp):
        Cd=np.zeros(nsym,complex); Cp=np.zeros(nsym,complex)
        idx=np.arange(SPC)
        for k in range(nsym):
            pos=cp+k*SPC*(1.0+dopp/FL1); i0=int(pos); seg=sig[i0:i0+SPC]
            if len(seg)<SPC: nsym2=k; break
            carr=np.exp(-1j*2*np.pi*dopp*(np.arange(i0,i0+SPC))/sr)
            sc=seg*carr
            Cd[k]=np.dot(sc,dataR); Cp[k]=np.dot(sc,pilR)
        return Cd,Cp
    best=None
    for dd in np.arange(d0-30,d0+30.1,5):
        _,Cp=corr_streams(dd)
        m=np.sum(np.abs(Cp[:200]**2).real)  # pilot energy (rough); better: coherence
        # coherence: pilot squared sum magnitude (CS25 removed by squaring)
        coh=abs(np.sum(Cp[:200]**2))
        if best is None or coh>best[0]: best=(coh,dd)
    dopp=best[1]; print(f"  refined dopp={dopp:+.1f}")
    Cd,Cp=corr_streams(dopp)
    # pilot-squared phase, smoothed
    W=15; syms=np.zeros(nsym,int)
    Cp2=Cp**2
    for k in range(nsym):
        a=max(0,k-W); b=min(nsym,k+W)
        th=0.5*np.angle(np.sum(Cp2[a:b]))
        syms[k]=1 if (Cd[k]*np.exp(-1j*th)).real>0 else 0
    # sync search both polarities
    hits=[i for i in range(nsym-10) if np.array_equal(syms[i:i+10],SYNC)]
    inv=[i for i in range(nsym-10) if np.array_equal(syms[i:i+10],1-SYNC)]
    print(f"  symbols={nsym}  SYNC@{hits[:6]} diffs={list(np.diff(hits[:6]))}")
    print(f"  INVSYNC@{inv[:6]} diffs={list(np.diff(inv[:6]))}")
    return syms

if __name__=="__main__":
    f=sys.argv[1]; svid=int(sys.argv[2]); sr=float(sys.argv[3]) if len(sys.argv)>3 else 5e6
    demod(f,svid,sr)
