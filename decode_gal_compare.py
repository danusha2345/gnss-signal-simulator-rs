import numpy as np, sys, os
import verify_signal_enhanced as v

SR=5e6; SPM=5000; SPC=4*SPM; FL1=v.F_L1
SYNC=np.array([0,1,0,1,1,0,0,0,0,0])

def extract_symbols(fname, svid, dur_ms=8000):
    sig_acq=v.load_iq8(fname,SR,0,1600)
    res=v.acquire_system(sig_acq,"E1",[svid],v.generate_e1_boc,4,SR,v.NON_COHERENT_GAL,doppler_step=50,carrier_freq=FL1)
    sv=res['all'][0]; dopp=float(sv['doppler']); cp=float(sv['code_phase'])
    print(f"  {os.path.basename(fname)} SV{svid}: z={sv['zscore']:.0f} dopp={dopp:+.0f} cp={cp:.0f}")
    sig=v.load_iq8(fname,SR,0,dur_ms)
    repl=v.resample_code(v.generate_e1_boc(svid),SPC).astype(np.float64)  # 0/1 -> resampled
    repl=1.0-2.0*repl  # +1/-1
    nsym=(len(sig)-int(cp))//SPC - 1
    Cd=np.zeros(nsym,complex)
    n=np.arange(SPC)
    for k in range(nsym):
        pos=cp + k*SPC*(1.0+dopp/FL1)
        i0=int(pos)
        seg=sig[i0:i0+SPC]
        if len(seg)<SPC: break
        carr=np.exp(-1j*2*np.pi*dopp*(np.arange(i0,i0+SPC))/SR)
        Cd[k]=np.sum(seg*carr*repl)
    # phase via squaring, windowed
    syms=np.zeros(nsym,int)
    W=20
    for k in range(nsym):
        a=max(0,k-W); b=min(nsym,k+W)
        th=0.5*np.angle(np.sum(Cd[a:b]**2))
        val=(Cd[k]*np.exp(-1j*th)).real
        syms[k]=1 if val>0 else 0
    return syms,Cd

def find_sync(syms):
    hits=[]; inv=[]
    s=SYNC; si=1-SYNC
    for i in range(len(syms)-10):
        w=syms[i:i+10]
        if np.array_equal(w,s): hits.append(i)
        if np.array_equal(w,si): inv.append(i)
    return hits,inv

if __name__=="__main__":
    f=sys.argv[1]; svid=int(sys.argv[2]) if len(sys.argv)>2 else 30
    syms,Cd=extract_symbols(f,svid)
    h,iv=find_sync(syms)
    print(f"  symbols={len(syms)}  SYNC at: {h[:8]}  (diffs {np.diff(h[:8]) if len(h)>1 else []})")
    print(f"  INV SYNC at: {iv[:8]}")
