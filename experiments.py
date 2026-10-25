import json, math
import numpy as np
from sim import *

N = 30
res = {}

def ms(x): return float(x)

def stats(a):
    a = np.asarray(a, float)
    if len(a) == 0:
        return dict(mean=None, sd=None, n=0)
    return dict(mean=float(a.mean()), sd=float(a.std(ddof=1)) if len(a) > 1 else 0.0, n=int(len(a)))

def wilson(k, n, z=1.96):
    if n == 0: return (None, None)
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0, c - h), min(1, c + h))

def tracking_block(**kw):
    rms, mx, nstop, age_med, age_p95, ooo, stale = [], [], 0, [], [], [], []
    for sd in range(N):
        o = run(seed=sd, **kw)
        if len(o["stops"]) > 0:
            nstop += 1
            continue
        r, m = summarize_tracking(o)
        rms.append(r); mx.append(m)
        if o["ages"]:
            age_med.append(np.median(o["ages"]) * 1000)
            age_p95.append(np.percentile(o["ages"], 95) * 1000)
        ooo.append(o["rej"]["ooo"]); stale.append(o["rej"]["stale"])
    return dict(rms=stats(rms), mx=stats(mx), n_stop=nstop, n=N,
                age_med=stats(age_med), age_p95=stats(age_p95),
                ooo=stats(ooo), stale=stats(stale),
                pass_rms=int(sum(1 for r in rms if r <= 10.0)),
                pass_max=int(sum(1 for m in mx if m <= 20.0)),
                pass_both=int(sum(1 for r, m in zip(rms, mx) if r <= 10.0 and m <= 20.0)))

# ---- E1: delay sweep, freshness window / timeout = 250 ms
res["E1"] = {}
for d in [2, 50, 100, 150, 200, 250, 300]:
    res["E1"][d] = tracking_block(delay=d / 1000, T=0.250)

# ---- E1b: delay sweep with long timeout (isolates tracking effect from stop logic)
res["E1b"] = {}
for d in [2, 50, 100, 200, 300, 400, 500, 600]:
    res["E1b"][d] = tracking_block(delay=d / 1000, T=2.0)

# ---- E2: loss sweep at 100 ms delay
res["E2"] = {}
for pl in [0, 5, 10, 20, 30, 40]:
    res["E2"][pl] = tracking_block(delay=0.100, loss=pl / 100, T=0.250)

# ---- E3a: permanent disconnect at t=15 s, timeout sweep (+ no watchdog)
res["E3a"] = {}
CUT_T = 15.0
for T in [0.100, 0.150, 0.250, 0.400, 0.600, None]:
    det, sb, over3, overstop = [], [], [], []
    for sd in range(N):
        kw = dict(delay=0.050, T=T if T else 0.250, seed=sd, cut=(CUT_T, 1e9))
        if T is None:
            kw.update(watchdog=False, age_check=False)
        o = run(**kw)
        spd = o["spd_arr"]; P = o["P_arr"]; dt = o["dt"]
        i0 = int(CUT_T / dt)
        i3 = int((CUT_T + 3.0) / dt)
        path_len = np.sum(np.linalg.norm(np.diff(P[i0:i3 + 1], axis=0), axis=1))
        over3.append(path_len)
        if T is not None:
            st = [s for s in o["stops"] if s >= CUT_T - 0.5]
            if st:
                det.append((st[0] - CUT_T) * 1000)
                j = i0
                while j < len(spd) and spd[j] >= 1.0: j += 1
                # first time after trip with speed < 1 mm/s
                jt = int(st[0] / dt)
                j = jt
                while j < len(spd) and spd[j] >= 1.0: j += 1
                sb.append((j * dt - CUT_T) * 1000)
                overstop.append(np.sum(np.linalg.norm(np.diff(P[i0:j + 1], axis=0), axis=1)))
    res["E3a"][("none" if T is None else int(T * 1000))] = dict(
        detect_ms=stats(det), standstill_ms=stats(sb), overrun_to_stop_mm=stats(overstop),
        dist_3s_mm=stats(over3), n_detected=len(det), n=N)

# ---- E3b: nuisance trips under synthetic random blackouts (no real fault)
NB = 100
res["E3b"] = {}
for T in [0.100, 0.150, 0.250, 0.400, 0.600]:
    k = 0
    for sd in range(NB):
        o = run(delay=0.050, T=T, seed=1000 + sd, blackouts=True, bo_rate=0.2, bo_mean=0.10)
        if len(o["stops"]) > 0: k += 1
    lo, hi = wilson(k, NB)
    res["E3b"][int(T * 1000)] = dict(k=k, n=NB, rate=k / NB, lo=lo, hi=hi)

# ---- E4: disconnect 3 s, restore, compare recovery policies
res["E4"] = {}
CUT = (12.5, 15.5)
REARM = 17.5
for name, kw in [("latched+manual_rearm", dict(latch=True, rearm_time=REARM)),
                 ("auto_resume", dict(latch=False))]:
    mv, mxe, tfirst, dis, nst = [], [], [], [], []
    for sd in range(N):
        o = run(delay=0.050, T=0.250, seed=sd, cut=CUT, t_end=50.0, **kw)
        P = o["P_arr"]; dt = o["dt"]; spd = o["spd_arr"]
        a, b = int(CUT[1] / dt), int(REARM / dt)
        mv.append(np.sum(np.linalg.norm(np.diff(P[a:b + 1], axis=0), axis=1)))
        mxe.append(np.max(o["e"][a:]))
        j = a
        while j < len(spd) and spd[j] < 1.0: j += 1
        tfirst.append((j * dt - CUT[1]) if j < len(spd) else float("nan"))
        dis.append(o["rej"]["disarmed"]); nst.append(len(o["stops"]))
    res["E4"][name] = dict(moved_before_rearm_mm=stats(mv), max_err_after_mm=stats(mxe),
                            first_motion_s=stats([x for x in tfirst if x == x]),
                            rejected_disarmed=stats(dis), watchdog_trips=stats(nst))

# ---- E5: reference speed sensitivity (delay 100 ms)
res["E5"] = {}
for vr in [10, 20, 30, 40, 45]:
    res["E5"][vr] = tracking_block(delay=0.100, T=0.250, vref=float(vr))

# ---- E6: operator gain K x servo lag tau (delay 100 ms)
res["E6"] = {}
for Kk in [1.0, 2.0, 4.0]:
    for tau in [0.02, 0.05, 0.10]:
        res["E6"][f"K{Kk}_tau{int(tau*1000)}"] = tracking_block(delay=0.100, T=0.250, K=Kk, tau=tau)

with open("results.json", "w") as f:
    json.dump(res, f, indent=1, default=str)
print("done")