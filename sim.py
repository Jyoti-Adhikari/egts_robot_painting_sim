"""
Reference simulation of remote teleoperation of a velocity-controlled tool point
on a planar virtual painting surface, with a local safety layer (freshness check,
speed limit, watchdog, latched stop, manual re-arm).

This is NOT ROS 2 / MoveIt 2. It is the "small standalone simulated control model"
named in the project's contingency plan. All numeric settings below are test inputs
chosen by the authors; they are swept or stated explicitly, never presented as
measurements of a real network or robot.
"""
import heapq
import json
import sys
import numpy as np

# ---------------- reference path (boustrophedon raster) ----------------
STROKES, LEN, SPACING = 5, 240.0, 40.0
pts = [(0.0, 0.0)]
for k in range(STROKES):
    x_end = LEN if k % 2 == 0 else 0.0
    pts.append((x_end, k * SPACING))
    if k < STROKES - 1:
        pts.append((x_end, (k + 1) * SPACING))
PTS = np.array(pts)
SEG = PTS[1:] - PTS[:-1]
SEGLEN = np.linalg.norm(SEG, axis=1)
CUM = np.concatenate([[0.0], np.cumsum(SEGLEN)])
L_TOTAL = CUM[-1]


def ref_pos_vel(s):
    s = min(max(s, 0.0), L_TOTAL)
    i = min(np.searchsorted(CUM, s, side="right") - 1, len(SEGLEN) - 1)
    u = SEG[i] / SEGLEN[i]
    p = PTS[i] + u * (s - CUM[i])
    v = u if s < L_TOTAL else np.zeros(2)
    return p, v


def closest_s(p):
    best, bs = 1e18, 0.0
    for i in range(len(SEGLEN)):
        a = PTS[i]
        t = np.clip(np.dot(p - a, SEG[i]) / SEGLEN[i] ** 2, 0, 1)
        q = a + t * SEG[i]
        d = np.linalg.norm(p - q)
        if d < best:
            best, bs = d, CUM[i] + t * SEGLEN[i]
    return bs


def cross_track(P):
    """distance from each point in P (n,2) to the reference polyline."""
    d = np.full(len(P), 1e18)
    for i in range(len(SEGLEN)):
        a = PTS[i]
        t = np.clip(((P - a) @ SEG[i]) / SEGLEN[i] ** 2, 0, 1)
        q = a + t[:, None] * SEG[i]
        d = np.minimum(d, np.linalg.norm(P - q, axis=1))
    return d


# ---------------- one run ----------------
def run(delay=0.100, jitter_frac=0.10, loss=0.0, T=0.250, seed=0,
        vref=40.0, vmax=50.0, K=2.0, tau=0.05, noise=3.0,
        dt=0.005, op_dt=0.02, t_end=None,
        cut=None,               # (t_start, t_stop) link cut window, t_stop may be inf
        blackouts=False, bo_rate=0.2, bo_mean=0.10,
        watchdog=True, age_check=True, latch=True, rearm_time=None,
        record=False):
    rng = np.random.default_rng(seed)
    if t_end is None:
        t_end = L_TOTAL / vref + 4.0
    n = int(round(t_end / dt))
    P = np.zeros((n, 2)); V = np.zeros((n, 2)); STATE = np.zeros(n, dtype=np.int8)
    p = PTS[0].copy(); v = np.zeros(2); vcmd = np.zeros(2)
    armed = True
    last_accept = 0.0; last_seq = -1; seq = 0
    heap = []
    s0, t_sync = 0.0, 0.0
    p_seen = p.copy()
    u_cur = np.zeros(2)
    rej = {"stale": 0, "ooo": 0, "disarmed": 0}
    ages = []; clipped = 0; sent = 0; lost = 0
    stops = []            # watchdog trip times
    tripped = False
    rearm_done = False
    stop_t = None
    # random blackouts (synthetic burst-loss events)
    bo = []
    if blackouts:
        tt = 0.0
        while tt < t_end:
            tt += rng.exponential(1.0 / bo_rate)
            bo.append((tt, tt + rng.exponential(bo_mean)))
    op_every = int(round(op_dt / dt))
    sj = max(jitter_frac * delay, 0.001)

    def dly():
        return max(0.001, delay + sj * rng.standard_normal())

    for i in range(n):
        t = i * dt
        # --- operator tick
        if i % op_every == 0:
            # delayed feedback
            idx = max(0, int((t - dly()) / dt))
            in_cut = cut is not None and cut[0] <= t < cut[1]
            in_bo = any(a <= t < b for a, b in bo)
            if not (in_cut or in_bo):
                p_seen = P[idx].copy() if i > 0 else p.copy()
            s = s0 + vref * (t - t_sync)
            r, rv = ref_pos_vel(s)
            u = vref * rv + K * (r - p_seen) + noise * rng.standard_normal(2)
            seq += 1; sent += 1
            drop = (rng.random() < loss) or in_cut or in_bo
            if drop:
                lost += 1
            else:
                heapq.heappush(heap, (t + dly(), seq, t, u))
        # --- manual re-arm
        if rearm_time is not None and (not rearm_done) and t >= rearm_time and not armed:
            armed = True; rearm_done = True; last_accept = t; tripped = False
            s0 = closest_s(p); t_sync = t
        # --- deliveries
        while heap and heap[0][0] <= t:
            t_arr, sq, t_send, u = heapq.heappop(heap)
            age = t_arr - t_send
            if sq <= last_seq:
                rej["ooo"] += 1; continue
            if age_check and age > T:
                rej["stale"] += 1; continue
            if not armed:
                rej["disarmed"] += 1; continue
            sp = np.linalg.norm(u)
            if sp > vmax:
                u = u * vmax / sp; clipped += 1
            vcmd = u; last_accept = t_arr; last_seq = sq; ages.append(age)
            tripped = False
        # --- watchdog
        if watchdog and (t - last_accept) > T and not tripped:
            tripped = True; stops.append(t)
            vcmd = np.zeros(2)
            if latch:
                armed = False
        if tripped:
            vcmd = np.zeros(2)
        # --- plant
        v = v + dt * (vcmd - v) / tau
        p = p + v * dt
        P[i] = p; V[i] = v; STATE[i] = 1 if armed else 0
    spd = np.linalg.norm(V, axis=1)
    e = cross_track(P)
    out = dict(rej=rej, clipped=clipped, sent=sent, lost=lost,
               stops=stops, ages=ages)
    out["P"] = P if record else None
    out["spd"] = spd if record else None
    out["e"] = e
    out["dt"] = dt
    out["spd_arr"] = spd
    out["state"] = STATE
    out["P_arr"] = P
    return out


def summarize_tracking(o, until=None):
    e = o["e"]
    if until is not None:
        e = e[: int(until / o["dt"])]
    return float(np.sqrt(np.mean(e ** 2))), float(np.max(e))