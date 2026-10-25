import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
from sim import *

plt.rcParams.update({
    "font.family": "DejaVu Serif", "font.size": 8.5, "axes.linewidth": 0.8,
    "axes.edgecolor": "black", "axes.labelcolor": "black", "text.color": "black",
    "xtick.color": "black", "ytick.color": "black", "figure.facecolor": "white",
    "axes.facecolor": "white", "savefig.facecolor": "white", "savefig.dpi": 200,
})
R = json.load(open("results.json"))
OUT = "figs/"
import os; os.makedirs(OUT, exist_ok=True)


def box(ax, x, y, w, h, text, lw=1.0, ls="-", fs=8, fill="white"):
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                       fc=fill, ec="black", lw=lw, ls=ls)
    ax.add_patch(p)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs)


def arrow(ax, x1, y1, x2, y2, ls="-", lw=1.0, text=None, tx=0, ty=0.12, fs=7):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="-|>", lw=lw, ls=ls, color="black",
                                shrinkA=0, shrinkB=0))
    if text:
        ax.text((x1 + x2) / 2 + tx, (y1 + y2) / 2 + ty, text, ha="center", fontsize=fs)


# ---------------- Fig 1: architecture ----------------
fig, ax = plt.subplots(figsize=(7.0, 4.3))
ax.set_xlim(0, 14); ax.set_ylim(0, 8.6); ax.axis("off")
# regions
ax.add_patch(Rectangle((0.15, 0.4), 3.7, 7.7, fc="white", ec="black", lw=1.0, ls=(0, (6, 3))))
ax.text(2.0, 7.8, "OPERATOR STATION", ha="center", fontsize=8, fontweight="bold")
ax.add_patch(Rectangle((6.6, 0.4), 7.25, 7.7, fc="#eeeeee", ec="black", lw=2.0))
ax.text(10.2, 7.8, "LOCAL SIMULATION HOST  (safety boundary)", ha="center", fontsize=8, fontweight="bold")
# operator side
box(ax, 0.6, 5.7, 2.8, 1.2, "Operator\nkeyboard / gamepad")
box(ax, 0.6, 3.7, 2.8, 1.4, "Input processing\ndead zone, scaling,\ntimestamp, sequence no.")
box(ax, 0.6, 1.0, 2.8, 1.8, "Operator dashboard\nstatus, command age,\nstop / re-arm state")
arrow(ax, 2.0, 5.7, 2.0, 5.1)
# network
box(ax, 4.55, 3.7, 1.55, 1.4, "Network\nROS 2\ntopics", fs=7.5, ls=(0, (3, 2)))
arrow(ax, 3.4, 4.4, 4.55, 4.4, text="commands", ty=0.15)
arrow(ax, 4.55, 1.95, 3.4, 1.95, text="feedback", ty=0.15)
arrow(ax, 6.1, 4.4, 7.2, 4.4)
# host boxes
box(ax, 7.2, 3.5, 2.7, 1.8, "LOCAL SAFETY LAYER\nfresh? in bounds?\nspeed limit, watchdog\nlatched stop", lw=2.0, fs=7.5)
box(ax, 10.6, 3.5, 2.7, 1.8, "Motion control\nMoveIt 2 Servo /\npath tracking", fs=7.5)
arrow(ax, 9.9, 4.4, 10.6, 4.4, text="accepted", ty=0.15)
box(ax, 10.6, 1.0, 2.7, 1.5, "Simulated 6-axis arm\n+ virtual surface", fs=7.5)
arrow(ax, 11.95, 3.5, 11.95, 2.5)
box(ax, 7.2, 1.0, 2.7, 1.5, "Logger\nlatency, path error,\nrejections, stops", fs=7.5)
arrow(ax, 10.6, 1.75, 9.9, 1.75, text="state", ty=0.12)
arrow(ax, 7.2, 1.75, 6.1, 1.95, text=None)
ax.plot([6.1, 6.1], [1.95, 1.95], "k")
arrow(ax, 6.1, 1.95, 4.55, 1.95) if False else None
# line from logger to network box (feedback path)
ax.plot([7.2, 6.4, 6.4, 5.33], [1.75, 1.75, 1.95, 1.95], "k", lw=1.0)
ax.plot([5.33, 5.33], [1.95, 3.7], "k", lw=1.0)
# local stop
box(ax, 7.2, 5.9, 2.7, 1.2, "LOCAL STOP BUTTON\n(no network path)", lw=2.0, fs=7.5)
arrow(ax, 8.55, 5.9, 8.55, 5.3, lw=1.5)
ax.text(7.0, 0.1, "Dashed = untrusted link side; heavy outline = safety function that must work with the link down.",
        fontsize=6.8, ha="left")
fig.savefig(OUT + "fig1_architecture.png", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)

# ---------------- Fig 2: timing definitions ----------------
fig, ax = plt.subplots(figsize=(7.0, 2.7))
ax.set_xlim(0, 10); ax.set_ylim(0, 4); ax.axis("off")
ax.annotate("", xy=(9.8, 0.6), xytext=(0.2, 0.6), arrowprops=dict(arrowstyle="-|>", lw=1))
ax.text(9.8, 0.25, "time", ha="right", fontsize=8)
def tick(x, label, y=0.6, up=True):
    ax.plot([x, x], [y - 0.1, y + 0.1], "k", lw=1)
    ax.text(x, y - 0.4, label, ha="center", fontsize=7.5)
tick(1.0, "$t_s$\ncommand\nsent"); tick(2.6, "$t_r$\nlast accepted\ncommand\nreceived")
tick(6.0, "$t_d$\nwatchdog\ntrips"); tick(8.2, "$t_0$\nstandstill\n(speed < 1 mm/s)")
ax.annotate("", xy=(2.6, 1.3), xytext=(1.0, 1.3), arrowprops=dict(arrowstyle="<|-|>", lw=1))
ax.text(1.8, 1.45, "command age  $a = t_r - t_s$", ha="center", fontsize=7.5)
ax.annotate("", xy=(6.0, 2.3), xytext=(2.6, 2.3), arrowprops=dict(arrowstyle="<|-|>", lw=1))
ax.text(4.3, 2.45, "timeout window  $T$  (no accepted command)", ha="center", fontsize=7.5)
ax.annotate("", xy=(8.2, 3.2), xytext=(6.0, 3.2), arrowprops=dict(arrowstyle="<|-|>", lw=1))
ax.text(7.1, 3.35, "stop lag (servo response)", ha="center", fontsize=7.5)
ax.plot([6.0, 6.0], [0.7, 3.2], "k", lw=0.7, ls=":")
ax.plot([2.6, 2.6], [0.7, 2.3], "k", lw=0.7, ls=":")
fig.savefig(OUT + "fig2_timing.png", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)

# ---------------- Fig 3: reference path + traces ----------------
fig, axs = plt.subplots(1, 3, figsize=(7.0, 2.9), sharey=True)
for ax, (d, T, ttl) in zip(axs, [(0.002, 0.25, "(a) 2 ms delay"), (0.200, 0.25, "(b) 200 ms delay"),
                                  (0.400, 2.0, "(c) 400 ms delay\n(timeout relaxed to 2 s)")]):
    o = run(delay=d, T=T, seed=0)
    ax.plot(PTS[:, 0], PTS[:, 1], color="0.6", lw=4, solid_capstyle="round", label="reference")
    ax.plot(o["P_arr"][:, 0], o["P_arr"][:, 1], "k", lw=1.0, label="actual")
    ax.set_title(ttl, fontsize=8); ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
axs[0].set_ylabel("y (mm)"); axs[0].legend(fontsize=7, frameon=False, loc="upper left")
fig.tight_layout()
fig.savefig(OUT + "fig3_paths.png", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)

# ---------------- Fig 4: error vs delay ----------------
fig, ax = plt.subplots(figsize=(6.0, 3.2))
ds = sorted(int(k) for k in R["E1b"].keys())
rms = [R["E1b"][str(d)]["rms"]["mean"] for d in ds]
rsd = [R["E1b"][str(d)]["rms"]["sd"] for d in ds]
mx = [R["E1b"][str(d)]["mx"]["mean"] for d in ds]
msd = [R["E1b"][str(d)]["mx"]["sd"] for d in ds]
ax.errorbar(ds, rms, yerr=rsd, fmt="o-", color="black", ms=4, lw=1, capsize=2, label="RMS error")
ax.errorbar(ds, mx, yerr=msd, fmt="s--", color="black", mfc="white", ms=4, lw=1, capsize=2, label="maximum error")
ax.axhline(10, color="black", lw=0.8, ls=":"); ax.text(605, 10.6, "RMS target 10 mm", fontsize=7, ha="right")
ax.axhline(20, color="black", lw=0.8, ls=":"); ax.text(605, 20.6, "maximum target 20 mm", fontsize=7, ha="right")
ax.axvline(250, color="black", lw=1.2, ls="-.")
ax.text(255, 48, "default timeout\n250 ms", fontsize=7, va="top")
ax.set_xlabel("one-way command delay (ms)"); ax.set_ylabel("cross-track error (mm)")
ax.legend(frameon=False, fontsize=7.5, loc="upper left"); ax.set_xlim(0, 620); ax.set_ylim(0, 62)
fig.tight_layout(); fig.savefig(OUT + "fig4_error_vs_delay.png", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)

# ---------------- Fig 5: disconnect traces ----------------
fig, ax = plt.subplots(figsize=(6.5, 3.0))
CUTT = 15.0
oa = run(delay=0.05, T=0.25, seed=0, cut=(CUTT, 1e9))
ob = run(delay=0.05, T=0.25, seed=0, cut=(CUTT, 1e9), watchdog=False, age_check=False)
tt = np.arange(len(oa["spd_arr"])) * oa["dt"]
m = (tt > 13.5) & (tt < 19.0)
ax.plot(tt[m], ob["spd_arr"][m], color="0.55", lw=1.6, label="no watchdog (baseline)")
ax.plot(tt[m], oa["spd_arr"][m], "k", lw=1.3, label="watchdog, T = 250 ms")
ax.axvline(CUTT, color="black", ls=":", lw=1); ax.text(CUTT + 0.05, 47, "link cut", fontsize=7, va="top")
trip = [s for s in oa["stops"] if s > 14][0]
ax.axvline(trip, color="black", ls="--", lw=1); ax.text(trip + 0.05, 33, "watchdog trips", fontsize=7)
ax.set_xlabel("time (s)"); ax.set_ylabel("tool speed (mm/s)"); ax.set_ylim(0, 52)
ax.legend(frameon=False, fontsize=7.5, loc="upper right")
fig.tight_layout(); fig.savefig(OUT + "fig5_disconnect.png", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)

# ---------------- Fig 6: timeout trade-off ----------------
fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.9))
Ts = [100, 150, 250, 400, 600]
ov = [R["E3a"][str(T)]["overrun_to_stop_mm"]["mean"] for T in Ts]
ovs = [R["E3a"][str(T)]["overrun_to_stop_mm"]["sd"] for T in Ts]
axs[0].errorbar(Ts, ov, yerr=ovs, fmt="o-", color="black", ms=4, lw=1, capsize=2)
axs[0].axhline(R["E3a"]["none"]["dist_3s_mm"]["mean"], color="black", ls="--", lw=1)
axs[0].text(590, R["E3a"]["none"]["dist_3s_mm"]["mean"] - 9, "no watchdog (first 3 s): %.0f mm" %
            R["E3a"]["none"]["dist_3s_mm"]["mean"], fontsize=7, ha="right")
axs[0].set_xlabel("timeout T (ms)"); axs[0].set_ylabel("overrun after link cut (mm)")
axs[0].set_title("(a) distance travelled after a real cut", fontsize=8); axs[0].set_ylim(0, 140)
rate = [100 * R["E3b"][str(T)]["rate"] for T in Ts]
lo = [100 * R["E3b"][str(T)]["lo"] for T in Ts]; hi = [100 * R["E3b"][str(T)]["hi"] for T in Ts]
axs[1].errorbar(Ts, rate, yerr=[np.array(rate) - np.array(lo), np.array(hi) - np.array(rate)],
                fmt="s-", color="black", mfc="white", ms=4, lw=1, capsize=2)
axs[1].set_xlabel("timeout T (ms)"); axs[1].set_ylabel("runs with a false stop (%)")
axs[1].set_title("(b) false stops under synthetic drop-outs", fontsize=8); axs[1].set_ylim(0, 105)
fig.tight_layout(); fig.savefig(OUT + "fig6_timeout_tradeoff.png", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)

# ---------------- Fig 7: recovery ----------------
fig, axs = plt.subplots(2, 1, figsize=(6.5, 3.8), sharex=True)
CUT = (12.5, 15.5); REARM = 17.5
for ax, (ttl, kw) in zip(axs, [("(a) latched stop + manual re-arm (proposed)", dict(latch=True, rearm_time=REARM)),
                               ("(b) automatic resume (no latch)", dict(latch=False))]):
    o = run(delay=0.05, T=0.25, seed=0, cut=CUT, t_end=30.0, **kw)
    tt = np.arange(len(o["spd_arr"])) * o["dt"]
    m = (tt > 11) & (tt < 25)
    ax.plot(tt[m], o["spd_arr"][m], "k", lw=1.2)
    ax.axvspan(CUT[0], CUT[1], color="0.85", lw=0)
    ax.text((CUT[0] + CUT[1]) / 2, 44, "link down", ha="center", fontsize=7)
    ax.set_ylabel("speed (mm/s)"); ax.set_title(ttl, fontsize=8, loc="left"); ax.set_ylim(0, 55)
    if kw.get("rearm_time"):
        ax.axvline(REARM, color="black", ls="--", lw=1); ax.text(REARM + 0.15, 6, "operator re-arms", fontsize=7)
axs[1].set_xlabel("time (s)")
fig.tight_layout(); fig.savefig(OUT + "fig7_recovery.png", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)

# ---------------- Fig 8: Gantt ----------------
tasks = [("Scope, sources, requirements", 1, 1), ("Comparison of tools and devices", 2, 2),
         ("Architecture and safety design", 3, 3), ("Test plan, resources, cost", 4, 4),
         ("Draft, review, slides", 5, 5), ("Environment set-up (+ buffer)", 6, 7),
         ("Teleoperation and path", 7, 8), ("Limits, watchdog, logging", 8, 8),
         ("Test runs and analysis", 9, 9), ("Presentation and feedback", 10, 10)]
fig, ax = plt.subplots(figsize=(6.5, 3.0))
for i, (nm, a, b) in enumerate(tasks):
    hatch = "///" if "buffer" in nm else None
    ax.barh(len(tasks) - i, b - a + 1, left=a - 0.5, height=0.55, color="0.35" if not hatch else "white",
            edgecolor="black", hatch=hatch, lw=0.8)
ax.set_yticks([len(tasks) - i for i in range(len(tasks))]); ax.set_yticklabels([t[0] for t in tasks], fontsize=7.5)
ax.set_xticks(range(1, 11)); ax.set_xlabel("project day"); ax.set_xlim(0.5, 10.5)
ax.axvline(5.5, color="black", ls="--", lw=1); ax.text(3, 10.9, "Week 1", ha="center", fontsize=8)
ax.text(8, 10.9, "Week 2", ha="center", fontsize=8); ax.set_ylim(0.3, 11.4)
fig.tight_layout(); fig.savefig(OUT + "fig8_gantt.png", bbox_inches="tight", pad_inches=0.05)
plt.close(fig)
print("figs ok")