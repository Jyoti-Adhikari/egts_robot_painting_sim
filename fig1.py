import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

plt.rcParams.update({"font.family": "DejaVu Serif", "savefig.dpi": 200,
                     "figure.facecolor": "white", "savefig.facecolor": "white"})

fig, ax = plt.subplots(figsize=(7.4, 4.6))
ax.set_xlim(0, 17); ax.set_ylim(-0.6, 9.2); ax.axis("off")


def box(x, y, w, h, text, lw=1.0, ls="-", fs=7.2, fill="white", bold_first=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.0,rounding_size=0.12",
                                fc=fill, ec="black", lw=lw, ls=ls))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, linespacing=1.35)


def arrow(x1, y1, x2, y2, lw=1.0, ls="-"):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="-|>", lw=lw, ls=ls, color="black",
                                shrinkA=0, shrinkB=0, mutation_scale=9))


# regions
ax.add_patch(Rectangle((0.1, 0.0), 4.9, 8.5, fc="white", ec="black", lw=1.0, ls=(0, (6, 3))))
ax.text(2.55, 8.15, "OPERATOR STATION", ha="center", va="center", fontsize=7.8, fontweight="bold")
ax.add_patch(Rectangle((7.2, 0.0), 9.7, 8.5, fc="#ececec", ec="black", lw=2.2))
ax.text(12.05, 8.15, "LOCAL SIMULATION HOST", ha="center", va="center", fontsize=7.8, fontweight="bold")
ax.text(12.05, 7.72, "safety does not depend on the link", ha="center", va="center", fontsize=6.6, style="italic")

# operator side
box(0.5, 5.9, 4.1, 1.2, "Operator\nkeyboard or gamepad")
box(0.5, 3.7, 4.1, 1.6, "Input processing\ndead zone, scaling,\ntimestamp, sequence no.")
box(0.5, 0.9, 4.1, 1.6, "Operator dashboard\nstatus, command age,\nstop and re-arm state")
arrow(2.55, 5.9, 2.55, 5.3)

# network band
ax.add_patch(Rectangle((5.5, 0.9), 1.2, 4.9, fc="white", ec="black", lw=1.0, ls=(0, (2, 2))))
ax.text(6.1, 3.35, "Network\n(ROS 2\ntopics)", ha="center", va="center", fontsize=6.8, rotation=90, linespacing=1.3)
arrow(4.6, 4.5, 7.7, 4.5)
ax.text(5.1, 4.72, "commands", ha="center", fontsize=6.6)
arrow(7.7, 1.7, 4.6, 1.7)
ax.text(5.1, 1.92, "feedback", ha="center", fontsize=6.6)

# host side
box(7.7, 3.6, 3.7, 1.8, "LOCAL SAFETY LAYER\nage and order check,\nbounds, speed limit,\nwatchdog, latched stop", lw=2.4, fs=6.6)
box(12.6, 3.6, 3.7, 1.8, "Motion control\nMoveIt 2 Servo,\npath tracking")
arrow(11.4, 4.5, 12.6, 4.5)
ax.text(12.0, 4.72, "accepted", ha="center", fontsize=6.6)
box(7.7, 6.0, 3.7, 1.2, "LOCAL STOP BUTTON\n(no network path)", lw=2.4)
arrow(9.55, 6.0, 9.55, 5.4, lw=1.8)
box(12.6, 0.9, 3.7, 1.6, "Simulated 6-axis arm\nand virtual surface")
arrow(14.45, 3.6, 14.45, 2.5)
box(7.7, 0.9, 3.7, 1.6, "Logger\nlatency, path error,\nrejections, stops")
arrow(12.6, 1.7, 11.4, 1.7)
ax.text(12.0, 1.92, "state", ha="center", fontsize=6.6)

ax.text(0.1, -0.35, "Dashed outline: link side, not trusted for safety.   Heavy outline: functions that must work with the link down.",
        fontsize=6.4, ha="left")
fig.savefig("figs/fig1_architecture.png", bbox_inches="tight", pad_inches=0.05)