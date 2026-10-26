# AGENTS.md: Project context for AI agents and new team members

Keep this file in the root of the repository (`egts_robot_painting_sim/`). Read it fully before changing any code or text. It explains **what** this project is, **how** the code works, and **why** each choice was made.

Many tools read this file name automatically. If your tool expects another name (for example `CLAUDE.md`), copy this file to that name.

---

## 1. What this project is

**Name:** Remote-Controlled Robotic Painting
**Programme:** EGTS Physical AI and Robotics Research Project (2 weeks)
**Team:** EGTS Engineering Team
**Type:** Research and simulation only.

**Goal.** Study how an operator can control a robotic painting arm from far away, using a keyboard, gamepad or similar device, and how to keep it **safe when the network is slow or breaks**.

**Research question.** How can a low-cost teleoperation design move a simulated arm along a set painting path, keep the path accurate, and stay safe under network delay or disconnection?

**Deliverables in the brief:** a 3 to 5 page proposal, an architecture diagram, a development time and cost estimate, and a 10-minute presentation. The paper is graded on a 100-point rubric (research quality 20, technical design 25, problem-solving 20, project planning 20, presentation 15). The target is 70 or more **and** every mandatory safety requirement met.

**Guiding principle from the brief:** search before building, reuse proven technology, use realistic costs, design for safety, measure results, share what we learn.

---

## 2. Rules that must never be broken

1. **Simulation only.** No real spraying, no paint, no powder, no industrial robot, no hardware control code. Do not add code that talks to a real robot, even "for testing".
2. **Safety stays local.** A remote message must never be the only thing that can stop the robot. The watchdog, the latched stop and the stop button live on the local simulation host.
3. **Never invent facts or numbers.** Every number in the paper either comes from `results.json` (our simulation), from a cited source, or is clearly marked as "our choice" or "planning estimate". If you cannot source a claim, remove it or mark it as an assumption.
4. **Do not oversell the results.** They come from a simple Python model, not ROS 2 and not a real robot. Always say so near any result.
5. **Do not claim a score.** The graders give the score.
6. **A software stop is not a certified emergency stop.** Say this wherever the stop is described.
7. **Keep figures black and white.** The paper is black and white. Use black, white and greys with different line styles and markers.
8. **Keep language simple and clear.** Short sentences. Explain terms.
9. **Keep the random seeds.** Seeds make results repeatable. Do not change them without saying so.

---

## 3. Repository layout

```
egts_robot_painting_sim/
├── AGENTS.md           this file
├── README.md           (optional) short run instructions
├── requirements.txt    numpy, matplotlib
├── sim.py              the model and metric functions
├── experiments.py      runs all experiments, writes results.json
├── figs.py             draws figures 2 to 8 (and an older figure 1)
├── fig1.py             draws the final architecture figure
├── results.json        created by experiments.py (keep in Git)
└── figs/               created by figs.py (keep in Git)
```

Tested with Python 3.12.3, NumPy 2.4.4, Matplotlib 3.10.8.

**How to run (from the repository root):**

```
pip install numpy matplotlib
python experiments.py     # writes results.json (takes a short while)
python figs.py            # writes figs/fig1 ... fig8 (needs results.json)
python fig1.py            # redraws the final architecture figure; run AFTER figs.py
```

Use `python3` on Mac or Linux.

---

## 4. What the system is (concept)

The design has two sides:

- **Operator station.** Keyboard or gamepad. Input is cleaned (dead zone), scaled, time-stamped and numbered. This side is **not trusted for safety**.
- **Local simulation host.** A **local safety layer** checks every command, then passes it to motion control and the simulated arm. A **logger** records everything. A **local stop button** has no network path. This side is **trusted for safety**.

Messages travel between them over the **network** (ROS 2 in the planned build). Feedback (position and status) returns to the operator.

**Local safety layer rules**

- A command is accepted only if it is **new** (its sequence number is higher than the last accepted one), **fresh** (its age is at most the timeout T), and the system is **ARMED**.
- Commands are clipped to the speed limit (50 mm/s).
- A **watchdog** trips if no command has been accepted for T. It sets the commanded velocity to zero and **latches** the stop (state STOPPED).
- In STOPPED, every command is rejected. Late messages are dropped, never replayed.
- Only an explicit **operator re-arm** returns to ARMED. After re-arm the operator resumes from the path point **nearest to the tool**, not from where the clock says the tool should be.

**Why this design**

- Remote links fail. Safe behaviour must not depend on receiving another message, so the watchdog is local.
- "Stop" and "hold" are different. We define the stop exactly (velocity command to zero, then servo decay to standstill below 1 mm/s).
- Without a latch, the tool restarts by itself when the link returns and moves toward where the clock says it should be. That can cut across the path. The latch plus manual re-arm prevents unintended movement.
- ROS 2 and MoveIt 2 were chosen because they are free, show each step of the control path, and MoveIt Servo already gives collision checks, singularity handling and a command-timeout parameter.

---

## 5. How the code works

### `sim.py` (the model)

A **reference model**, not ROS 2. It is the "small standalone simulated control model" from the project's backup plan.

- **Plant.** A tool point `p` on a flat plane, velocity controlled with a first-order servo lag: `tau * dv/dt = v_cmd - v`, `dp/dt = v`, with `|v_cmd| <= v_max`.
- **Path.** A raster of 5 horizontal strokes, 240 mm long, 40 mm apart, joined by 40 mm transitions. Total length 1,360 mm. Constants: `STROKES, LEN, SPACING`.
- **Operator.** A simple controller, not a human model: `u = v_ref * direction + K * (r(t) - p_seen) + noise`. `p_seen` is the delayed tool position. It does not reproduce human move-and-wait behaviour.
- **Network.** Each message gets delay `max(1 ms, d0 + J * N(0,1))` with jitter J = 10% of d0. Messages can be dropped (independent loss), reordered, cut by a link-cut window, or blocked by synthetic drop-outs.
- **Safety layer.** Order check, age check, disarmed check, speed clip, watchdog, latch, optional manual re-arm.
- **Main function:** `run(...)`. Key arguments: `delay, jitter_frac, loss, T, seed, vref, vmax, K, tau, noise, dt, op_dt, t_end, cut, blackouts, bo_rate, bo_mean, watchdog, age_check, latch, rearm_time`.
- **Helpers:** `ref_pos_vel`, `closest_s` (nearest path point), `cross_track` (error to the path), `summarize_tracking` (RMS and max error).

### `experiments.py` (the tests)

| ID | What it tests | Variable | Runs |
|---|---|---|---|
| E1 | Tracking with T = 250 ms | Delay 2, 50, 100, 150, 200, 250, 300 ms | 30 each |
| E1b | Tracking with T = 2 s (isolates tracking from stopping) | Delay 2 to 600 ms | 30 each |
| E2 | Packet loss at 100 ms delay | Loss 0 to 40% | 30 each |
| E3a | Permanent link cut at 15 s | T = 100, 150, 250, 400, 600 ms, and no watchdog | 30 each |
| E3b | False stops under synthetic drop-outs | Same T values | 100 each |
| E4 | 3 s link cut (12.5 to 15.5 s), re-arm at 17.5 s | Latched stop vs automatic resume | 30 each |
| E5 | Reference speed | 10 to 45 mm/s | 30 each |
| E6 | Operator gain K and servo lag tau | K = 1, 2, 4; tau = 20, 50, 100 ms | 30 each |

Seeds: 0 to 29 (E3b uses 1000 to 1099). Results are saved to `results.json`.

### `figs.py` and `fig1.py` (the figures)

`figs.py` reads `results.json` and runs the model again to draw figures 2 to 8:
fig2 timing, fig3 paths, fig4 error vs delay, fig5 disconnect, fig6 timeout trade-off, fig7 recovery, fig8 schedule. It also draws an **older, rougher** figure 1. Run `fig1.py` **after** it so the final architecture figure replaces the old one.

---

## 6. Metric definitions (use these exactly)

- **Command age** `a = t_receive - t_send`. In the model both use one clock. In a real test, clocks must be synchronised, otherwise report round-trip time.
- **Cross-track error** `e_i = distance from the tool to the nearest point on the reference path`. It measures where the tool is, not when it gets there.
- **RMS error** `sqrt(mean(e^2))` and **max error** `max(e)`.
- **Detection time.** Link cut to watchdog trip.
- **Standstill time.** Link cut until tool speed is below 1 mm/s.
- **Overrun.** Path length travelled from the cut to standstill.
- **False-stop rate.** Share of runs with a watchdog trip when no real fault was injected (95% Wilson interval).
- **Movement before re-arm.** Path length between link return and operator re-arm.

**Targets set before testing (our choices, not standards):** RMS 10 mm or less, max 20 mm or less, speed never above 50 mm/s, old commands always rejected, stop on link cut, no movement after reconnection until re-arm, every test logged.

---

## 7. Results so far (reference model only)

Mean and standard deviation over 30 runs. The standard deviation reflects only the model's random inputs, not real-world spread.

**Tracking against delay, T = 250 ms (E1)**

| Delay (ms) | Runs finished | RMS (mm) | Max (mm) | Both targets met |
|---|---|---|---|---|
| 2 | 30/30 | 0.52 | 2.01 | 30/30 |
| 50 | 30/30 | 1.55 | 5.83 | 30/30 |
| 100 | 30/30 | 2.83 | 10.01 | 30/30 |
| 150 | 30/30 | 4.09 | 14.21 | 30/30 |
| 200 | 30/30 | 5.48 | 18.32 | 30/30 |
| 250 | 11/30 | 7.28 | 23.58 | 0/11 |
| 300 | 0/30 | no finished run | no finished run | not applicable |

With T = 2 s (E1b): 300 ms gives 8.79 RMS and 26.98 max; 400 ms gives 13.50 and 39.06; 600 ms gives 19.16 and 55.95.

**Link cut and timeout (E3a, E3b)**

| T (ms) | Detection after cut (ms) | Standstill after cut (ms) | Overrun (mm) | False-stop rate (100 runs) |
|---|---|---|---|---|
| 100 | 132 | 306 | 7.1 | 97% |
| 150 | 182 | 356 | 9.2 | 88% |
| 250 | 282 | 456 | 13.3 | 48% |
| 400 | 432 | 606 | 19.5 | 11% |
| 600 | 632 | 806 | 27.7 | 2% |
| No watchdog | not applicable | not applicable | 123.9 in first 3 s | not applicable |

**Overrun formula** (matches the simulation): `d_overrun ≈ v * (delta + T + tau)`, with v = 40 mm/s, delta ≈ 0.032 s (in-flight messages) and tau = 0.05 s. It gives 7.3 mm at T = 100 ms and 13.3 mm at T = 250 ms. Solving for T: `T <= d_max / v - delta - tau`. A 20 mm budget gives T up to about 0.42 s. The budget should come from a risk assessment. We chose 20 mm.

**Recovery (E4)**

| Policy | Movement before re-arm (mm) | Max error after link return (mm) |
|---|---|---|
| Latched stop + manual re-arm | 0.00 | 6.15 |
| Automatic resume | 95.4 | 19.98 |

**Other results.** Independent packet loss up to 40% at 100 ms delay changed RMS from 2.83 to 3.03 mm and max from 10.01 to 11.20 mm, with no stops (E2). Error rises with speed: RMS 0.49 mm at 10 mm/s and 3.02 mm at 45 mm/s (E5). Across K and tau, RMS ranged from 2.27 to 3.86 mm and max from 9.12 to 11.63 mm at 100 ms delay (E6). Peak speed was 49.999 mm/s, and about 14% of commands were clipped to the limit.

**What the results mean.** Targets are met up to 200 ms delay in the model. The timeout trades overrun against false stops, so **250 ms is a start value, not a justified final value**. The latched stop prevents unintended movement. None of this is proof for a real system.

---

## 8. Settings and why they were chosen

| Setting | Value | Reason | What it is not |
|---|---|---|---|
| Local time step | 5 ms | Fine enough to measure stop timing | |
| Command rate | 50 Hz | A common teleoperation rate | Not measured on a real link |
| Reference speed | 40 mm/s | Below the 50 mm/s cap, leaves room to catch up | |
| Speed limit | 50 mm/s | One fifth of the 250 mm/s manual-mode limit in robot standards | Not from a risk assessment or human-factors data |
| Timeout T | 250 ms start | Sweep 100 to 600 ms. MoveIt Servo has a command-timeout parameter, with example values of 0.1 s and 1.0 s | Not a validated safety limit |
| Operator gain K | 2 per second | Simple stable choice | Varied in E6 |
| Servo lag tau | 50 ms | Simple servo response | Varied in E6 |
| Command noise | 3 mm/s per axis | Small random input to give run-to-run spread | Author's choice |
| Drop-outs (E3b) | 0.2 per second, mean 100 ms | Synthetic test input | **Not a real network measurement** |
| Path error targets | 10 mm RMS, 20 mm max | Pass line set before testing | Not an industry standard |
| Standstill threshold | 1 mm/s | Small value meaning "stopped" | |

---

## 9. Known limits and known issues

**Scientific limits**

- The model is not ROS 2 or MoveIt 2. It has no six-axis kinematics, no collision checking, no workspace bounds, no joystick and no real stop-button object.
- The operator is a simple controller, not a human.
- Delay, jitter, loss and drop-out settings are synthetic. False-stop rates only apply to them.
- Paint quality is **not** modelled. Do not claim coating quality from path accuracy.
- The literature review is limited and is not a systematic review.

**Code issues to fix when convenient**

- `figs.py` and `fig1.py` contain the path `/home/claude/figs/`. Change it to `figs/`. `fig1.py` does not create the folder, so run `figs.py` first or create `figs/` yourself.
- `figs.py` also draws an older figure 1. Always run `fig1.py` after it. A cleaner fix is to remove the figure 1 block from `figs.py`.
- `sim.py` has unused imports and variables (`json`, `sys`, `u_cur`, `stop_t`). They are harmless.

---

## 10. Planned next steps (in priority order)

1. **Repeat the tests in ROS 2 with MoveIt Servo** and a real gamepad. First fix and record every version (OS, ROS 2 distribution, DDS, MoveIt 2, simulator, robot model). Send `geometry_msgs/TwistStamped` at 50 Hz. Put the watchdog on the local host. Use best-effort QoS, depth 1, lifespan equal to the freshness window.
2. **Measure real drop-outs** on the real link and set the timeout from them.
3. **Add workspace bounds, collision checks and a local stop button** to the tests, and test the stop with the link down.
4. **Add a painting coverage metric.** Define footprint width `w` and stroke spacing `s`. `overlap = 1 - s/w`. `coverage = area covered / surface area`. Take `w` from a published deposition model or a measured pattern. Do not guess it.
5. **AI study (later, only after the base system works).** Predict cross-track error about 0.5 s ahead from recent commands, delayed position and command age. Compare with a non-AI baseline (constant-velocity extrapolation). Blend assistance with the operator's input. The AI may only slow the robot or steer toward the path. It may never raise the speed limit, skip the age check, or release a latched stop. A second idea, learning from demonstrations, is background only.
6. Physical testing is a **separate phase** and needs a qualified safety review.

---

## 11. Safety background (for context)

- Painting hazards named in the brief: ventilation (fumes, overspray), electrical hazards and ignition, and for powder coating, combustible dust and electrostatic ignition. OSHA 29 CFR 1910.107 covers spray finishing.
- Robot safety standards: ISO 10218-1 and -2 (revised 2025). ISO 10218-2:2025 does not cover use in potentially explosive environments, so spraying flammable materials needs extra requirements. FANUC's paint cobot is described as ATEX Cat. II 2G/2D.
- Emergency stop: ISO 13850. A real emergency stop must be safety-rated hardware, not a Python or ROS 2 watchdog. IEC 61508 is the general functional-safety standard.
- Risk method: ISO 12100 (find hazards, judge risk, reduce risk). The risk scale in the paper (severity 1 to 3, likelihood 1 to 3) is our own and describes a hypothetical physical deployment.

---

## 12. Key sources (do not cite anything not listed here without checking it)

- ROS 2: Macenski et al., *Science Robotics* 7(66), 2022. https://arxiv.org/abs/2211.07752
- MoveIt Servo documentation: https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html
- MoveIt 2 pull request #427 (servo timeout examples): https://github.com/moveit/moveit2/pull/427/files
- Isaac Sim requirements: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- CoppeliaSim licensing: https://coppeliarobotics.com/licensing
- Haply in Isaac Lab: https://isaac-sim.github.io/IsaacLab/develop/source/how-to/haply_teleoperation.html
- FANUC CRX Paint specs: https://pfonline.com/products/worlds-first-global-ex-proof-collaborative-paint-robot
- Hirebotics: https://www.robotics247.com/article/automate-2026-hirebotics-launches-no-code-explosion-proof-cobot-offering-for-painting/Hirebotics
- Teleoperation survey: Hokayem and Spong, *Automatica* 42(12), 2006.
- Delay and human performance: Korte et al., *Telemedicine and e-Health* 20(12), 2014.
- Shared control: Dragan and Srinivasa, *IJRR* 32(7), 2013. https://www.ri.cmu.edu/publications/a-policy-blending-formalism-for-shared-control
- Painting: Conner et al., *IEEE T-ASE* 2(4), 2005 (deposition); Atkar et al., *IJRR* 24(11), 2005 (coverage); Schaldenbrand et al., FRIDA, ICRA 2023.
- Learning from demonstration: Zhao et al., ALOHA, RSS 2023. https://roboticsproceedings.org/rss19/p016.html
- Standards and rules: ISO 12100:2010; ISO 13850:2015; ISO 10218-2:2025 (https://www.iso.org/standard/73934.html); ABB safety slides for the 250 mm/s limit (https://library.e.abb.com/public/188d2f386fbd4417a484ba4cf56b0df9/Safety_WS.pdf); OSHA 1910.107 (https://www.osha.gov/node/45911).
- Gamepad price: Logitech F310 at US$19.33 (SHI) and US$27.99 (Zones). Exchange rate: NPR 155.08 per US$ (Nepal Rastra Bank, 12 Oct 2026).

**Facts we removed because they could not be sourced:** the FANUC system price (FANUC says request a quote), the old NPR 140 rate, the old joystick range of US$20 to 40, the printing and internet cost, and the Lite 6 and Magician E6 arm specifications.

---

## 13. How to work on this repository

- **Before changing code:** run `experiments.py` and keep a copy of the old `results.json` so you can compare.
- **When changing a setting:** change one thing at a time, say why, and re-run all experiments. Update the table in section 8.
- **When adding a test:** give it a new ID (E7, E8, ...), set its seeds, define its metric in section 6, and save the numbers in `results.json`.
- **When writing about results:** copy numbers from `results.json`. Round the way the paper does (mean ± SD). State the number of runs. Say "in the model".
- **When adding a source:** open it, check that it supports the claim, and add it to section 12.
- **When something is uncertain:** write "not measured", "our choice" or "planning estimate". Do not fill the gap with a guess.

## 14. Glossary

- **Teleoperation:** controlling a machine from a distance.
- **Latency or delay:** time a message needs to arrive.
- **Jitter:** how much the delay changes from message to message.
- **Watchdog:** a timer that triggers a stop if expected messages stop arriving.
- **Latched stop:** a stop that stays on until a person resets it.
- **Re-arm:** the manual reset that allows motion again.
- **Freshness window:** the maximum allowed command age (T).
- **Overrun:** distance the tool still travels after the link is lost.
- **False stop:** a stop caused by a harmless drop-out, not a real fault.
- **Cross-track error:** distance from the tool to the nearest point on the planned path.
- **RMS:** root mean square, a typical size of the error.
- **Singularity:** an arm pose where control is lost or unstable.
- **QoS:** quality-of-service settings for ROS 2 messages.
- **ATEX:** European rules for equipment used in explosive atmospheres.