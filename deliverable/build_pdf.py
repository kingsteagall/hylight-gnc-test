"""Build the submission PDF: GNC-Technical-Test-Answers.pdf.

Fonts: DejaVu (shipped with matplotlib) registered for full unicode
(alpha/beta, arrows, norms). Figures come from ../analysis and ./ (figures.py).
"""
from pathlib import Path

import matplotlib
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, PageBreak, Paragraph, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

HERE = Path(__file__).parent
AN = HERE.parent / "analysis"

FT = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
pdfmetrics.registerFont(TTFont("DVS", str(FT / "DejaVuSans.ttf")))
pdfmetrics.registerFont(TTFont("DVS-B", str(FT / "DejaVuSans-Bold.ttf")))
pdfmetrics.registerFont(TTFont("DVS-I", str(FT / "DejaVuSans-Oblique.ttf")))
pdfmetrics.registerFont(TTFont("DVM", str(FT / "DejaVuSansMono.ttf")))

INK = colors.HexColor("#1a2733")
BLUE = colors.HexColor("#2563eb")
ORANGE = colors.HexColor("#ea580c")
GRAYBG = colors.HexColor("#f6f8fa")
BLUEBG = colors.HexColor("#eff6ff")

S = {
    "title": ParagraphStyle("t", fontName="DVS-B", fontSize=22, leading=27,
                            textColor=INK, spaceAfter=2),
    "sub": ParagraphStyle("s", fontName="DVS", fontSize=11, leading=15,
                          textColor=colors.HexColor("#475569"), spaceAfter=10),
    "h1": ParagraphStyle("h1", fontName="DVS-B", fontSize=15.5, leading=19,
                         textColor=BLUE, spaceBefore=8, spaceAfter=5),
    "h2": ParagraphStyle("h2", fontName="DVS-B", fontSize=12, leading=15,
                         textColor=INK, spaceBefore=9, spaceAfter=3),
    "body": ParagraphStyle("b", fontName="DVS", fontSize=9.6, leading=13.6,
                           textColor=INK, spaceAfter=4),
    "li": ParagraphStyle("li", fontName="DVS", fontSize=9.6, leading=13.4,
                         textColor=INK, leftIndent=14, bulletIndent=4,
                         spaceAfter=2.5),
    "code": ParagraphStyle("c", fontName="DVM", fontSize=8.2, leading=11.4,
                           textColor=INK, backColor=GRAYBG,
                           borderPadding=(5, 7, 5, 7), spaceAfter=6,
                           borderColor=colors.HexColor("#d0d7de"),
                           borderWidth=0.7, borderRadius=3),
    "cap": ParagraphStyle("cap", fontName="DVS-I", fontSize=8.6, leading=11.5,
                          textColor=colors.HexColor("#64748b"), spaceAfter=6,
                          alignment=1),
}


def P(text, style="body", **kw):
    st = S[style]
    if kw:
        st = ParagraphStyle("x", parent=st, **kw)
    return Paragraph(text, st)


def LI(text):
    return Paragraph(f"<bullet>&bull;</bullet>{text}", S["li"])


def CODE(text):
    return Paragraph(text.replace("&", "&amp;").replace("<", "&lt;")
                     .replace(" ", "&nbsp;").replace("\n", "<br/>"), S["code"])


def callout(text, bg=BLUEBG, border=BLUE):
    t = Table([[P(text)]], colWidths=[17 * cm])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), bg),
        ("BOX", (0, 0), (-1, -1), 1, border),
        ("LEFTPADDING", (0, 0), (-1, -1), 9),
        ("RIGHTPADDING", (0, 0), (-1, -1), 9),
        ("TOPPADDING", (0, 0), (-1, -1), 6),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
        ("ROUNDEDCORNERS", [4, 4, 4, 4]),
    ]))
    return t


def fig(path, width, caption=None):
    from PIL import Image as PILImage
    w0, h0 = PILImage.open(path).size
    els = [Image(str(path), width=width, height=width * h0 / w0)]
    if caption:
        els.append(P(caption, "cap"))
    return els


def table(data, widths, header=True, fontsize=8.8, align="CENTER"):
    al = 1 if align == "CENTER" else 0
    cell = ParagraphStyle("cell", fontName="DVS", fontSize=fontsize,
                          leading=fontsize * 1.35, textColor=INK)
    cellc = ParagraphStyle("cellc", parent=cell, alignment=al)
    cellb = ParagraphStyle("cellb", parent=cell, fontName="DVS-B", alignment=al)
    wrapped = [[Paragraph(str(c), cellb if (header and i == 0)
                          else (cell if j == 0 else cellc))
                for j, c in enumerate(row)] for i, row in enumerate(data)]
    t = Table(wrapped, colWidths=widths)
    style = [
        ("FONTNAME", (0, 0), (-1, -1), "DVS"),
        ("FONTSIZE", (0, 0), (-1, -1), fontsize),
        ("TEXTCOLOR", (0, 0), (-1, -1), INK),
        ("ALIGN", (1, 0), (-1, -1), align),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("GRID", (0, 0), (-1, -1), 0.6, colors.HexColor("#cbd5e1")),
        ("TOPPADDING", (0, 0), (-1, -1), 3.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, GRAYBG]),
    ]
    if header:
        style += [("FONTNAME", (0, 0), (-1, 0), "DVS-B"),
                  ("BACKGROUND", (0, 0), (-1, 0), BLUEBG)]
    t.setStyle(TableStyle(style))
    return t


story = []

# ================================================================= cover head
story += [
    P("GNC Technical Test — Answers", "title"),
    P("David Steagall  ·  July 2026", "sub"),
    callout(
        "<b>Exercise 1</b> — closed-form control allocation, delivered as a "
        "C++17 class with a demo script and 10 property tests (all green)."
        "<br/><b>Exercise 2</b> — drift diagnosis from the Annex B log, and a "
        "course-based guidance fix, <b>measured</b>: ~3× less cross-track "
        "error on two independent simulations. Full sources ship alongside "
        "this document."),
    Spacer(1, 10),
]

# ================================================================= Exercise 1
story += [
    P("Exercise 1 — Control allocation", "h1"),
    P("1.1 · The actuator, in one picture", "h2"),
    P("Each gyro is a thrust vector on a 2-axis gimbal: the exterior servo "
      "(α, the outer gimbal — it carries the other one) tilts the thrust "
      "fore/aft; the interior servo (β) tilts it left/right. Neutral points "
      "up. Composing the two rotations, outer applied last:"),
]
story += fig(HERE / "conventions.png", 17 * cm)
story += [
    CODE("t(α, β)  =  R_y(α) · R_x(β) · (0, 0, −1)  =  "
         "( −sin α · cos β ,   sin β ,   −cos α · cos β )"),
    P("Sanity-checked against all three Annex A reference figures:"),
    table([
        ["α", "β", "t(α, β)", "Annex A says"],
        ["0°", "0°", "(0, 0, −1) — straight up", "neutral position  ✓"],
        ["90°", "0°", "(−1, 0, 0) — backward", "force to the back  ✓"],
        ["0°", "−90°", "(0, −1, 0) — left", "force to the left  ✓"],
    ], [1.8 * cm, 1.8 * cm, 6.6 * cm, 6.4 * cm]),
    Spacer(1, 4),
    P("<i>Stated assumption: the exterior servo is the outer gimbal (that is "
      "what the gyro photo shows). The three single-angle figures cannot "
      "distinguish the order; if reversed, only the two per-pod angle "
      "formulas swap — nothing else changes.</i>", fontSize=8.8),
    PageBreak(),
]

story += [
    P("1.2 · The accounting: five equations, six unknowns", "h2"),
    P("Pod forces f = T·t(α,β) at r_f = (+Lf, 0, 0) and r_r = (−Lr, 0, 0); "
      "moments at G via M = r × f. The demand gives five equations:"),
    CODE("Fx = fx,f + fx,r                (axial)\n"
         "Fz = fz,f + fz,r                (vertical pair)      "
         "My = −Lf·fz,f + Lr·fz,r\n"
         "0  = fy,f + fy,r                (no side force)      "
         "Mz = +Lf·fy,f − Lr·fy,r"),
    P("Both pods sit on the centerline, so Mx ≡ 0 falls out for free — "
      "consistent with roll being uncontrolled. Six unknowns minus five "
      "equations leaves <b>one free degree of freedom: the axial split</b>, "
      "resolved by splitting Fx equally (symmetric; exactly minimum-peak-"
      "thrust when the pods carry equal transverse loads). The closed form, "
      "with L = Lf + Lr:"),
    CODE("fx,f = fx,r = Fx′/2\n"
         "fz,f = ( Lr·Fz′ − My′ ) / L         fz,r = ( Lf·Fz′ + My′ ) / L\n"
         "fy,f = + Mz′ / L                    fy,r = − Mz′ / L        "
         "(pure yaw couple)"),
    P("Physics check: an upward front thrust (fz,f &lt; 0, z is down) alone "
      "gives My &gt; 0 — nose up. A rightward front force gives Mz &gt; 0 — "
      "nose right. Both correct. Per pod, the inversion back to actuator "
      "space is exact and iteration-free:"),
    CODE("T = ‖f‖          β = asin( fy / T )          α = atan2( −fx , −fz )"),

    P("1.3 · Constraint 1 — normalized, unit-free demands", "h2"),
    P("The allocator declares its own scaling, so a unit command on any axis "
      "means «the most this airframe can do on that axis»:"),
    LI("|Fx| = 1 or |Fz| = 1  ⇔  both motors at full thrust along that axis "
       "(scale 2·Tmax); thrust is expressed as a fraction of Tmax."),
    LI("|My| = 1 or |Mz| = 1  ⇔  the full antisymmetric pair "
       "(scale (Lf+Lr)·Tmax)."),
    LI("Combined demands can exceed the per-pod envelope ‖f‖ ≤ Tmax (the "
       "feasible set is coupled, not a unit box). Then <b>both</b> pods are "
       "scaled by one factor: the wrench shrinks but keeps its direction — "
       "saturation never invents a moment the controller didn't ask for. "
       "The achieved fraction is reported back (lastScale) so upstream "
       "integrators can anti-windup against reality."),

    P("1.4 · Constraint 2 — no abrupt servo motion", "h2"),
    P("The inversion is memoryless; continuity is a stateful post-processing "
      "layer, stage 4 of the pipeline below. The guiding rule, learned the "
      "hard way on a simulator: <i>any discontinuity fed to a vectoring "
      "actuator under load becomes a transient disturbance — sweep slowly, "
      "with thrust faded, and restore on arrival.</i> A full thrust reversal "
      "becomes: fade → sweep 180° at servo rate → restore (≈1.5 s)."),
    PageBreak(),
]

story += [
    P("1.5 · The whole allocator on one page", "h2"),
]
story += fig(HERE / "flowchart_alloc.png", 14.5 * cm,
             "Stages 1–3: pure stateless algebra. Stage 4 owns all memory. "
             "NaN/Inf demands are rejected at the door — the last command holds.")

story += [
    P("1.6 · Task 2 — the C++ deliverable", "h2"),
    table([
        ["file", "what it is"],
        ["src/ControlAllocator.hpp",
         "the class (header-only C++17, no deps): allocate(Wrench) → "
         "ActuatorCommand; forward() model; state(), lastScale()"],
        ["src/demo.cpp",
         "the demonstration script: 8 steady demands, config variants "
         "(asymmetric arms, servo hard stops), NaN robustness, reversal & "
         "zero-thrust showcases"],
        ["src/tests.cpp",
         "10 property tests — exact reconstruction (1000 random demands, "
         "symmetric + asymmetric geometry), saturation colinearity, slew "
         "compliance over 20 000 adversarial ticks, hand-computed inversion "
         "check, NaN recovery, noise freeze, hard stops, easing effect"],
    ], [4.6 * cm, 12.0 * cm], fontsize=8.4, align="LEFT"),
    Spacer(1, 6),
    CODE("g++ -std=c++17 -O2 src/demo.cpp  -o demo  &&  ./demo\n"
         "g++ -std=c++17 -O2 src/tests.cpp -o tests &&  ./tests     "
         "# → ALL TESTS PASSED"),
    P("Demo output (excerpt) — every case prints the command <i>and</i> the "
      "wrench rebuilt by the forward model:"),
    CODE("climb (Fz=-0.6)      front: T=0.600 α=  +0.0° β= +0.0°   "
         "rear: T=0.600 α=  +0.0° β= +0.0°\n"
         "cruise+climb+yaw     front: T=0.640 α= -45.0° β=+27.9°   "
         "rear: T=0.640 α= -45.0° β=-27.9°\n"
         "saturating demand    produced F=(+0.53,-0.53) M=(+0.27,+0.27)  "
         "→ 53%, direction kept\n"
         "reversal Fx +0.8→-0.8:  thrust fades to 0, α sweeps -90°→+90° "
         "at 120°/s, thrust returns"),
    P("Design choices an interviewer may probe:", "h2"),
    LI("<b>Closed form, not a QP</b> — for this plant it <i>is</i> the "
       "pseudo-inverse: same answer, fixed WCET, saturation handled "
       "explicitly instead of emerging from a solver."),
    LI("<b>Stateful class</b> — continuity is a property <i>between</i> "
       "commands; the allocator remembers the servo pose."),
    LI("<b>Real-time honest</b> — no heap, no STL containers, noexcept, "
       "O(1); double precision, mechanically portable to float."),
    PageBreak(),
]

# ================================================================= Exercise 2
story += [
    P("Exercise 2 — Mission mode", "h1"),
    P("2.1 · Q1 — why it drifts off the line", "h2"),
    P("Everything below is measured from the Annex B log "
      "(365 s: altitude → mission 33–263 s → position → altitude; every "
      "number reproduces via <font face='DVM'>analysis/extract_log.py</font>)."),
    P("<b>Physical</b>", spaceBefore=5),
    LI("<b>No lateral force (Fy = 0)</b> — pointing the hull is the only "
       "lateral authority the vehicle has."),
    LI("<b>Wind ≈ half the airspeed.</b> Cruise runs at 2.24 m/s against a "
       "3.0 m/s setpoint; in position mode gusts push the ship "
       "<b>backwards to −2.4 m/s</b>."),
    LI("<b>Slender-body heading hunt.</b> Yaw error: RMS 10°, peaks 41°, "
       "~30 s period. Each degree of nose error at cruise ≈ 4 cm/s of "
       "sideways velocity → 0.4–1.4 m/s of drift rate."),
    LI("<b>Huge inertia, low yaw authority.</b> The waypoint turnaround "
       "takes 45 s for 261° (~5.8°/s) — long stretches flown off-course."),
    P("<b>Algorithmic</b>", spaceBefore=5),
    LI("<b>The PID tracks heading, not course.</b> It tracks its setpoint "
       "reasonably well — and the ship still drifts, because in wind the "
       "velocity vector is not where the nose points."),
    LI("<b>Bearing-chasing guidance converges to the point, not the line</b> "
       "— the classic pursuit curve, bowing downwind; cross-track error is "
       "never penalized."),
    LI("<b>±180° wrap bug: the yaw setpoint flips 22 times</b> in the log "
       "when the desired heading sits near south; the vehicle winds through "
       "full circles chasing it."),
    LI("<b>The loop cannot even see the error.</b> The log has no lateral "
       "position, no vy, no course, no cross-track, no actuator commands — "
       "the drift is only inferable. (That is the honest answer to whether "
       "Annex B «contains enough information»: not quite — and that gap is "
       "part of the problem. Altitude and pitch meanwhile track fine: the "
       "problem lives entirely in the horizontal plane.)"),
]
story += fig(AN / "flight_zoom.png", 15.2 * cm,
             "Annex B, t = 33–150 s: first leg flown in reverse (vx setpoint "
             "≈ −2.3 m/s), then the turnaround — the yaw setpoint (red) "
             "thrashes across ±180° while the vehicle grinds through a 261° "
             "rotation in 45 s.")
story += [PageBreak()]

story += [
    P("2.2 · Q2 — control the track, not the nose", "h2"),
    P("Realistic with this actuator set: bounded corrections, no wind "
      "sensor, no perfect cancellation — the crab angle <i>emerges</i> from "
      "integral action."),
]
story += fig(HERE / "flowchart_guidance.png", 16.6 * cm)
story += [
    LI("<b>Measure what matters:</b> project GNSS position onto the current "
       "leg → cross-track error e. Pure software; the sensors already exist."),
    LI("<b>Command course:</b> course_cmd = leg azimuth + a bounded "
       "correction (L1-style). Boundedness is the honesty: in wind beyond "
       "the achievable crab the residual drift is bounded and known."),
    LI("<b>Let the integral find the crab:</b> yaw_sp = course_cmd + crab, "
       "crab = slow integral of the course error. Steady wind ⇒ steady "
       "crab, no wind sensor needed."),
    LI("<b>Fix the yaw pipeline while at it:</b> shortest-arc errors + an "
       "unwrapped setpoint remove the 22 wrap flips for free; switching by "
       "perpendicular plane (with turn anticipation R = V/r_max) removes "
       "the close-range bearing thrash."),
    LI("<b>Adapt speed:</b> when the needed crab saturates, slow the "
       "along-track setpoint instead of abandoning the line."),
    LI("<b>Log e, course and actuator commands</b> — to close the loop and "
       "to make the next flight debuggable."),

    P("2.3 · The proposal, measured", "h2"),
    P("Both guidance laws flew the same missions in the same wind, twice: a "
      "minimal planar model, and my own full airship simulator (twin "
      "vectoring pods fore/aft, hull weathervane aerodynamics, actuator "
      "lags, gusts + turbulence, noisy sensors; the Exercise 1 allocation "
      "with Fy = 0 respected by construction — yaw is a pure couple). "
      "Metrics are whole-leg, no transient exclusions:"),
    table([
        ["", "cross-track RMS", "worst", "mission time"],
        ["planar — baseline", "29.8 m", "57.4 m", "410 s"],
        ["planar — proposed", "7.5 m", "28.4 m", "343 s"],
        ["full sim — baseline (4 wind seeds)", "23.4 – 27.0 m",
         "51.6 – 64.3 m", "~360 s"],
        ["full sim — proposed (4 wind seeds)", "8.2 – 9.0 m",
         "20.8 – 30.0 m", "410 – 550 s"],
    ], [6.4 * cm, 3.6 * cm, 3.3 * cm, 3.3 * cm]),
    Spacer(1, 4),
    LI("<b>2.6–3.2× less RMS cross-track, 1.7–2.7× smaller worst case</b>, "
       "mission completed in every run."),
    LI("Honest costs: 14–52% more mission time (crabbing against wind at "
       "40–60% of airspeed spends airspeed — the baseline is «faster» "
       "because it surrenders to the drift), and a bounded residual: "
       "±10–17 m breathing on crosswind legs, 23–30 m at gust peaks on the "
       "slow upwind leg, where course observability drops."),
    PageBreak(),
]

story += fig(AN / "guidance_demo.png", 17.6 * cm,
             "Planar model: same vehicle, same wind. The baseline (red) bows "
             "downwind on every leg; the proposed guidance (blue) holds the "
             "line and hands the yaw PID a continuous setpoint (right panel).")
story += fig(AN / "sim_validation.png", 15.6 * cm,
             "Full simulator, seed 42. The proposed residual is gust "
             "breathing around the line, not a systematic bow.")
story += [PageBreak()]

story += [
    P("2.4 · Using the code (run · read · modify)", "h2"),
    P("Exercise 2 ships as two runnable comparisons plus the log-analysis "
      "script. Python pieces need numpy + matplotlib; the simulator piece "
      "needs Node ≥ 22 inside the simulator repo (branch ex2-guidance)."),
    CODE("# 1) planar demo — self-contained, ~5 s\n"
         "cd analysis && python guidance_demo.py        "
         "# prints metrics, writes guidance_demo.png\n\n"
         "# 2) full-simulator comparison — inside the simulator repo\n"
         "node tests/ex2-guidance.mjs                   # default wind seed 42\n"
         "EX2_SEED=7 node tests/ex2-guidance.mjs        # any other wind realization\n"
         "python analysis/plot_sim_validation.py analysis/ex2-results.json\n\n"
         "# 3) every Annex B number quoted in section 2.1\n"
         "python analysis/extract_log.py mission_hylight.html"),
    P("What to change, and where:", spaceBefore=4),
    table([
        ["knob", "planar (guidance_demo.py)", "simulator (ex2-guidance.mjs)"],
        ["mission legs", "WAYPOINTS, ACCEPT_R", "WPTS, ACCEPT_R, Z_HOLD"],
        ["wind & gusts", "WIND_BASE, GUST_AMP, GUST_T",
         "p.windMean, p.windDir, p.gustAmp, p.turb"],
        ["vehicle agility", "R_MAX, R_TAU, KP_YAW, V_CRUISE",
         "R_DES_MAX, K_RATE, V_CRUISE"],
        ["guidance tuning", "L1 = 25 m, crab gain 0.35, clamp ±35°",
         "L1_DIST, K_CRAB (0.12 — see note), CORR_MAX, CRAB_MAX"],
        ["which law flies", "run('baseline' | 'proposed')",
         "same, in the modes loop"],
        ["wind realization", "Plant(seed=…)", "EX2_SEED=… env var"],
    ], [3.1 * cm, 6.6 * cm, 6.9 * cm], fontsize=8.2, align="LEFT"),
    Spacer(1, 6),
    callout(
        "<b>Two transferable tuning notes.</b> (1) The crab integrator must "
        "be slow relative to the heading loop: in the full simulator "
        "K_CRAB = 0.35 limit-cycled against the heavy, integral-carried yaw "
        "loop with a ~100 s period; 0.12 is stable (the planar model's "
        "faster heading loop tolerates 0.35). (2) Metrics are whole-leg "
        "with no «settling» exclusions — an earlier filtered metric turned "
        "out to trim transients only from the proposed run, and was removed. "
        "The comparison pays for its own corner transients.",
        bg=colors.HexColor("#fff7ed"), border=ORANGE),
    Spacer(1, 10),
    P("<i>Everything in this document regenerates from the attached sources: "
      "src/ (Exercise 1), analysis/ (Exercise 2 + log), deliverable/ (this "
      "PDF). Property tests: ./tests → ALL TESTS PASSED.</i>", fontSize=8.8),
]

doc = SimpleDocTemplate(str(HERE / "GNC-Technical-Test-Answers.pdf"),
                        pagesize=A4,
                        leftMargin=2.0 * cm, rightMargin=2.0 * cm,
                        topMargin=1.7 * cm, bottomMargin=1.6 * cm,
                        title="GNC Technical Test — Answers",
                        author="David Steagall")
doc.build(story)
print("built GNC-Technical-Test-Answers.pdf")
