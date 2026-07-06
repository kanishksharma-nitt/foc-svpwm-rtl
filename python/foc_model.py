#!/usr/bin/env python3
"""Bit-exact golden model for the FOC (field-oriented control) pipeline.

Fixed-point contract (identical to the RTL):
  currents / voltages  : Q1.15 signed 16-bit
  rotor angle          : 16-bit unsigned, 0..65535 -> 0..2*pi
  PI accumulators      : Q8.24 signed 32-bit, clamped (anti-windup)
  sin/cos              : 257-entry quarter-wave LUT, Q1.15, 10-bit phase

The script:
  1. writes rtl/sin_lut.mem (the LUT the RTL loads with $readmemh),
  2. runs a closed loop: bit-exact integer controller + float first-order
     PMSM dq plant, for a speed-reference step,
  3. asserts analytic sanity (speed settles at the reference),
  4. emits test/foc_vectors.mem: per control step, 5 stimulus words
     (ia, ib, theta, spd_fb, spd_ref) and 5 expected words
     (id_meas, iq_meas, duty_a, duty_b, duty_c).

Stdlib only, deterministic.
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- constants
K_1_SQRT3 = 18919     # round(32768/sqrt(3))   Q1.15
K_SQRT3   = 56756     # round(32768*sqrt(3))   Q1.15 (needs 17 bits)
DMIN, DMAX = 10, 1013  # duty clamp (10-bit PWM, dead-band at the rails)

# current-loop PI (Q4.12 gains), accumulator clamp +/-1.0 in Q8.24
KP_C, KI_C = 4096, 205          # Kp=1.0, Ki=0.05
# speed-loop PI, runs every SPEED_DIV control strobes
KP_S, KI_S = 8192, 205          # Kp=2.0, Ki=0.05
ACC_LIM = 1 << 24               # +/-1.0 in Q8.24
SPEED_DIV = 20

NSTEPS = 200
STEP_AT = 20                    # speed reference step applied here
SPD_REF = 16384                 # 0.5 in Q1.15


def sat16(x):
    return max(-32768, min(32767, x))


# ------------------------------------------------------------------ sin/cos
def build_lut():
    return [min(32767, int(math.sin(math.pi / 2 * i / 256) * 32768 + 0.5))
            for i in range(257)]


LUT = build_lut()


def sin_q15(theta):
    ph = (theta >> 6) & 0x3FF          # 10-bit phase
    quad, idx = ph >> 8, ph & 0xFF
    if quad == 0:
        return LUT[idx]
    if quad == 1:
        return LUT[256 - idx]
    if quad == 2:
        return -LUT[idx]
    return -LUT[256 - idx]


def cos_q15(theta):
    return sin_q15((theta + 16384) & 0xFFFF)


# ------------------------------------------------------------- transforms
def clarke(ia, ib):
    alpha = ia
    beta = sat16(((ia + 2 * ib) * K_1_SQRT3) >> 15)
    return alpha, beta


def park(alpha, beta, s, c):
    d = sat16((alpha * c + beta * s) >> 15)
    q = sat16((-alpha * s + beta * c) >> 15)
    return d, q


def inv_park(vd, vq, s, c):
    alpha = sat16((vd * c - vq * s) >> 15)
    beta = sat16((vd * s + vq * c) >> 15)
    return alpha, beta


def svpwm(valpha, vbeta):
    """Min/max common-mode injection SVPWM; returns duties + sector."""
    va = valpha
    t = (vbeta * K_SQRT3) >> 15
    vb = (-valpha + t) >> 1
    vc = (-valpha - t) >> 1
    vmax, vmin = max(va, vb, vc), min(va, vb, vc)
    vcm = (vmax + vmin) >> 1
    duties = []
    for v in (va, vb, vc):
        d = (v - vcm + 32768) >> 6
        duties.append(max(DMIN, min(DMAX, d)))
    # sector from the phase ordering (1..6)
    a_b, b_c, a_c = va >= vb, vb >= vc, va >= vc
    if a_b and b_c:
        sector = 1
    elif (not a_b) and a_c:
        sector = 2
    elif b_c and not a_c:
        sector = 3
    elif (not b_c) and not a_b:
        sector = 4
    elif a_b and not a_c:
        sector = 5
    else:
        sector = 6
    return duties, sector


class PI:
    """Q8.24 accumulator, clamped; output Q1.15, saturated."""

    def __init__(self, kp, ki):
        self.kp, self.ki, self.acc = kp, ki, 0

    def step(self, ref, fb):
        e = sat16(ref - fb)
        i_next = self.acc + ((self.ki * e) >> 3)       # Q8.24
        i_next = max(-ACC_LIM, min(ACC_LIM, i_next))   # anti-windup clamp
        u = (((self.kp * e) >> 3) + i_next) >> 9       # Q8.24 -> Q1.15
        self.acc = i_next
        return sat16(u)


def q15f(x):
    return sat16(int(x * 32768))


def main():
    # ---- write the sine LUT the RTL loads --------------------------------
    lut_path = os.path.join(HERE, "..", "rtl", "sin_lut.mem")
    os.makedirs(os.path.dirname(lut_path), exist_ok=True)
    with open(lut_path, "w") as f:
        for v in LUT:
            f.write(f"{v & 0xFFFF:04x}\n")

    # ---- closed loop: integer controller + float plant -------------------
    pi_d = PI(KP_C, KI_C)
    pi_q = PI(KP_C, KI_C)
    pi_w = PI(KP_S, KI_S)
    iq_ref = 0

    idf = iqf = wf = 0.0          # plant state (floats, normalized)
    theta = 0
    rows = []
    for k in range(NSTEPS):
        spd_ref = SPD_REF if k >= STEP_AT else 0

        # --- stimulus: what the "ADC" and encoder present this step -------
        thf = theta * 2 * math.pi / 65536
        ialpha_f = idf * math.cos(thf) - iqf * math.sin(thf)
        ibeta_f = idf * math.sin(thf) + iqf * math.cos(thf)
        ia = q15f(ialpha_f)
        ib = q15f((-ialpha_f + math.sqrt(3) * ibeta_f) / 2)
        spd_fb = q15f(wf)

        # --- bit-exact controller (mirrors foc_top exactly) ---------------
        s, c = sin_q15(theta), cos_q15(theta)
        al, be = clarke(ia, ib)
        id_meas, iq_meas = park(al, be, s, c)
        vd = pi_d.step(0, id_meas)
        vq = pi_q.step(iq_ref, iq_meas)
        va, vb = inv_park(vd, vq, s, c)
        (da, db, dc), _sector = svpwm(va, vb)
        # speed PI updates AFTER the current loop, every SPEED_DIV strobes
        if (k % SPEED_DIV) == SPEED_DIV - 1:
            iq_ref = pi_w.step(spd_ref, spd_fb)

        rows.append((ia, ib, theta, spd_fb, spd_ref,
                     id_meas, iq_meas, da, db, dc))

        # --- float plant update -------------------------------------------
        vdf, vqf = vd / 32768.0, vq / 32768.0
        idf += 0.25 * (vdf - idf)
        iqf += 0.25 * (vqf - iqf - wf)      # back-emf coupling
        wf += 0.04 * (iqf - 0.1 * wf)
        theta = (theta + int(wf * 600)) & 0xFFFF

    # ---- analytic sanity checks ------------------------------------------
    assert abs(wf - 0.5) < 0.05, f"speed loop did not settle: w={wf:.3f}"
    assert abs(idf) < 0.05, f"d-axis current not regulated: id={idf:.3f}"
    print(f"closed-loop check: final speed {wf:.3f} (ref 0.500), "
          f"id {idf:.4f} (ref 0)")

    # ---- emit vectors ------------------------------------------------------
    out = os.path.join(HERE, "..", "test", "foc_vectors.mem")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(f"{NSTEPS:08x}\n")
        for r in rows:
            for v in r:
                f.write(f"{v & 0xFFFFFFFF:08x}\n")
    print(f"wrote {NSTEPS} control steps -> test/foc_vectors.mem")


if __name__ == "__main__":
    main()
