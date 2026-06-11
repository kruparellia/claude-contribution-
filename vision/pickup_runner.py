#!/usr/bin/env python3
"""
pickup_runner.py
----------------
End-to-end auto-mode runner: detect the rock on the real pickup spot (a
black-tape X on the table), wait for the operator to authorise, lift the arm
to a hover pose, then visually servo the base until the claw is swung over
the spot.

Flow (matches the firmware's auto-mode command surface)
-------------------------------------------------------
    1. enter auto mode (startup: set_auto) — firmware homes on auto-ON
    2. lock the pickup spot (the black-tape X) from the clean scene
    3. wait for the rock to land on the spot          IDLE -> LOCKING -> READY
    4. wait for the operator's go-ahead               SPACE in READY
    5. lift to the hover pose                         'v'  -> HOVERING
    6. nudge the base until the claw is over the spot 'b<deg>' loop -> SERVOING
    7. hold aligned over the spot                     ALIGNED
    (8. grasp + drop — TODO, see note below)

Spot vs rock
------------
The X is a FIXED mark — camera and tape don't move during a run — so we lock
its pixel position once, from the clean scene (the rock occludes the X once
placed). After that:
  - the ROCK detection is only the TRIGGER: "is something sitting on the
    spot?" (IDLE -> READY). It is debounced against shape-gate flicker.
  - the LOCKED SPOT is the AIM TARGET for the base servo. A fixed target
    bearing converges cleanly and is immune to the arm occluding the rock
    once it swings over.
The base rotation centre is likewise frozen at authorise-time (the arm
occludes the white disc during hover), so during servoing only the claw
marker moves and the geometry stays stable:

    Δbase = atan2(spot − centre) − atan2(claw − centre)

NOTE on step 8: firing 'p' from the hover pose is not wired yet — the current
PICKUP_SEQUENCE starts at KF_HOME (would undo the alignment) and seqStart()
requires being at home. A grasp-from-aligned-hover keyframe set (recorded at
the final rig, preserving the servoed base) is the next step. For now the
runner stops at ALIGNED — the "claw aligned over the pickup spot" deliverable.

State machine
-------------
    IDLE        spot lockable; no rock on the spot yet
    LOCKING     rock on the spot — counting consecutive frames
    READY       rock locked in — waiting for SPACE to authorise
    HOVERING    'v' sent — waiting HOVER_SETTLE_S for the arm to lift
    SERVOING    nudging the base toward the spot's bearing
    ALIGNED     claw is over the spot (|Δbase| < ALIGN_DEG) — holding
    COOLDOWN    short pause before the next cycle

Controls
--------
    SPACE   authorise (READY -> HOVERING); in ALIGNED, re-run the servo
    K       lock / unlock the pickup spot (auto-locks after SPOT_LOCK_FRAMES)
    F       flip the base-servo sign (polarity unknown until bring-up)
    [ / ]   trim the claw-bearing offset phi -/+ 5 deg (the elbow-spine
            marker is offset from the claw tip — dial this until the TIP,
            not the marker, lands on the spot; then bake it into the constant)
    H       send 'h' (home) to the arm + reset to IDLE
    A       toggle arm auto-mode
    X       send 'x' (abort) to the arm + reset to IDLE
    L       log the current diagnostic state to the terminal
    Q       quit

Sliders: V_min/S_max tune the white-disc mask (base centre); V_max/MinArea/
MaxArea tune the dark-tape mask (pickup spot) — same as pickup_spot.py.

Run
---
    python3 vision/pickup_runner.py
    python3 vision/pickup_runner.py --camera 1 --dry-run    # no serial open
"""
import argparse
import json
import os
import re
import time
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

from arm_link import ArmLink
from base_outline import largest_disc
from colour_detect import (
    COLOURS, MIN_BLOB_AREA, TARGET_COLOUR, TARGET_REFS,
    MATCH_Z_TOL, MATCH_MIN_FEATS, MATCH_MAX_TOTAL,
    _make_mask, _clean_mask,
)
from pickup_spot import find_dark_x, dist, SPOT_STABLE_PX
from shape_match import ShapeMatcher

# ---------------------------------------------------------------------------
# Rig-specific constants — RE-TUNE FOR THE FINAL SETUP
# ---------------------------------------------------------------------------
CAM_W, CAM_H = 640, 480

# Frames the rock must stay on the spot before we promote IDLE -> READY.
LOCK_FRAMES = 8

# Frames the dark-X detection must be stable before the spot auto-locks.
SPOT_LOCK_FRAMES = 12

# Rock-detection debounce. The shape gate can flicker frame-to-frame, so we
# treat the rock as "present" if it was detected in at least ROCK_DEBOUNCE_M
# of the last ROCK_DEBOUNCE_N frames, and reuse the last known centroid on the
# dropped frames (the rock isn't moving). Smooths the trigger gate.
ROCK_DEBOUNCE_N = 5
ROCK_DEBOUNCE_M = 2

# Pause after a cycle before we'll accept the next trigger.
COOLDOWN_S = 3.0

# ---- Visual-servo loop -----------------------------------------------------
# ArUco marker roles: 0-3 are stuck on the arm (elbow spine) and estimate the
# claw bearing; 10 is the base-heading marker on the BASE and must NOT be
# averaged into the claw estimate (it sits across the axis and is almost
# always visible, so it wrecks the claw position if included).
CLAW_IDS = {0, 1, 2, 3}
# All claw markers are rigidly fixed to the same elbow spine, so we reconstruct
# ONE consistent reference point (where marker CLAW_REF_ID sits) from whichever
# markers are visible — instead of averaging a random visible subset, which made
# the claw point (and thus the aligned pose) jump between runs.
CLAW_REF_ID = 0

# Markers stuck directly on the CLAW TIP (20 mm, IDs 5-8). With a tip marker we
# servo the tip pixel straight onto the rock pixel — no elbow proxy, no phi, no
# clicked centre, no bearing math. Reconstructed from whichever subset is seen.
TIP_IDS = {5, 6, 7, 8}
TIP_REF_ID = 5
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# Tip within this many pixels of the rock counts as aligned (direct pixel servo).
# Kept a little loose because the heavy base can't make sub-MIN_SERVO_STEP moves,
# so demanding < ~15 px just causes a limit cycle (overshoot / re-overshoot).
ALIGN_PX = 20.0
# Once ALIGNED, only re-servo if the error grows this much past where it settled
# — big enough that detection noise / the unfixable perpendicular residual can't
# bounce it back into SERVOING (that was the ALIGNED<->SERVOING flicker).
RESERVO_MARGIN_PX = 18.0

# |Δbase| below this (degrees) counts as aligned — stop nudging. Kept a touch
# wide because the heavy base can't reliably make sub-5-deg moves (stiction).
ALIGN_DEG = 4.0

# Damped proportional gain: apply only this fraction of the model-predicted
# correction (Δbase / k) each step, so k error, servo lag, and stiction lurch
# settle out over a few iterations instead of overshooting and ringing.
SERVO_GAIN = 0.5

# The base servo (MG996R) carries the whole arm and has real stiction — small
# commands don't break it loose, so the damped step is floored to MIN_SERVO_STEP
# whenever a correction is still needed, and capped at MAX_SERVO_STEP so a large
# error / bad k doesn't swing it wildly. PROBE_STEP is the first move on each
# servo entry: deliberately large so it reliably breaks stiction and yields a
# clean k in one shot (a 5-deg probe was too small and got lost in stiction).
MIN_SERVO_STEP = 6.0
MAX_SERVO_STEP = 8.0     # small: the heavy base lags, so never command far ahead
PROBE_STEP     = 10.0

# Online gain estimation. The servo learns k = Δ(image bearing) / Δ(commanded
# base) from each move, so it discovers BOTH the base direction (sign of k) and
# the local scale (an oblique camera makes deg-bearing-per-deg-base vary around
# the circle) without any hand-tuned SERVO_SIGN/SERVO_GAIN. A move only updates
# k if it was big enough to trust (not lost in stiction or marker noise), and
# wild estimates are rejected.
K_MIN_MOVE = 3.0     # deg of commanded base change before we trust a J/k sample
K_MIN_PX   = 3.0     # px of measured tip motion before we trust a J sample
J_ABS_LO   = 0.2     # reject |J| (px/deg) outside this plausible band
J_ABS_HI   = 60.0

# Base-rotation polarity: +1 or -1. Applied to BOTH the bootstrap probe and the
# adaptive step, so 'f' reverses the whole servo. Set to -1 after bring-up: the
# claw was converging to the mirror of the target, i.e. the base needs to turn
# the opposite way.
SERVO_SIGN = -1

# The claw markers sit on the elbow spine, across the base axis from the claw
# tip, so the claw's true bearing = marker bearing + φ. Calibrate φ in one shot:
# park the claw tip exactly over the rock/spot and press 'g' — the app computes
# φ so Δbase reads 0 there, and persists it (loaded on next launch).
CLAW_BEARING_OFFSET_DEG = 0.0

# Seconds to wait after 'v' for the arm to slew into the hover pose before we
# start servoing.
HOVER_SETTLE_S = 2.0

# Pacing the servo to the actual (slow, lagging) base: after a command we wait
# at least SERVO_MIN_GAP, then hold until the claw marker has actually STOPPED
# moving (settled within CLAW_STILL_PX between frames) before measuring/stepping
# again — so we never read the claw mid-motion and never command ahead of the
# real base. SERVO_SETTLE_MAX is a fallback in case the marker never fully
# stills (noise) so the loop can't deadlock.
SERVO_MIN_GAP    = 0.5
SERVO_SETTLE_MAX = 3.0
CLAW_STILL_PX    = 5.0

# Absolute servo cap, and a separate "blind" cap: the claw marker drops out for
# seconds at a time, and those blind stretches must NOT burn the budget — we
# only give up if we've gone SERVO_BLIND_S with no usable measurement, or hit
# the absolute SERVO_TIMEOUT_S overall.
SERVO_TIMEOUT_S = 45.0
SERVO_BLIND_S   = 6.0

# Base angle the arm homes to — starting estimate of the base position until
# the arm's [pos] telemetry corrects it (and for dry runs). Match KF_HOME.base.
HOME_BASE = 90

# Parses the 1 Hz "[pos] base=NN shoulder=.." telemetry for the live base angle.
_POS_RE = re.compile(r"\[pos\]\s+base=(-?\d+)")
# Parses the "[seq] auto=ON ... atHome=yes" status line (from '?' or transitions).
_SEQ_RE = re.compile(r"\[seq\]\s+auto=(ON|OFF).*atHome=(yes|no)")

# Startup home-first: wait this long for the auto-on home to finish (arm out of
# frame) before starting anyway with a warning. Poll arm status this often.
HOME_TIMEOUT_S = 12.0
STATUS_POLL_S  = 1.0

WIN = "Pickup runner (q quit)"

# Once authorised, the spot + base centre are frozen (the arm occludes both
# the rock and the disc during hover), so these states aim from cached values.
COMMITTED_STATES = ("HOVERING", "SERVOING", "ALIGNED")

# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------
@dataclass
class RunnerState:
    name: str = "IDLE"
    lock_count: int = 0
    state_started: float = 0.0
    last_nudge: float = 0.0
    cur_base: int = HOME_BASE        # best estimate of the arm's base angle
    last_dbase: float | None = None  # most recent measured Δbase
    last_arm_line: str = ""
    # Rock debounce history + last known centroid (see ROCK_DEBOUNCE_*).
    rock_hist: deque = field(default_factory=lambda: deque(maxlen=ROCK_DEBOUNCE_N))
    last_rock: tuple[int, int] | None = None
    # Pickup-spot cache (the black-tape X).
    spot: tuple[int, int] | None = None
    spot_r: int = 0
    spot_stable: int = 0
    spot_locked: bool = False
    # Base rotation centre. The camera is fixed, so the base pivot never moves
    # in-frame: prefer a click-set, persisted centre (centre_locked) over the
    # flaky white-disc detector. Detection only fills in until the user clicks.
    centre: tuple[int, int] | None = None
    centre_locked: bool = False
    # Servo aim target — the rock centroid, frozen at commit (the arm occludes
    # the live rock once it hovers, so we steer toward where the rock was when
    # the operator gave the go-ahead). Falls back to the spot if never set.
    aim: tuple[int, int] | None = None
    # Online image Jacobian J = d(tip pixel)/d(base deg), learned each run so the
    # base direction + pixel scale are discovered automatically (reset on entry).
    servo_J: np.ndarray | None = None        # [jx, jy] px per deg base
    servo_J_seed: np.ndarray | None = None   # persisted J — seeds each run, no probe
    servo_last_base: float | None = None     # base angle before the last nudge
    servo_last_tip: tuple[int, int] | None = None  # tip pixel before the last nudge
    servo_progress_t: float = 0.0            # last time we had a usable measurement
    servo_steps: int = 0                     # nudges taken this SERVOING session
    aligned_err: float = 0.0                 # px error captured when we aligned
    last_tip_px: tuple[int, int] | None = None     # prev-frame tip, for settle test
    # Recent tip detections, median-filtered to kill single-frame marker jitter.
    tip_hist: deque = field(default_factory=lambda: deque(maxlen=5))
    # Learned rigid offsets of each tip marker to the reference, so the tip point
    # is consistent regardless of which markers are seen.
    tip_offsets: dict = field(default_factory=dict)
    # Live arm status (polled via '?') — drives the home-first gate + HUD.
    auto_on: bool = False
    at_home: bool = False
    last_status_poll: float = 0.0

# The base pivot is fixed in the camera frame, so we calibrate it once (click)
# and persist it next to the refs. Far more robust than per-frame disc detection.
CENTRE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "refs", "base_centre.json")

def load_centre() -> tuple[int, int] | None:
    try:
        with open(CENTRE_FILE) as f:
            d = json.load(f)
        return (int(d["x"]), int(d["y"]))
    except (OSError, ValueError, KeyError):
        return None

def save_centre(centre: tuple[int, int]) -> None:
    try:
        os.makedirs(os.path.dirname(CENTRE_FILE), exist_ok=True)
        with open(CENTRE_FILE, "w") as f:
            json.dump({"x": int(centre[0]), "y": int(centre[1])}, f)
    except OSError as e:
        print(f"[pickup_runner] could not save centre: {e}")

PHI_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "refs", "claw_phi.json")

# Per-nudge servo trace (truncated each launch) for diagnosing convergence.
SERVO_LOG = "/tmp/servo_log.csv"

def load_phi() -> float | None:
    try:
        with open(PHI_FILE) as f:
            return float(json.load(f)["phi"])
    except (OSError, ValueError, KeyError):
        return None

def save_phi(phi: float) -> None:
    try:
        os.makedirs(os.path.dirname(PHI_FILE), exist_ok=True)
        with open(PHI_FILE, "w") as f:
            json.dump({"phi": float(phi)}, f)
    except OSError as e:
        print(f"[pickup_runner] could not save phi: {e}")

J_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "refs", "servo_jacobian.json")

def load_jacobian() -> np.ndarray | None:
    try:
        with open(J_FILE) as f:
            d = json.load(f)
        return np.array([float(d["jx"]), float(d["jy"])], dtype=float)
    except (OSError, ValueError, KeyError):
        return None

def save_jacobian(j: np.ndarray) -> None:
    try:
        os.makedirs(os.path.dirname(J_FILE), exist_ok=True)
        with open(J_FILE, "w") as f:
            json.dump({"jx": float(j[0]), "jy": float(j[1])}, f)
    except OSError:
        pass

def on_mouse(event, x, y, _flags, state) -> None:
    """Left-click sets and locks the base rotation centre."""
    if event == cv2.EVENT_LBUTTONDOWN:
        state.centre = (int(x), int(y))
        state.centre_locked = True
        save_centre(state.centre)
        print(f"[pickup_runner] CENTRE set + locked at {state.centre} (saved)")

def norm180(deg: float) -> float:
    """Wrap an angle to (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0

def bearing_deg(centre: tuple[int, int], pt: tuple[int, int]) -> float:
    """Image-frame bearing of pt as seen from centre, in degrees."""
    return float(np.degrees(np.arctan2(pt[1] - centre[1], pt[0] - centre[0])))

def find_rock(hsv: np.ndarray, matcher: ShapeMatcher) -> tuple[int, int] | None:
    """Return rock centroid in pixels, or None if not detected this frame."""
    red = next(c for c in COLOURS if c["name"] == TARGET_COLOUR)
    clean = _clean_mask(_make_mask(hsv, red))
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_BLOB_AREA:
            continue
        is_target, _n, total_z, _ = matcher.score(cnt, hsv)
        if not is_target:
            continue
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            continue
        cx = int(M["m10"] / M["m00"]); cy = int(M["m01"] / M["m00"])
        if best is None or total_z < best[0]:
            best = (total_z, (cx, cy))
    return best[1] if best else None

def reconstruct_point(corners, ids, id_set: set, ref_id: int,
                      offsets: dict) -> tuple[int, int] | None:
    """Consistent reference point from a rigid cluster of markers.

    Every marker in `id_set` is rigidly fixed to the same body, so marker `mid`
    always sits at a fixed offset from the reference marker. We learn those
    offsets online (whenever the reference is co-visible) and then estimate the
    reference point from EVERY visible marker via its offset, averaging them.
    This yields the same physical point regardless of which subset is detected,
    where a plain mean-of-visible-subset jumps around. `offsets` is mutated in
    place to persist the learned geometry across frames.
    """
    if ids is None:
        return None
    pos = {}
    for marker_corners, mid in zip(corners, ids.flatten()):
        mid = int(mid)
        if mid in id_set:
            c = marker_corners.reshape(-1, 2).mean(axis=0)
            pos[mid] = np.array([c[0], c[1]], dtype=float)
    if not pos:
        return None

    # Learn offsets to the reference marker while it's visible (EMA-smoothed).
    if ref_id in pos:
        ref = pos[ref_id]
        for mid, p in pos.items():
            off = ref - p
            prev = offsets.get(mid)
            offsets[mid] = off if prev is None else 0.85 * prev + 0.15 * off

    # Estimate the reference point from every marker with a known offset.
    ests = [p + offsets[mid] for mid, p in pos.items() if mid in offsets]
    if not ests:                       # offsets not learned yet — best effort
        ests = list(pos.values())
    m = np.mean(ests, axis=0)
    return (int(m[0]), int(m[1]))

def find_centre(hsv: np.ndarray, v_min: int, s_max: int) -> tuple[int, int] | None:
    """Base rotation centre from the white-disc ellipse, or None."""
    disc_mask = cv2.inRange(hsv, (0, 0, v_min), (180, s_max, 255))
    ellipse, _cnt, _clean = largest_disc(disc_mask)
    if ellipse is None:
        return None
    return tuple(map(int, ellipse[0]))

def compute_dbase(centre, target, claw) -> float | None:
    """Δbase (deg) to swing the claw onto the target's bearing, or None."""
    if centre is None or target is None or claw is None:
        return None
    claw_bearing = bearing_deg(centre, claw) + CLAW_BEARING_OFFSET_DEG
    return norm180(bearing_deg(centre, target) - claw_bearing)

def drain_serial(arm: ArmLink | None) -> list[str]:
    """Read whatever the arm has emitted since the last call, non-blocking."""
    if arm is None or arm._ser is None or arm._ser.in_waiting == 0:
        return []
    data = arm._ser.read(arm._ser.in_waiting)
    return [
        line.rstrip("\r")
        for line in data.decode(errors="replace").split("\n")
        if line.strip()
    ]

def transition(state: RunnerState, new_name: str) -> None:
    state.name = new_name
    state.state_started = time.monotonic()
    state.lock_count = 0

def start_servoing(state: RunnerState, now: float) -> None:
    """Enter SERVOING with a fresh Jacobian estimate and progress/blind timers."""
    transition(state, "SERVOING")
    state.last_nudge = 0.0                # nudge immediately
    state.servo_J = (None if state.servo_J_seed is None
                     else state.servo_J_seed.copy())  # seed -> skip blind probe
    state.servo_last_base = None
    state.servo_last_tip = None
    state.servo_steps = 0
    state.servo_progress_t = now          # start the blind-timeout clock now

# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------
STATE_COLOURS = {
    "HOMING":   (255, 200,   0),
    "IDLE":     (160, 160, 160),
    "LOCKING":  (  0, 200, 255),
    "READY":    (  0, 255,   0),
    "HOVERING": (255, 200,   0),
    "SERVOING": (255, 140,   0),
    "ALIGNED":  (  0, 255,   0),
    "CALIB":    (255,   0, 255),
    "COOLDOWN": (130, 130, 200),
}

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--dry-run", action="store_true",
                   help="Skip the arm serial connection — vision-only test.")
    args = p.parse_args()

    global SERVO_SIGN, CLAW_BEARING_OFFSET_DEG   # 'F' / '[' ']' adjust live

    matcher = ShapeMatcher(TARGET_REFS, z_tol=MATCH_Z_TOL,
                           min_features=MATCH_MIN_FEATS,
                           max_total_z=MATCH_MAX_TOTAL)

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    if not cap.isOpened():
        raise SystemExit(f"Could not open /dev/video{args.camera}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    # Looser, more robust detection — the elbow markers are viewed at an oblique
    # angle and partly motion-blurred, which the stock params miss. Wider
    # adaptive-threshold window range, sub-pixel corner refine, and a smaller
    # min perimeter so partly-foreshortened markers still register.
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.adaptiveThreshWinSizeMin = 3
    aruco_params.adaptiveThreshWinSizeMax = 35
    aruco_params.adaptiveThreshWinSizeStep = 4
    aruco_params.minMarkerPerimeterRate = 0.02
    aruco_params.polygonalApproxAccuracyRate = 0.06
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector   = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    cv2.namedWindow(WIN)
    cv2.createTrackbar("V_min",   WIN, 150,  255,   lambda v: None)   # disc bright
    cv2.createTrackbar("S_max",   WIN, 60,   255,   lambda v: None)   # disc sat
    cv2.createTrackbar("V_max",   WIN, 60,   255,   lambda v: None)   # tape dark
    cv2.createTrackbar("MinArea", WIN, 300,  5000,  lambda v: None)
    cv2.createTrackbar("MaxArea", WIN, 8000, 40000, lambda v: None)

    arm: ArmLink | None = None
    if not args.dry_run:
        arm = ArmLink().__enter__()
        print(f"[pickup_runner] connected to arm on {arm.port}")
        try:
            arm.set_auto(True)          # firmware homes the arm on auto-ON
            arm.home()                  # belt-and-suspenders home request
            print("[pickup_runner] auto mode ON — homing the arm first")
        except RuntimeError as e:
            print(f"[pickup_runner] WARN: {e} — arm may not respond")
    else:
        print("[pickup_runner] DRY RUN — no serial connection")

    state = RunnerState(state_started=time.monotonic())
    with open(SERVO_LOG, "w") as lg:
        lg.write("t,base_before,err_px,step,base_after,tip,target,-,J\n")
    state.servo_J_seed = load_jacobian()
    if state.servo_J_seed is not None:
        print(f"[pickup_runner] loaded servo J={state.servo_J_seed} "
              "(seeds servo, skips probe)")
    saved_centre = load_centre()
    if saved_centre is not None:
        state.centre, state.centre_locked = saved_centre, True
        print(f"[pickup_runner] loaded base centre {saved_centre} from {CENTRE_FILE}")
    else:
        print("[pickup_runner] no saved base centre — LEFT-CLICK the base pivot to set it")
    cv2.setMouseCallback(WIN, on_mouse, state)
    # Home FIRST so the arm is out of frame before we try to lock the spot
    # (a down-pointed arm reads as a dark blob and gets picked as the X).
    transition(state, "IDLE" if arm is None else "HOMING")
    print("[pickup_runner] running — homing, then clear the rock so the SPOT "
          "locks and drop it on the X.\n  SPACE=go  K=lock spot  F=flip sign  "
          "H=home  A=auto  X=abort  L=log  Q=quit")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05); continue

            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            v_min    = cv2.getTrackbarPos("V_min", WIN)
            s_max    = cv2.getTrackbarPos("S_max", WIN)
            v_max    = cv2.getTrackbarPos("V_max", WIN)
            min_area = cv2.getTrackbarPos("MinArea", WIN)
            max_area = max(cv2.getTrackbarPos("MaxArea", WIN), min_area + 1)

            now_t = time.monotonic()

            # ---- arm telemetry: poll '?' periodically, drain the log ----
            if arm is not None and now_t - state.last_status_poll > STATUS_POLL_S:
                arm.send_line("?")
                state.last_status_poll = now_t
            for line in drain_serial(arm):
                state.last_arm_line = line
                mp = _POS_RE.search(line)
                if mp:
                    state.cur_base = int(mp.group(1))
                ms = _SEQ_RE.search(line)
                if ms:
                    state.auto_on = ms.group(1) == "ON"
                    state.at_home = ms.group(2) == "yes"
                print(f"  [arm] {line}")

            # ---- home FIRST: wait for the auto-on home before touching vision ----
            if state.name == "HOMING":
                if state.at_home:
                    print("[pickup_runner] arm home — starting spot detection")
                    transition(state, "IDLE")
                elif now_t - state.state_started > HOME_TIMEOUT_S:
                    print(f"[pickup_runner] WARN: home not confirmed "
                          f"(auto={'ON' if state.auto_on else 'OFF'}) — starting anyway")
                    transition(state, "IDLE")
                else:
                    cv2.rectangle(frame, (0, 0), (CAM_W, 58), (35, 35, 35), -1)
                    cv2.putText(frame, "HOMING - clearing arm from frame...",
                                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 200, 0), 2, cv2.LINE_AA)
                    acol = (120, 255, 120) if state.auto_on else (120, 120, 255)
                    cv2.putText(frame, f"auto={'ON' if state.auto_on else 'OFF'}   "
                                f"atHome={'yes' if state.at_home else 'no'}   "
                                "(H rehome  A auto  Q quit)", (10, 48),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, acol, 1, cv2.LINE_AA)
                    cv2.imshow(WIN, frame)
                    k = cv2.waitKey(1) & 0xFF
                    if k == ord("q"):
                        break
                    if k == ord("h") and arm is not None:
                        arm.home(); print("[pickup_runner] -> home (retry)")
                    if k == ord("a") and arm is not None:
                        arm.toggle_auto(); print("[pickup_runner] -> toggle auto")
                    continue

            committed = state.name in COMMITTED_STATES

            # ---- pickup spot (black-tape X): lock from the clean scene ----
            det, det_r, dark_mask = find_dark_x(hsv, v_max, min_area, max_area)
            if not state.spot_locked and det is not None:
                if state.spot is not None and dist(det, state.spot) < SPOT_STABLE_PX:
                    state.spot_stable += 1
                else:
                    state.spot_stable = 0
                state.spot, state.spot_r = det, det_r
                if state.spot_stable >= SPOT_LOCK_FRAMES:
                    state.spot_locked = True
                    print(f"[pickup_runner] SPOT locked at {state.spot} r={state.spot_r}")

            # ---- base centre: prefer the locked/click-set centre; otherwise
            # fall back to disc detection (cached while not committed) ----
            if not state.centre_locked:
                det_centre = find_centre(hsv, v_min, s_max)
                if det_centre is not None and not committed:
                    state.centre = det_centre

            # ---- rock (debounced) — the trigger only ----
            raw_rock = find_rock(hsv, matcher)
            state.rock_hist.append(raw_rock is not None)
            if raw_rock is not None:
                state.last_rock = raw_rock
            rock_present = sum(state.rock_hist) >= ROCK_DEBOUNCE_M
            rock = state.last_rock if rock_present else None
            rock_on_spot = (state.spot_locked and rock is not None
                            and dist(rock, state.spot) <= state.spot_r)

            # ---- markers -> CLAW TIP estimate (IDs 5-8 on the gripper) ----
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            raw_tip = reconstruct_point(corners, ids, TIP_IDS, TIP_REF_ID,
                                        state.tip_offsets)
            # Median-filter the tip over the last few detections to suppress
            # single-frame marker jitter. None this frame -> tip is None (the
            # servo waits) rather than reusing a stale position.
            if raw_tip is not None:
                state.tip_hist.append(raw_tip)
                xs = sorted(p[0] for p in state.tip_hist)
                ys = sorted(p[1] for p in state.tip_hist)
                tip = (xs[len(xs) // 2], ys[len(ys) // 2])
            else:
                tip = None
            # How far the tip moved since last frame — used to tell when the
            # (slow, lagging) base has actually settled before we measure again.
            tip_moved_px = (dist(tip, state.last_tip_px)
                            if tip is not None and state.last_tip_px is not None
                            else None)
            if tip is not None:
                state.last_tip_px = tip

            # ---- pixel error: tip -> ROCK (frozen at commit), spot as fallback ----
            aim_target = state.aim if state.aim is not None else state.spot
            if tip is not None and aim_target is not None:
                err_vec = np.array([aim_target[0] - tip[0],
                                    aim_target[1] - tip[1]], dtype=float)
                err_px = float(np.hypot(err_vec[0], err_vec[1]))
            else:
                err_vec = None
                err_px = None
            state.last_dbase = err_px

            # ---- state machine ----
            if state.name == "IDLE":
                if rock_on_spot:
                    transition(state, "LOCKING"); state.lock_count = 1
            elif state.name == "LOCKING":
                if rock_on_spot:
                    state.lock_count += 1
                    if state.lock_count >= LOCK_FRAMES:
                        transition(state, "READY")
                else:
                    transition(state, "IDLE")
            elif state.name == "READY":
                if not rock_on_spot:
                    transition(state, "IDLE")
                # SPACE -> HOVERING handled in input section below.
            elif state.name == "HOVERING":
                if now_t - state.state_started > HOVER_SETTLE_S:
                    start_servoing(state, now_t)
            elif state.name == "SERVOING":
                if now_t - state.state_started > SERVO_TIMEOUT_S:
                    print("[pickup_runner] servo timeout (absolute) — backing off")
                    transition(state, "COOLDOWN")
                elif err_px is None:
                    # Tip marker not visible this frame — wait, don't burn the
                    # budget, but bail if we've been blind too long.
                    if now_t - state.servo_progress_t > SERVO_BLIND_S:
                        print("[pickup_runner] lost tip marker too long — backing off")
                        transition(state, "COOLDOWN")
                elif err_px < ALIGN_PX:
                    print(f"[pickup_runner] ALIGNED err={err_px:.0f}px base={state.cur_base} "
                          f"tip={tip} aim={aim_target} J={state.servo_J}")
                    # Remember the learned Jacobian so the next run skips the probe.
                    if state.servo_J is not None:
                        state.servo_J_seed = state.servo_J.copy()
                        save_jacobian(state.servo_J)
                    state.aligned_err = err_px
                    transition(state, "ALIGNED")
                elif (now_t - state.last_nudge > SERVO_MIN_GAP and
                      (tip_moved_px is not None and tip_moved_px <= CLAW_STILL_PX
                       or now_t - state.last_nudge > SERVO_SETTLE_MAX)):
                    # Base has settled (tip stopped moving) — safe to measure the
                    # true response and take the next step.
                    state.servo_progress_t = now_t    # we have a usable measurement
                    tip_v = np.array([tip[0], tip[1]], dtype=float)

                    # Learn the image Jacobian J = Δ(tip px) / Δ(base deg) from the
                    # previous move (carries both base direction and pixel scale).
                    if state.servo_last_base is not None and state.servo_last_tip is not None:
                        dcmd = state.cur_base - state.servo_last_base
                        dtip = tip_v - np.array(state.servo_last_tip, dtype=float)
                        if abs(dcmd) >= K_MIN_MOVE and np.hypot(*dtip) >= K_MIN_PX:
                            j_meas = dtip / dcmd
                            if J_ABS_LO <= np.hypot(*j_meas) <= J_ABS_HI:
                                state.servo_J = (j_meas if state.servo_J is None
                                                 else 0.5 * state.servo_J + 0.5 * j_meas)

                    step = None
                    if state.servo_J is None:
                        # Bootstrap: a fixed probe to measure J. Direction doesn't
                        # matter — J is learned either way (re-probe if stiction
                        # ate it).
                        step = SERVO_SIGN * PROBE_STEP
                    else:
                        # Ideal 1-DOF base change to null the error (least squares).
                        JdotJ = float(state.servo_J @ state.servo_J)
                        ideal = float(err_vec @ state.servo_J) / max(JdotJ, 1e-6)
                        if abs(ideal) < MIN_SERVO_STEP and state.servo_steps >= 1:
                            # A min step would overshoot — this is the closest the
                            # base can physically get. Accept it (stops the limit
                            # cycle) instead of bouncing past the rock forever.
                            # Require >=1 real move first so a stale seeded J can't
                            # declare "closest" before ever moving.
                            print(f"[pickup_runner] ALIGNED (closest) err={err_px:.0f}px "
                                  f"base={state.cur_base} tip={tip} aim={aim_target}")
                            state.servo_J_seed = state.servo_J.copy()
                            save_jacobian(state.servo_J)
                            state.aligned_err = err_px
                            transition(state, "ALIGNED")
                        else:
                            # Damped so J error / lag settle instead of ringing.
                            desired = SERVO_GAIN * ideal
                            mag = float(np.clip(abs(desired), MIN_SERVO_STEP, MAX_SERVO_STEP))
                            step = float(np.sign(desired)) * mag

                    if step is not None:
                        new_base = int(round(np.clip(state.cur_base + step, 0, 180)))
                        old_base = state.cur_base
                        # Remember this nudge's start for the next J sample.
                        state.servo_last_tip = tip
                        state.servo_last_base = old_base
                        state.cur_base = new_base
                        state.last_nudge = now_t
                        state.servo_steps += 1
                        jstr = ("probe" if state.servo_J is None
                                else f"[{state.servo_J[0]:+.2f},{state.servo_J[1]:+.2f}]")
                        with open(SERVO_LOG, "a") as lg:
                            lg.write(f"{now_t:.2f},{old_base},{err_px:.1f},{step:+.2f},"
                                     f"{new_base},{tip},{aim_target},-,{jstr}\n")
                        if arm is not None:
                            arm.base(new_base)
                            print(f"[pickup_runner] nudge err={err_px:.0f}px J={jstr} "
                                  f"base {old_base}->{new_base}")
                        else:
                            print(f"[pickup_runner] (dry) err={err_px:.0f}px J={jstr} -> b{new_base}")
            elif state.name == "ALIGNED":
                # Sticky: only re-servo if the error grows well past where we
                # settled (a real rock move), not on detection noise or the
                # unfixable residual — otherwise it flickers ALIGNED<->SERVOING.
                if err_px is not None and err_px > state.aligned_err + RESERVO_MARGIN_PX:
                    start_servoing(state, now_t)
            elif state.name == "CALIB":
                pass   # hold the hover pose; user jogs base (n/m) then presses g
            elif state.name == "COOLDOWN":
                if now_t - state.state_started > COOLDOWN_S:
                    transition(state, "IDLE")

            # ---- drawing ----
            # dark-mask preview (bottom-right) to help re-tune the spot in context
            thumb = cv2.resize(dark_mask, (160, 120))
            frame[CAM_H - 120:CAM_H, CAM_W - 160:CAM_W] = cv2.cvtColor(thumb, cv2.COLOR_GRAY2BGR)

            if det is not None and not state.spot_locked:
                cv2.circle(frame, det, det_r, (0, 180, 255), 1)
            if state.spot is not None:
                scol = (0, 255, 0) if state.spot_locked else (0, 180, 255)
                cv2.circle(frame, state.spot, state.spot_r, scol, 2)
                cv2.drawMarker(frame, state.spot, scol, cv2.MARKER_TILTED_CROSS, 18, 2)
                cv2.putText(frame, "SPOT" + (" (locked)" if state.spot_locked else ""),
                            (state.spot[0] + state.spot_r + 4, state.spot[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, scol, 1, cv2.LINE_AA)
            if state.centre is not None:
                cv2.drawMarker(frame, state.centre, (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
                cv2.putText(frame, "CENTRE" + (" (locked)" if state.centre_locked else ""),
                            (state.centre[0] + 10, state.centre[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)
            else:
                cv2.putText(frame, "LEFT-CLICK the base pivot to set CENTRE",
                            (10, CAM_H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 255), 2, cv2.LINE_AA)
            if rock is not None:
                rcol = (0, 255, 0) if rock_on_spot else (0, 200, 255)
                cv2.circle(frame, rock, 8, rcol, 2)
                cv2.circle(frame, rock, 2, rcol, -1)
                cv2.putText(frame, "ROCK held" if raw_rock is None else "ROCK",
                            (rock[0] + 12, rock[1] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, rcol, 1, cv2.LINE_AA)
            if tip is not None:
                cv2.drawMarker(frame, tip, (0, 255, 255),
                               cv2.MARKER_CROSS, markerSize=22, thickness=2)
                cv2.putText(frame, "TIP", (tip[0] + 12, tip[1] + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            # Direct error guide: the line the servo is shrinking, tip -> rock.
            if state.aim is not None:
                cv2.drawMarker(frame, state.aim, (0, 255, 0),
                               cv2.MARKER_DIAMOND, 16, 2)
            if tip is not None and aim_target is not None:
                cv2.line(frame, tip, aim_target, (0, 255, 255), 1, cv2.LINE_AA)

            scolour = STATE_COLOURS.get(state.name, (200, 200, 200))
            extra = ""
            if state.name == "IDLE" and not state.spot_locked:
                extra = f"  locking spot ({state.spot_stable}/{SPOT_LOCK_FRAMES})"
            elif state.name == "LOCKING":
                extra = f" ({state.lock_count}/{LOCK_FRAMES})"
            elif state.name == "READY":
                extra = "  press SPACE to hover"
            elif state.name == "HOVERING":
                extra = f"  ({now_t - state.state_started:3.1f}s)"
            elif state.name == "SERVOING":
                extra = (f"  err={err_px:.0f}px" if err_px is not None
                         else "  (tip marker not visible)")
            elif state.name == "ALIGNED":
                extra = (f"  err={err_px:.0f}px  on rock"
                         if err_px is not None else "  on rock")
            elif state.name == "CALIB":
                extra = "  hover hold (jog base n/m; SPACE=servo)"
            elif state.name == "COOLDOWN":
                left = COOLDOWN_S - (now_t - state.state_started)
                extra = f"  ({left:3.1f}s)"

            cv2.rectangle(frame, (0, 0), (CAM_W, 30), (35, 35, 35), -1)
            cv2.putText(frame, f"STATE: {state.name}{extra}",
                        (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                        scolour, 2, cv2.LINE_AA)
            cv2.putText(frame, f"base={state.cur_base} sign={SERVO_SIGN:+d}",
                        (CAM_W - 175, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (200, 200, 200), 1, cv2.LINE_AA)
            acol = (120, 255, 120) if state.auto_on else (120, 120, 255)
            cv2.putText(frame, f"auto={'ON' if state.auto_on else 'OFF'} "
                        f"home={'Y' if state.at_home else 'N'}",
                        (CAM_W - 175, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        acol, 1, cv2.LINE_AA)
            errtxt = f"{err_px:.0f}px" if err_px is not None else "--"
            cv2.putText(frame, f"err={errtxt}",
                        (CAM_W - 175, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (200, 200, 120), 1, cv2.LINE_AA)

            if state.last_arm_line:
                cv2.putText(frame, f"arm: {state.last_arm_line[:54]}",
                            (10, CAM_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (200, 200, 200), 1, cv2.LINE_AA)

            cv2.imshow(WIN, frame)

            # ---- input handling ----
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" ") and state.name == "READY":
                # Freeze the rock's current position as the servo aim — the arm
                # will occlude the live rock once it hovers, so we steer toward
                # where it is right now (fall back to the spot if not held).
                state.aim = rock if rock is not None else state.spot
                print(f"[pickup_runner] aim frozen at {state.aim}")
                if arm is not None:
                    arm.hover(); print("[pickup_runner] -> hover")
                else:
                    print("[pickup_runner] (dry run) would send hover")
                transition(state, "HOVERING")
            elif key == ord(" ") and state.name in ("ALIGNED", "CALIB"):
                # Re-run the servo from the hover pose (e.g. after trimming phi
                # or finishing a calibration).
                print("[pickup_runner] -> re-servo")
                start_servoing(state, now_t)
            elif key == ord("v") and arm is not None:
                # Enter phi-calibration: park at the HOVER pose and hold (no auto
                # servo) so phi is captured at the exact pose it will servo at.
                state.aim = rock if rock is not None else state.spot
                arm.hover()
                print("[pickup_runner] -> hover for CALIB (jog base n/m; SPACE=servo)")
                transition(state, "CALIB")
            elif key in (ord("n"), ord("m")) and arm is not None:
                # Jog ONLY the base (pose stays = hover) for manual checks.
                state.cur_base = int(np.clip(state.cur_base + (5 if key == ord("m") else -5), 0, 180))
                arm.base(state.cur_base)
                print(f"[pickup_runner] base jog -> {state.cur_base}")
            elif key == ord("g"):
                # Forget the learned image Jacobian (and its saved seed) so the
                # next servo re-probes from scratch — use if the camera/rig moved.
                state.servo_J = None
                state.servo_J_seed = None
                try:
                    os.remove(J_FILE)
                except OSError:
                    pass
                print("[pickup_runner] Jacobian reset — will re-probe next servo")
            elif key == ord("c"):
                state.centre_locked = False
                state.centre = None
                print("[pickup_runner] CENTRE unlocked — left-click to re-set")
            elif key == ord("k"):
                state.spot_locked = not state.spot_locked
                state.spot_stable = 0
                print(f"[pickup_runner] spot {'LOCKED' if state.spot_locked else 'UNLOCKED'} "
                      f"at {state.spot}")
            elif key == ord("f"):
                SERVO_SIGN = -SERVO_SIGN
                print(f"[pickup_runner] servo sign -> {SERVO_SIGN:+d}")
            elif key == ord("h") and arm is not None:
                arm.home(); print("[pickup_runner] -> home")
                transition(state, "IDLE")
            elif key == ord("a") and arm is not None:
                arm.toggle_auto(); print("[pickup_runner] -> toggle auto")
            elif key == ord("x") and arm is not None:
                arm.abort(); print("[pickup_runner] -> abort")
                transition(state, "IDLE")
            elif key == ord("l"):
                print(f"[log] state={state.name} spot={state.spot} "
                      f"locked={state.spot_locked} rock={rock} on_spot={rock_on_spot} "
                      f"tip={tip} aim={state.aim} base={state.cur_base} "
                      f"err_px={err_px} J={state.servo_J}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if arm is not None:
            arm.__exit__(None, None, None)

if __name__ == "__main__":
    main()
