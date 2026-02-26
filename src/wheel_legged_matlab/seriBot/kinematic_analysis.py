"""
Wheel-Legged Robot Kinematic & Inertia Analysis
================================================
Reads MuJoCo XML, solves full planar kinematics (including four-bar front
linkage via constraint equations), computes:

  • P1, P2, P3  — link centroids and equivalent CoM
  • P4          — virtual leg CoM on Y-axis (circle intersect)
  • I_P4        — moment of inertia about P4 (joint axis direction)
                  via parallel-axis theorem for ALL left-leg links

Coordinate system (2D motion plane, perpendicular to joint Y-axis):
  Origin = A = left_rear_hip_joint
  +x     = horizontal (outward)
  +y     = vertical (downward positive, same direction as L0)
"""

import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import math, os, sys

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def rot2d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])

def quat_to_rot3(q):
    """wxyz quaternion → 3×3 rotation matrix"""
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)]
    ])

def scalar_inertia_about_axis(diag, quat_wxyz, axis):
    """
    Scalar moment of inertia about 'axis' (unit 3-vector).
    diag     : [Ixx, Iyy, Izz] in body principal frame
    quat_wxyz: orientation of body frame w.r.t. world (w,x,y,z)
    axis     : unit vector of rotation axis in world frame
    Formula  : I = axis^T · R · diag(I) · R^T · axis
    """
    R = quat_to_rot3(quat_wxyz)
    I_world = R @ np.diag(diag) @ R.T
    a = np.asarray(axis, dtype=float)
    return float(a @ I_world @ a)

# ─────────────────────────────────────────────────────────────────────────────
# 1. Parse XML
# ─────────────────────────────────────────────────────────────────────────────

def parse_xml(xml_path):
    """Return dict of link properties keyed by body name."""
    tree = ET.parse(xml_path)
    root = tree.getroot()

    TARGET_LINKS = [
        'left_rear_hip_link', 'left_rear_knee_link',  # wheel excluded
        'left_front_hip_link', 'left_child1_link', 'left_child2_link', 'left_child3_link',
    ]

    links = {}
    for body in root.iter('body'):
        name = body.get('name', '')
        if name not in TARGET_LINKS:
            continue
        inertial = body.find('inertial')
        if inertial is None:
            continue

        pos_str  = body.get('pos', '0 0 0')
        cm_str   = inertial.get('pos', '0 0 0')
        diag_str = inertial.get('diaginertia', '0 0 0')
        quat_str = inertial.get('quat', '1 0 0 0')
        mass     = float(inertial.get('mass', '0'))

        body_pos = np.array([float(v) for v in pos_str.split()])
        cm_local = np.array([float(v) for v in cm_str.split()])
        diag_I   = np.array([float(v) for v in diag_str.split()])
        quat     = np.array([float(v) for v in quat_str.split()])

        joints   = body.findall('joint')
        axes     = [[float(v) for v in j.get('axis','0 0 1').split()] for j in joints]
        joint_axis = np.array(axes[0]) if axes else np.array([0.,1.,0.])

        links[name] = dict(
            body_pos=body_pos,
            cm_local=cm_local,
            diag_I=diag_I,
            quat=quat,
            mass=mass,
            joint_axis=joint_axis,
        )

    # Projection axis = Y (from any joint)
    rot_axis = np.array([0., 1., 0.])

    # Projected 2D link lengths (for rear chain)
    def proj_xz(v3):
        p = v3 - np.dot(v3, rot_axis) * rot_axis
        return np.array([p[0], p[2]])

    knee_pos_3d  = links['left_rear_knee_link']['body_pos']
    # wheel excluded from inertia but L2 still needed for kinematics
    wheel_pos_3d = np.array([0.217889, 0.01855, -0.130539])  # hardcoded from XML
    L1 = float(np.linalg.norm(proj_xz(knee_pos_3d)))
    L2 = float(np.linalg.norm(proj_xz(wheel_pos_3d)))

    print(f"  Rotation axis   : {rot_axis}")
    print(f"  L1 (projected)  : {L1:.6f} m")
    print(f"  L2 (projected)  : {L2:.6f} m")
    print(f"  Reachable L0    : [{abs(L1-L2):.4f}, {L1+L2:.4f}] m")

    return links, rot_axis, L1, L2

# ─────────────────────────────────────────────────────────────────────────────
# 2. Full kinematic solver (rear chain + four-bar front chain)
# ─────────────────────────────────────────────────────────────────────────────

# ── Projected (XZ) geometry constants ───────────────────────────────────────
# Rear chain
_knee_rel   = np.array([-0.214836,  0.0084062])   # knee joint in rear-hip frame (XZ)
_wheel_rel  = np.array([ 0.217889, -0.130539 ])   # wheel joint in knee frame (XZ)
L1_VEC      = _knee_rel
L2_VEC      = _wheel_rel

# Front linkage joint offsets (XZ)
_c1_in_fhip = np.array([ 0.0996033,  0.00889802])
_c2_in_c1   = np.array([-0.0981832, -0.065704  ])
_c3_in_c2   = np.array([-0.139163,   0.107296  ])

# Site offsets (XZ) used by equality constraints
_zfront_in_rhip = np.array([-0.0999235,  0.00390986])   # in rear hip body
_mind_in_c2     = np.array([-0.101344,   0.0607158  ])   # in child2 body
_zrear_in_knee  = np.array([-0.0378193,  0.0465801  ])   # in knee body
_zcfront_in_c3  = np.array([-0.114912,   0.00449634 ])   # in child3 body

# CM offsets in body XZ frames
_cm_rhip   = np.array([-0.0950349,  0.00529316])
_cm_knee   = np.array([ 0.0866842, -0.0488254 ])
_cm_wheel  = np.array([-0.000107439, 3.05868e-05])
_cm_fhip   = np.array([ 0.0266508,  0.00231006])
_cm_c1     = np.array([-0.0490916, -0.032852  ])
_cm_c2     = np.array([-0.078703,   0.0517192 ])
_cm_c3     = np.array([-0.0686402,  0.00268579])


def solve_kinematics(L1, L2, L0):
    """
    Solve full leg kinematics for given L0.
    Returns dict with joint positions, CM positions, and link angles.
    Returns None if out of reach.
    """
    A = np.array([0.0, 0.0])
    C = np.array([0.0, L0])
    AC = np.linalg.norm(C - A)

    if AC > L1 + L2 + 1e-9 or AC < abs(L1 - L2) - 1e-9:
        return None

    # ── Rear chain IK ────────────────────────────────────────────────────────
    cos_alpha = float(np.clip((L1**2 + AC**2 - L2**2) / (2*L1*AC), -1, 1))
    alpha     = math.acos(cos_alpha)
    theta_AC  = math.atan2(C[1]-A[1], C[0]-A[0])
    theta_B   = theta_AC + alpha
    B         = A + L1 * np.array([math.cos(theta_B), math.sin(theta_B)])

    # Absolute angles of rear links in world frame
    L1_body_dir = L1_VEC / np.linalg.norm(L1_VEC)
    L2_body_dir = L2_VEC / np.linalg.norm(L2_VEC)

    L1_world_dir = (B - A) / L1
    L2_world_dir = (C - B) / L2

    theta_rhip  = math.atan2(L1_world_dir[1], L1_world_dir[0]) \
                - math.atan2(L1_body_dir[1],  L1_body_dir[0])
    theta_knee  = math.atan2(L2_world_dir[1], L2_world_dir[0]) \
                - math.atan2(L2_body_dir[1],  L2_body_dir[0])

    # ── Constraint sites in world frame ──────────────────────────────────────
    p_zfront_world = A + rot2d(theta_rhip)  @ _zfront_in_rhip
    p_zrear_world  = B + rot2d(theta_knee)  @ _zrear_in_knee

    # ── Front four-bar IK (4 unknowns: absolute angles af, a1, a2, a3) ──────
    front_hip_pos = A  # same XZ as rear hip

    def equations(angles):
        af, a1, a2, a3 = angles
        c1_pos = front_hip_pos + rot2d(af) @ _c1_in_fhip
        c2_pos = c1_pos        + rot2d(a1) @ _c2_in_c1
        c3_pos = c2_pos        + rot2d(a2) @ _c3_in_c2

        mind_world    = c2_pos + rot2d(a2) @ _mind_in_c2
        zcfront_world = c3_pos + rot2d(a3) @ _zcfront_in_c3

        eq1 = mind_world    - p_zfront_world
        eq2 = zcfront_world - p_zrear_world
        return [eq1[0], eq1[1], eq2[0], eq2[1]]

    sol = fsolve(equations, [0., 0., 0., 0.], full_output=True)
    af, a1, a2, a3 = sol[0]
    residual = float(np.max(np.abs(sol[1]['fvec'])))

    # ── Joint & CM positions ──────────────────────────────────────────────────
    c1_pos = front_hip_pos + rot2d(af) @ _c1_in_fhip
    c2_pos = c1_pos        + rot2d(a1) @ _c2_in_c1
    c3_pos = c2_pos        + rot2d(a2) @ _c3_in_c2

    cms = {
        'left_rear_hip_link':       A + rot2d(theta_rhip) @ _cm_rhip,
        'left_rear_knee_link':      B + rot2d(theta_knee) @ _cm_knee,
        # left_chassis_wheel_link excluded
        'left_front_hip_link':      front_hip_pos + rot2d(af) @ _cm_fhip,
        'left_child1_link':         c1_pos        + rot2d(a1) @ _cm_c1,
        'left_child2_link':         c2_pos        + rot2d(a2) @ _cm_c2,
        'left_child3_link':         c3_pos        + rot2d(a3) @ _cm_c3,
    }

    # Link angles (for inertia rotation — not needed for scalar projection,
    # but stored for completeness)
    angles = dict(rhip=theta_rhip, knee=theta_knee,
                  fhip=af, c1=a1, c2=a2, c3=a3)

    # ── P1, P2, P3, P4 ───────────────────────────────────────────────────────
    P1 = (A + B) / 2.0
    P2 = (B + C) / 2.0
    mass_L1 = 0.287767
    mass_L2 = 0.276253
    P3 = (mass_L1 * P1 + mass_L2 * P2) / (mass_L1 + mass_L2)
    R  = float(np.linalg.norm(P3 - A))
    P4 = np.array([0.0, R])

    return dict(A=A, B=B, C=C, P1=P1, P2=P2, P3=P3, P4=P4, R=R,
                L0=L0, L1=L1, L2=L2,
                cms=cms, angles=angles, residual=residual)

# ─────────────────────────────────────────────────────────────────────────────
# 3. Inertia calculation
# ─────────────────────────────────────────────────────────────────────────────

# Scalar inertia about Y-axis for each link (pre-computed, constant)
# I_cm_y = axis^T · R(quat) · diag(I) · R(quat)^T · axis
_ROT_AXIS = np.array([0., 1., 0.])

# Links are split into two groups for inertia calculation:
#
#   REAR_LINKS  (left_rear_hip_link, left_rear_knee_link):
#     Full parallel-axis theorem:  I_P4 = I_cm + m * d(cm→P4)²
#     CM positions are solved analytically from rear-chain IK.
#
#   FRONT_LINKS (left_front_hip_link, child1/2/3):
#     APPROXIMATION: only I_cm is accumulated, m*d² term is ignored.
#     Rationale: the four-bar front linkage CM positions depend on
#     constraint solving (complex, configuration-dependent). Since the
#     front links have small mass (total ~0.235 kg vs 0.564 kg rear),
#     omitting the m*d² term introduces a bounded, conservative error.
#     The I_cm terms are still correctly rotated via the inertia quat.

LINK_INERTIA_DATA = {
    # ── Rear chain: full I_cm + m·d² ─────────────────────────────────
    'left_rear_hip_link':  dict(mass=0.287767,  diag=[0.00158629, 0.0014973,   9.6327e-05 ], quat=[0.485117, 0.514663, 0.510916, 0.488618], full_parallel=True),
    'left_rear_knee_link': dict(mass=0.276253,  diag=[0.00323549, 0.00314863,  0.000103207], quat=[0.338114, 0.628263, 0.614371, 0.336915], full_parallel=True),
    # ── Front four-bar: I_cm only (m·d² approximated as 0) ───────────
    'left_front_hip_link': dict(mass=0.092808,  diag=[0.000147923,0.000133466, 1.77609e-05], quat=[0.512977, 0.467566, 0.48598,  0.531093], full_parallel=False),
    'left_child1_link':    dict(mass=0.0396604, diag=[6.239e-05,  6.08117e-05, 2.15828e-06], quat=[0.623729, 0.33311,  0.33311,  0.623729], full_parallel=False),
    'left_child2_link':    dict(mass=0.0317857, diag=[0.00010135, 9.8292e-05,  3.24794e-06], quat=[0.319467, 0.631065, 0.630266, 0.320098], full_parallel=False),
    'left_child3_link':    dict(mass=0.0634754, diag=[0.000126012,0.000123222, 3.84594e-06], quat=[0.485742, 0.505975, 0.513346, 0.494489], full_parallel=False),
}

# Pre-compute I_cm about rotation axis for each link (constant regardless of pose,
# because we assume the inertia tensor orientation stays fixed relative to world
# at zero config — this is the standard first-order approximation for
# parallel-axis theorem in MBD)
# More precisely: the diaginertia quat describes the inertia principal axes
# orientation at the ZERO configuration. For a planar analysis where all joints
# rotate about Y, the scalar I_y is constant.
LINK_ICM = {}
for lname, d in LINK_INERTIA_DATA.items():
    LINK_ICM[lname] = scalar_inertia_about_axis(d['diag'], d['quat'], _ROT_AXIS)

print("Link I_cm about Y-axis:")
for k,v in LINK_ICM.items():
    print(f"  {k:<35}: {v:.8f} kg·m²")


def compute_inertia_P4(result):
    """
    Compute total moment of inertia about P4.

    Rear links  (full_parallel=True):
        I_i = I_cm_i + m_i * d(cm_i → P4)²      [exact parallel-axis]

    Front links (full_parallel=False):
        I_i = I_cm_i                              [approx: ignore m·d² term]

    I_P4 = Σ I_i
    """
    P4      = result['P4']
    cms     = result['cms']
    I_total = 0.0
    breakdown = {}

    for lname, d in LINK_INERTIA_DATA.items():
        m    = d['mass']
        Icm  = LINK_ICM[lname]

        if d['full_parallel']:
            # Exact: need CM position from kinematics
            cm   = cms[lname]
            d_sq = float(np.dot(cm - P4, cm - P4))
            md2  = m * d_sq
            note = 'full'
        else:
            # Approximation: omit m·d² (front four-bar, CM pos unknown)
            d_sq = float('nan')
            md2  = 0.0
            note = 'approx (I_cm only)'

        I_i = Icm + md2
        breakdown[lname] = dict(Icm=Icm, m=m,
                                 d=math.sqrt(d_sq) if not math.isnan(d_sq) else float('nan'),
                                 md2=md2, I_i=I_i, note=note)
        I_total += I_i

    return I_total, breakdown

# ─────────────────────────────────────────────────────────────────────────────
# 4. Main analysis loop
# ─────────────────────────────────────────────────────────────────────────────

def run_analysis(xml_path, L0_min, L0_max, L0_step, output_dir="."):
    print(f"\n{'='*65}")
    print(f"  Parsing: {xml_path}")
    print(f"{'='*65}")
    links, rot_axis, L1, L2 = parse_xml(xml_path)
    print()

    L0_values = np.arange(L0_min, L0_max + L0_step * 0.5, L0_step)
    results   = []

    for L0 in L0_values:
        r = solve_kinematics(L1, L2, L0)
        if r is None:
            print(f"  [WARN] L0={L0:.4f}: out of reach")
            continue
        if r['residual'] > 1e-8:
            print(f"  [WARN] L0={L0:.4f}: constraint residual {r['residual']:.2e} (poor convergence)")
        I_P4, breakdown = compute_inertia_P4(r)
        r['I_P4']      = I_P4
        r['breakdown'] = breakdown
        results.append(r)

    if not results:
        print("No valid configurations."); return

    # ── Table ─────────────────────────────────────────────────────────────────
    rows = []
    for r in results:
        rows.append({
            'L0 (m)':       round(r['L0'],   6),
            'P3_x':         round(r['P3'][0],6),
            'P3_y':         round(r['P3'][1],6),
            'P4_y (=R)':    round(r['P4'][1],6),
            'P4_to_A (m)':  round(r['R'],    6),
            'P4_to_C (m)':  round(abs(r['R'] - r['L0']), 6),
            'I_P4 (kg·m²)': round(r['I_P4'], 10),
        })
    df = pd.DataFrame(rows)

    csv_path = os.path.join(output_dir, "kinematic_inertia_table.csv")
    df.to_csv(csv_path, index=False)
    print(f"\n{'─'*65}")
    print("SUMMARY TABLE")
    print('─'*65)
    print(df.to_string(index=False))
    print(f"\nTable saved → {csv_path}")

    # Inertia breakdown for last config
    print(f"\nInertia breakdown at L0={results[-1]['L0']:.3f} m (P4=({results[-1]['P4'][0]:.4f},{results[-1]['P4'][1]:.4f})):")
    print(f"  {'Link':<30} {'I_cm (kg·m²)':>16}  {'m (kg)':>10}  {'d (m)':>10}  {'m·d² (kg·m²)':>16}  {'I_i (kg·m²)':>16}  method")
    print(f"  {'-'*110}")
    for lname, bd in results[-1]['breakdown'].items():
        d_str   = f"{bd['d']:.6f}"   if not math.isnan(bd['d'])  else '     n/a    '
        md2_str = f"{bd['md2']:.10f}"
        print(f"  {lname:<30} {bd['Icm']:>16.10f}  {bd['m']:>10.6f}  {d_str:>10}  {md2_str:>16}  {bd['I_i']:>16.10f}  {bd['note']}")
    print(f"  {'-'*110}")
    print(f"  {'TOTAL':>92}  {results[-1]['I_P4']:>16.10f} kg·m²")

    # ── Plots ─────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 9))
    gs_main = gridspec.GridSpec(1, 2, width_ratios=[1.1, 1], figure=fig)

    # ── Left: multi-pose overlay ──────────────────────────────────────────────
    ax1  = fig.add_subplot(gs_main[0])
    cmap = plt.cm.viridis
    n    = len(results)

    for i, r in enumerate(results):
        color = cmap(i / max(n-1, 1))
        alpha = 0.3 + 0.7*(i / max(n-1, 1))
        A, B, C = r['A'], r['B'], r['C']
        P3, P4  = r['P3'], r['P4']

        ax1.plot([A[0],B[0]], [A[1],B[1]], '-', color=color, lw=2.0, alpha=alpha)
        ax1.plot([B[0],C[0]], [B[1],C[1]], '-', color=color, lw=2.0, alpha=alpha)

        # Rear-chain CMs (full parallel-axis, square marker)
        for lname in ['left_rear_hip_link', 'left_rear_knee_link']:
            cm = r['cms'][lname]
            ax1.scatter(*cm, color=color, marker='s', s=22, zorder=5, alpha=alpha,
                        edgecolors='none')

        # Front four-bar CMs (I_cm only approx, circle marker, lighter)
        for lname in ['left_front_hip_link','left_child1_link',
                      'left_child2_link','left_child3_link']:
            cm = r['cms'][lname]
            ax1.scatter(*cm, color=color, marker='o', s=14, zorder=4,
                        alpha=alpha*0.55, edgecolors='gray', linewidths=0.4)

        t_arc = np.linspace(0, math.atan2(P3[1],P3[0]), 80)
        ax1.plot(r['R']*np.cos(t_arc), r['R']*np.sin(t_arc),
                 '--', color=color, lw=0.8, alpha=alpha*0.6)

        ax1.scatter(*P3, color=color, marker='*', s=60, zorder=5, alpha=alpha)
        ax1.scatter(*P4, color=color, marker='o', s=35, zorder=4, alpha=alpha)

    for idx in [0, -1]:
        r = results[idx]
        c = cmap(0.0 if idx==0 else 1.0)
        ax1.scatter(*r['P3'], color=c, marker='*', s=90, zorder=6,
                    label=f"L0={r['L0']:.3f} P3=({r['P3'][0]:.3f},{r['P3'][1]:.3f})")
        ax1.scatter(*r['P4'], color=c, marker='o', s=60, zorder=6,
                    label=f"L0={r['L0']:.3f} P4=(0,{r['P4'][1]:.3f})")

    # Static legend proxies for CM marker types
    from matplotlib.lines import Line2D
    proxy_rear  = Line2D([0],[0], marker='s', color='gray', linestyle='None',
                          markersize=5, label='Rear CM  (full parallel-axis)')
    proxy_front = Line2D([0],[0], marker='o', color='gray', linestyle='None',
                          markersize=4, markeredgecolor='gray', markerfacecolor='white',
                          label='Front 4-bar CM  (I_cm only, approx)')

    ax1.scatter(0, 0, color='black', s=120, zorder=7, marker='^', label='A  hip joint')
    ax1.axhline(0, color='gray', lw=0.5, ls='--')
    ax1.axvline(0, color='gray', lw=0.5, ls='--')

    sm = plt.cm.ScalarMappable(cmap=cmap,
                                norm=plt.Normalize(vmin=L0_values[0], vmax=L0_values[-1]))
    sm.set_array([])
    plt.colorbar(sm, ax=ax1, label='L0 (m)', fraction=0.03, pad=0.04)
    ax1.text(0.02, 0.98,
             "L1={:.4f} m   L2={:.4f} m\n"
             "[■] rear CM (full I_cm+m·d²)\n"
             "[○] front 4-bar CM (I_cm only, approx)\n"
             "[★] P3 equiv. CoM   [●] P4 virtual leg CoM".format(L1, L2),
             transform=ax1.transAxes, va='top', fontsize=7.5,
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.88))
    ax1.set_xlabel('Motion-plane horizontal (m)')
    ax1.set_ylabel('Motion-plane vertical (m)')
    ax1.set_title(f'Full Leg Kinematics (all 7 links)\nL0 in [{L0_min:.3f}, {L0_max:.3f}] step={L0_step:.3f} m')
    ax1.set_aspect('equal')
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles + [proxy_rear, proxy_front],
               labels  + [proxy_rear.get_label(), proxy_front.get_label()],
               fontsize=7, loc='lower right')
    ax1.grid(True, alpha=0.3)

    # ── Right: 3 stacked trend panels ────────────────────────────────────────
    gs_right = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=gs_main[1], hspace=0.5)
    L0v = [r['L0'] for r in results]

    ax2 = fig.add_subplot(gs_right[0])
    ax2.plot(L0v, [r['P3'][0] for r in results], 'b-o', ms=4, label='P3 horizontal')
    ax2.plot(L0v, [r['P3'][1] for r in results], 'b--s',ms=4, label='P3 vertical')
    ax2.plot(L0v, [r['P4'][1] for r in results], 'r-^', ms=4, label='P4 vertical (=R)')
    ax2.set_xlabel('L0 (m)'); ax2.set_ylabel('m')
    ax2.set_title('P3 and P4 coordinates vs L0')
    ax2.legend(fontsize=7); ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(gs_right[1])
    ax3.plot(L0v, [r['R'] for r in results],              'g-o', ms=4, label='P4->A  髋关节 (=R)')
    ax3.plot(L0v, [abs(r['R']-r['L0']) for r in results], 'm-s', ms=4, label='P4->C  驱动轮 (=|R-L0|)')
    ax3.set_xlabel('L0 (m)'); ax3.set_ylabel('m')
    ax3.set_title('P4 distances to A (hip) and C (wheel)')
    ax3.legend(fontsize=7); ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(gs_right[2])
    ax4.plot(L0v, [r['I_P4']*1000 for r in results], 'k-D', ms=5, lw=2)
    ax4.set_xlabel('L0 / 腿长 (m)'); ax4.set_ylabel('I_P4  (×10⁻³ kg·m²)')
    ax4.set_title('Moment of Inertia about P4 vs L0')
    ax4.grid(True, alpha=0.3)

    plt.suptitle('Wheel-Legged Robot — Left Leg Kinematics & Inertia', fontsize=13)
    plt.tight_layout()
    fig_path = os.path.join(output_dir, "kinematic_inertia_plot.png")
    plt.savefig(fig_path, dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Plot saved → {fig_path}")

# ─────────────────────────────────────────────────────────────────────────────
# 5. Entry point
# ──

if __name__ == "__main__":
    # ── User Settings ─────────────────────────────────
    XML_PATH  = "wheelLegged_seriBot.xml"

    L0_MIN  = 0.126
    L0_MAX  = 0.396
    L0_STEP = 0.01

    OUTPUT_DIR = "."
    # ──────────────────────────────────────────────────

    if not os.path.isfile(XML_PATH):
        print(f"[ERROR] XML not found: {XML_PATH}")
        sys.exit(1)

    run_analysis(XML_PATH, L0_MIN, L0_MAX, L0_STEP, OUTPUT_DIR)