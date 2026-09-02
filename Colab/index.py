"""
Interactive fruit sorter: a big platform with 10+ random fruit (at least one of
each type, random counts, random well-spaced positions kept clear of the edges).
Press "Sort all fruit" once and the arm sorts everything into labelled baskets.

Needs: conda install -c conda-forge opencv -y
Run:   python pick_place_sort.py
"""

import pybullet as p
import pybullet_data
import numpy as np
import cv2
import math
import time

# ---------------------------------------------------------------- setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.loadURDF("plane.urdf")

TABLE_C = [0.60, 0.0]
TABLE_HX, TABLE_HY = 0.40, 0.42           # half-extents -> 0.80 x 0.84 top
table_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[TABLE_HX, TABLE_HY, 0.20])
table_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[TABLE_HX, TABLE_HY, 0.20],
                                rgbaColor=[0.55, 0.40, 0.25, 1])
p.createMultiBody(0, table_col, table_vis,
                  basePosition=[TABLE_C[0], TABLE_C[1], 0.20])
TABLE_TOP = 0.40
FRUIT_Z = TABLE_TOP + 0.04
SPAWN_Z = TABLE_TOP + 0.12

# taller containment wall
WALL_H = 0.05
WALL_IDS = []
for dx, dy, hx, hy in [(-TABLE_HX, 0, 0.008, TABLE_HY),
                       (TABLE_HX, 0, 0.008, TABLE_HY),
                       (0, -TABLE_HY, TABLE_HX, 0.008),
                       (0, TABLE_HY, TABLE_HX, 0.008)]:
    c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, WALL_H])
    v = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, WALL_H],
                            rgbaColor=[0.35, 0.35, 0.38, 1])
    wid = p.createMultiBody(0, c, v, basePosition=[TABLE_C[0] + dx,
                                                   TABLE_C[1] + dy,
                                                   TABLE_TOP + WALL_H])
    WALL_IDS.append(wid)

# ---------------------------------------------------------------- fruit types
# Hue ranges are NON-OVERLAPPING so apple and orange can't both match one fruit:
#   apple   red      0-7 and 172-179
#   orange           8-19
#   banana  yellow   20-40
#   grapes  green    43-85
#   blueberry blue   100-135
TYPES = {
    "apple":     {"rgb": [0.85, 0.12, 0.12],
                  "hsv": [([0, 150, 60], [7, 255, 255]),
                          ([172, 150, 60], [179, 255, 255])]},
    "orange":    {"rgb": [0.95, 0.50, 0.08],
                  "hsv": [([8, 150, 60], [19, 255, 255])]},
    "banana":    {"rgb": [0.95, 0.85, 0.15],
                  "hsv": [([20, 130, 60], [40, 255, 255])]},
    "grapes":    {"rgb": [0.55, 0.78, 0.28],
                  "hsv": [([43, 60, 50], [85, 255, 255])]},
    "blueberry": {"rgb": [0.20, 0.25, 0.78],
                  "hsv": [([100, 110, 50], [135, 255, 255])]},
}
# fruits under here
# meshes for the shaped fruit; grapes are a simple small green sphere
SCALE = {"apple": 0.085, "orange": 0.088, "banana": 0.150, "blueberry": 0.070}
GRAPE_R = 0.035

BASKETS = {
    "apple":     [0.66, -0.70],
    "orange":    [0.98, -0.60],
    "banana":    [1.14, 0.00],
    "grapes":    [0.98, 0.60],
    "blueberry": [0.66, 0.70],
}


def build_basket(cx, cy, color, label):
    for dx, dy, hx, hy in [(0, -0.11, 0.11, 0.01), (0, 0.11, 0.11, 0.01),
                           (-0.11, 0, 0.01, 0.11), (0.11, 0, 0.01, 0.11)]:
        c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, 0.06])
        v = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, 0.06],
                                rgbaColor=color[:3] + [1])
        p.createMultiBody(0, c, v, basePosition=[cx + dx, cy + dy, 0.06])
    p.addUserDebugText(label, [cx, cy, 0.22], textColorRGB=[1, 1, 1], textSize=1.4)


for name, (bx, by) in BASKETS.items():
    build_basket(bx, by, TYPES[name]["rgb"], name)

# ---------------------------------------------------------------- fruits
SPACING = 0.15


def free_spot(placed):
    for _ in range(500):
        # spawn well INSIDE the walls so the claw never reaches the barrier
        x = np.random.uniform(0.34, 0.86)
        y = np.random.uniform(-0.28, 0.28)
        if all(math.dist((x, y), q) > SPACING for q in placed):
            return x, y
    return None


counts = {n: int(np.random.randint(1, 3)) for n in TYPES}   # >=1 of each
names = list(TYPES)
while sum(counts.values()) < 10:
    counts[names[np.random.randint(len(names))]] += 1

on_table = {}
placed = []
for name, n in counts.items():
    for _ in range(n):
        spot = free_spot(placed)
        if spot is None:
            continue
        placed.append(spot)
        if name == "grapes":
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=GRAPE_R)
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=GRAPE_R,
                                      rgbaColor=TYPES[name]["rgb"] + [1])
            fid = p.createMultiBody(0.05, col, vis,
                                    basePosition=[spot[0], spot[1], SPAWN_Z])
        else:
            s = SCALE[name]
            col = p.createCollisionShape(p.GEOM_MESH, fileName=f"meshes/{name}.obj",
                                         meshScale=[s, s, s])
            vis = p.createVisualShape(p.GEOM_MESH, fileName=f"meshes/{name}.obj",
                                      meshScale=[s, s, s],
                                      rgbaColor=TYPES[name]["rgb"] + [1])
            fid = p.createMultiBody(0.05, col, vis,
                                    basePosition=[spot[0], spot[1], SPAWN_Z])
            try:
                tex = p.loadTexture(f"textures/{name}.png")
                p.changeVisualShape(fid, -1, textureUniqueId=tex)
            except Exception:
                pass
        p.changeDynamics(fid, -1, lateralFriction=1.5, rollingFriction=0.002)
        on_table[fid] = name

print(f"spawned {len(on_table)} fruits: "
      + ", ".join(f"{k}x{sum(1 for t in on_table.values() if t == k)}" for k in TYPES))

# ---------------------------------------------------------------- arm (longer)
L1, L2 = 0.72, 0.66
pillar_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.25])
pillar_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.25],
                                 rgbaColor=[0.30, 0.30, 0.35, 1])


def seg(color, length):
    he = [0.03, 0.03, length / 2]
    off = [0, 0, length / 2]
    c = p.createCollisionShape(p.GEOM_BOX, halfExtents=he, collisionFramePosition=off)
    v = p.createVisualShape(p.GEOM_BOX, halfExtents=he, rgbaColor=color,
                            visualFramePosition=off)
    return c, v


yaw_c = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=0.06,
                               collisionFramePosition=[0, 0, 0.03])
yaw_v = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.06,
                            rgbaColor=[0.40, 0.40, 0.45, 1],
                            visualFramePosition=[0, 0, 0.03])
sh_c, sh_v = seg([0.20, 0.45, 0.75, 1], L1)
el_c, el_v = seg([0.20, 0.55, 0.85, 1], L2)
palm_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.035, 0.045, 0.02],
                                collisionFramePosition=[0, 0, 0.02])
palm_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.035, 0.045, 0.02],
                             rgbaColor=[0.25, 0.25, 0.30, 1],
                             visualFramePosition=[0, 0, 0.02])
fin_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.012, 0.010, 0.04],
                               collisionFramePosition=[0, 0, 0.04])
fin_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.012, 0.010, 0.04],
                            rgbaColor=[0.15, 0.15, 0.18, 1],
                            visualFramePosition=[0, 0, 0.04])

masses    = [0.3, 0.6, 0.5, 0.2, 0.05, 0.05]
cols      = [yaw_c, sh_c, el_c, palm_c, fin_c, fin_c]
viss      = [yaw_v, sh_v, el_v, palm_v, fin_v, fin_v]
link_pos  = [[0, 0, 0.25], [0, 0, 0.06], [0, 0, L1], [0, 0, L2],
             [0, -0.032, 0.03], [0, 0.032, 0.03]]
inert_pos = [[0, 0, 0.03], [0, 0, L1 / 2], [0, 0, L2 / 2], [0, 0, 0.02],
             [0, 0, 0.04], [0, 0, 0.04]]
parents   = [0, 1, 2, 3, 4, 4]
jtypes    = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE,
             p.JOINT_FIXED, p.JOINT_PRISMATIC, p.JOINT_PRISMATIC]
jaxes     = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 0, 0], [0, 1, 0], [0, 1, 0]]

arm = p.createMultiBody(
    baseMass=0, baseCollisionShapeIndex=pillar_col,
    baseVisualShapeIndex=pillar_vis, basePosition=[0, 0, 0.25],
    linkMasses=masses, linkCollisionShapeIndices=cols,
    linkVisualShapeIndices=viss, linkPositions=link_pos,
    linkOrientations=[[0, 0, 0, 1]] * 6, linkInertialFramePositions=inert_pos,
    linkInertialFrameOrientations=[[0, 0, 0, 1]] * 6,
    linkParentIndices=parents, linkJointTypes=jtypes, linkJointAxis=jaxes,
)

YAW, SHOULDER, ELBOW, PALM, LFING, RFING = 0, 1, 2, 3, 4, 5
ARM_JOINTS = [YAW, SHOULDER, ELBOW]
for j in (LFING, RFING):
    p.changeDynamics(arm, j, lateralFriction=1.5)

for wall in WALL_IDS:
    for link in range(-1, p.getNumJoints(arm)):
        p.setCollisionFilterPair(arm, wall, link, -1, enableCollision=0)

# ---------------------------------------------------------------- camera
CAM_W = CAM_H = 300
CAM_VIEW = p.computeViewMatrix([TABLE_C[0], 0.0, 1.60],
                               [TABLE_C[0], 0.0, 0.4], [0, 1, 0])
CAM_PROJ = p.computeProjectionMatrixFOV(45, 1.0, 0.1, 3.0)
_PV = np.linalg.inv(np.array(CAM_PROJ).reshape(4, 4, order="F")
                    @ np.array(CAM_VIEW).reshape(4, 4, order="F"))


def snapshot():
    img = p.getCameraImage(CAM_W, CAM_H, CAM_VIEW, CAM_PROJ,
                           renderer=p.ER_BULLET_HARDWARE_OPENGL)
    return np.reshape(img[2], (CAM_H, CAM_W, 4))[:, :, :3].astype(np.uint8)


def ray_to_plane(u, v, z):
    x = 2 * u / CAM_W - 1
    y = 1 - 2 * v / CAM_H
    near = _PV @ np.array([x, y, -1, 1.0]); near /= near[3]
    far = _PV @ np.array([x, y, 1, 1.0]); far /= far[3]
    o, d = near[:3], far[:3] - near[:3]
    return o + (z - o[2]) / d[2] * d


ROI = (26, 274, 32, 268)   # v0, v1, u0, u1  (tight to the table)


def scan_all():
    rgb = snapshot()
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    roi = np.zeros(hsv.shape[:2], np.uint8)
    roi[ROI[0]:ROI[1], ROI[2]:ROI[3]] = 255
    found = []
    for name, info in TYPES.items():
        mask = None
        for lo, hi in info["hsv"]:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.bitwise_and(mask, roi)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) < 60:
                continue
            M = cv2.moments(c)
            cu, cv_ = M["m10"] / M["m00"], M["m01"] / M["m00"]
            w = ray_to_plane(cu, cv_, FRUIT_Z)
            found.append((name, (float(w[0]), float(w[1]))))
    return found

# ---------------------------------------------------------------- motion
def set_fingers(o):
    p.setJointMotorControl2(arm, LFING, p.POSITION_CONTROL, targetPosition=-o, force=30)
    p.setJointMotorControl2(arm, RFING, p.POSITION_CONTROL, targetPosition=o, force=30)


def palm_pos():
    return p.getLinkState(arm, PALM, computeForwardKinematics=True)[4]


def move_to(target, steps=900, tol=0.02):
    sol = p.calculateInverseKinematics(arm, PALM, target,
                                       maxNumIterations=200, residualThreshold=1e-4)
    for j, val in zip(ARM_JOINTS, sol[:3]):
        p.setJointMotorControl2(arm, j, p.POSITION_CONTROL,
                                targetPosition=val, force=400)
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1 / 240)
        if math.dist(palm_pos(), target) < tol:
            break


def settle(n):
    for _ in range(n):
        p.stepSimulation()
        time.sleep(1 / 240)


def nearest_fruit(px, py):
    best, bd = None, 1e9
    for fid in on_table:
        fp = p.getBasePositionAndOrientation(fid)[0]
        d = math.dist((fp[0], fp[1]), (px, py))
        if d < bd:
            best, bd = fid, d
    return best, bd


PARK = [0.02, 0.0, 1.20]
SEED = list(p.calculateInverseKinematics(
    arm, PALM, [TABLE_C[0], 0.0, FRUIT_Z + 0.22],
    maxNumIterations=200, residualThreshold=1e-4)[:3])


def ready_seed():
    for j, val in zip(ARM_JOINTS, SEED):
        p.resetJointState(arm, j, val)
        p.setJointMotorControl2(arm, j, p.POSITION_CONTROL,
                                targetPosition=val, force=400)


def go_home():
    set_fingers(0.03)
    move_to(PARK, steps=500)


def pick_and_sort(ftype, xy):
    tx, ty = xy
    ready_seed()
    settle(20)
    set_fingers(0.05)
    move_to([tx, ty, 0.68])
    move_to([tx, ty, FRUIT_Z + 0.20], steps=400)
    move_to([tx, ty, FRUIT_Z + 0.10], steps=400)
    move_to([tx, ty, FRUIT_Z + 0.05], steps=400, tol=0.015)
    set_fingers(0.008)
    settle(60)

    fid, dist = nearest_fruit(*palm_pos()[:2])
    if fid is None or dist > 0.08:
        print(f"  {ftype}: nothing in grasp range, skipping")
        go_home()
        return

    pp, po = p.getLinkState(arm, PALM, computeForwardKinematics=True)[4:6]
    fp, fo = p.getBasePositionAndOrientation(fid)
    inv_p, inv_o = p.invertTransform(pp, po)
    rel_p, rel_o = p.multiplyTransforms(inv_p, inv_o, fp, fo)
    grip = p.createConstraint(arm, PALM, fid, -1, p.JOINT_FIXED,
                              [0, 0, 0], rel_p, [0, 0, 0], rel_o, [0, 0, 0, 1])

    bx, by = BASKETS[ftype]
    move_to([tx, ty, 0.72])
    move_to([bx, by, 0.72], steps=1200)
    for _ in range(6):
        px, py, _ = palm_pos()
        if math.dist((px, py), (bx, by)) < 0.05:
            break
        move_to([bx + (bx - px), by + (by - py), 0.72], steps=500, tol=0.02)
    move_to([bx, by, 0.44], steps=500)
    p.removeConstraint(grip)
    set_fingers(0.03)
    settle(150)
    on_table.pop(fid, None)
    print(f"  {ftype} -> basket. {sum(1 for t in on_table.values() if t == ftype)} {ftype}(s) left.")
    go_home()


def sort_type(ftype):
    for _ in range(12):
        dets = [d for d in scan_all() if d[0] == ftype]
        if not dets:
            break
        dets.sort(key=lambda d: math.hypot(*d[1]))
        pick_and_sort(ftype, dets[0][1])

# ---------------------------------------------------------------- run
p.resetDebugVisualizerCamera(cameraDistance=2.6, cameraYaw=50,
                             cameraPitch=-40, cameraTargetPosition=[0.55, 0.0, 0.2])
set_fingers(0.03)
settle(200)
go_home()

sort_btn = p.addUserDebugParameter("Sort all fruit", 1, 0, 1)
prev = p.readUserDebugParameter(sort_btn)
print("Press 'Sort all fruit' once to sort everything.")

try:
    while p.isConnected():
        val = p.readUserDebugParameter(sort_btn)
        if val > prev:
            prev = val
            print("sorting all fruit...")
            for ftype in TYPES:
                sort_type(ftype)
            print("all fruit sorted.")
        p.stepSimulation()
        time.sleep(1 / 240)
except (KeyboardInterrupt, p.error):
    pass