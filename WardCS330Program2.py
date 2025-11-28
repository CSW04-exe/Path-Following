# ---------------------------------------------------------------
# Author : Carter Ward
# Class  : CS 330-1
# Date   : 10/13/25
#
# Simulates 1 character moving in 2D by following a path (“chase the rabbit”) using Seek.
# Uses Newton–Euler-1 to update motion and writes the trajectory to a text file.
# ---------------------------------------------------------------

import math

# -------------------------
# Behavior codes (field 10)
# -------------------------
SEEK         = 6
FOLLOW_PATH  = 11

# -------------------------
# Simulation settings
# -------------------------
DT                = 0.50    # time step
STOP_TIME         = 125.0   # final time
TINY_SPEED_THRESH = 0.02    # snap tiny speeds to 0

# -------------------------
# 2D vector helpers (x, z)
# -------------------------
def v_add(a, b):   return (a[0] + b[0], a[1] + b[1])
def v_sub(a, b):   return (a[0] - b[0], a[1] - b[1])
def v_scale(v, s): return (v[0] * s, v[1] * s)
def v_len(v):      return math.hypot(v[0], v[1])
def v_norm(v):
    L = v_len(v)
    return (0.0, 0.0) if L == 0.0 else (v[0] / L, v[1] / L)
def v_dot(a, b):   return a[0]*b[0] + a[1]*b[1]

def dist_pp(A, B):         # point-point distance
    return v_len(v_sub(B, A))

def closest_point_segment(Q, A, B):
    """Closest point from Q to segment AB (standard projection + clamp)."""
    AB = v_sub(B, A)
    denom = v_dot(AB, AB)
    if denom == 0.0:
        return A
    t = v_dot(v_sub(Q, A), AB) / denom
    if t <= 0.0: return A
    if t >= 1.0: return B
    return v_add(A, v_scale(AB, t))

# -------------------------
# Path representation/utilities
# -------------------------
class Path:
    """Polyline with cumulative distance + normalized parameter [0,1]."""
    def __init__(self, path_id, xs, zs):
        self.id = path_id
        self.x = xs[:]          # list of x
        self.z = zs[:]          # list of z
        self.segments = len(xs) - 1

        # cumulative distances at vertices
        self.cumdist = [0.0] * (self.segments + 1)
        for i in range(1, self.segments + 1):
            A = (self.x[i-1], self.z[i-1])
            B = (self.x[i],   self.z[i])
            self.cumdist[i] = self.cumdist[i-1] + dist_pp(A, B)

        total = self.cumdist[-1] if self.cumdist[-1] > 0 else 1.0
        self.param = [d / total for d in self.cumdist]  # vertex params in [0,1]

    def get_position(self, p):
        """Point on path at parameter p (linear interpolate along segment)."""
        # clamp ends
        if p <= self.param[0]:
            return (self.x[0], self.z[0])
        if p >= self.param[-1]:
            return (self.x[-1], self.z[-1])

        # find segment with param[i] <= p <= param[i+1]
        i = 0
        for k in range(self.segments):
            if self.param[k] <= p <= self.param[k+1]:
                i = k
                break

        A = (self.x[i],   self.z[i])
        B = (self.x[i+1], self.z[i+1])

        denom = (self.param[i+1] - self.param[i])
        t = 0.0 if denom == 0.0 else (p - self.param[i]) / denom
        return v_add(A, v_scale(v_sub(B, A), t))

    def get_param(self, pos):
        """Closest point on path to pos → return its normalized parameter."""
        best_seg = 0
        best_pt  = (self.x[0], self.z[0])
        best_d   = float('inf')

        for i in range(self.segments):
            A = (self.x[i],   self.z[i])
            B = (self.x[i+1], self.z[i+1])
            C = closest_point_segment(pos, A, B)
            d = dist_pp(pos, C)
            if d < best_d:
                best_d  = d
                best_seg = i
                best_pt  = C

        # convert closest point back to param using local segment fraction
        A  = (self.x[best_seg],   self.z[best_seg])
        B  = (self.x[best_seg+1], self.z[best_seg+1])
        Ap = self.param[best_seg]
        Bp = self.param[best_seg+1]
        AB = v_sub(B, A)
        denom = v_len(AB)
        frac = 0.0 if denom == 0.0 else (v_len(v_sub(best_pt, A)) / denom)
        return Ap + frac * (Bp - Ap)

# -------------------------
# Character (minimal fields)
# -------------------------
class Character:
    def __init__(self, cid, steer_code,
                 position=(0.0, 0.0), velocity=(0.0, 0.0),
                 orientation=0.0, rotation=0.0,
                 max_speed=0.0, max_linear=0.0,
                 path=None, path_offset=0.0):
        self.id = cid
        self.steer = steer_code
        self.pos = position
        self.vel = velocity
        self.ori = orientation
        self.rot = rotation
        self.lin_acc = (0.0, 0.0)   # for output
        self.ang_acc = 0.0
        self.max_speed  = max_speed
        self.max_linear = max_linear
        self.path = path
        self.path_offset = path_offset
        self.collided = False       # output compatibility

# -------------------------
# Steering: Seek (reuse P1)
# -------------------------
def steering_seek(ch, target_pos):
    to_target = v_sub(target_pos, ch.pos)
    lin = v_scale(v_norm(to_target), ch.max_linear)
    return (lin, 0.0)

# -------------------------
# Steering: Follow Path (chase the rabbit)
# - find nearest path param
# - add offset
# - get that point and Seek to it
# -------------------------
def steering_follow_path(ch):
    p_now = ch.path.get_param(ch.pos)
    p_tgt = p_now + ch.path_offset
    if p_tgt > 1.0:               # don’t overrun end
        p_tgt = 1.0
    target_pos = ch.path.get_position(p_tgt)
    return steering_seek(ch, target_pos)

# -------------------------
# Newton–Euler-1 update
# -------------------------
def integrate_NE1(ch, lin_acc, ang_acc, dt):
    # position/orientation from current velocity/rotation
    ch.pos = v_add(ch.pos, v_scale(ch.vel, dt))
    ch.ori = (ch.ori + ch.rot * dt) % (2.0 * math.pi)

    # velocity/rotation from accelerations
    ch.vel = v_add(ch.vel, v_scale(lin_acc, dt))
    ch.rot = ch.rot + ang_acc * dt

    # keep what we applied (for output)
    ch.lin_acc = lin_acc
    ch.ang_acc = ang_acc

    # kill micro jitter
    if v_len(ch.vel) < TINY_SPEED_THRESH:
        ch.vel = (0.0, 0.0)

    # cap speed
    speed = v_len(ch.vel)
    if ch.max_speed > 0.0 and speed > ch.max_speed:
        ch.vel = v_scale(v_norm(ch.vel), ch.max_speed)

# -------------------------
# Scenario (PA2)
# One character following the 8-vertex path
# -------------------------
path_x = [  0, -20,  20, -40,  40, -60,  60,   0]
path_z = [ 90,  65,  40,  15, -10, -35, -60, -85]
the_path = Path(1, path_x, path_z)

ch2701 = Character(
    cid=2701,
    steer_code=FOLLOW_PATH,
    position=(20.0, 95.0),
    velocity=(0.0, 0.0),
    orientation=0.0,
    rotation=0.0,
    max_speed=4.0,
    max_linear=2.0,
    path=the_path,
    path_offset=0.04
)

characters = [ch2701]

# -------------------------
# Output: same 11 fields as P1
# time,id,posx,posz,velx,velz,linx,linz,orientation,steer_code,collided
# -------------------------
def write_record(fh, t, c):
    row = [
        f"{t:.2f}", str(c.id),
        f"{c.pos[0]:.6f}", f"{c.pos[1]:.6f}",
        f"{c.vel[0]:.6f}", f"{c.vel[1]:.6f}",
        f"{c.lin_acc[0]:.6f}", f"{c.lin_acc[1]:-.6f}".replace("-", "") if c.lin_acc[1]==-0.0 else f"{c.lin_acc[1]:.6f}",
        f"{c.ori:.6f}",
        str(c.steer),
        "FALSE"
    ]
    fh.write(",".join(row) + "\n")

# -------------------------
# Main
# -------------------------
def main():
    out_name = "dynamic_trajectories.txt"
    with open(out_name, "w", encoding="utf-8") as fh:
        t = 0.0
        # initial snapshot
        for c in characters:
            write_record(fh, t, c)

        steps = int(STOP_TIME / DT)
        for _ in range(steps):
            t += DT

            # steering phase
            steering = []
            for c in characters:
                if c.steer == FOLLOW_PATH:
                    s = steering_follow_path(c)
                elif c.steer == SEEK:   # kept for reuse patterns; not used here
                    s = ((0.0, 0.0), 0.0)
                else:
                    s = ((0.0, 0.0), 0.0)
                steering.append(s)

            # integrate phase
            for c, (lin, ang) in zip(characters, steering):
                integrate_NE1(c, lin, ang, DT)

            # log snapshot
            for c in characters:
                write_record(fh, t, c)

    print(f"Wrote trajectories to {out_name}")

if __name__ == "__main__":
    main()
