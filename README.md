# Path-Following

A CS 330 simulation where a 2D character steers itself along a hand-defined polyline path using the classic "chase the rabbit" path-following algorithm, built on top of a Newton–Euler-1 physics integrator. The simulation writes every frame to a text log, and a companion plotting script overlays the resulting trajectory, velocity, and acceleration vectors on the path itself.

## 1. Purpose

This is my second AI-for-games assignment for CS 330, following directly on from the dynamic-movement project (Program 1), where I implemented basic steering behaviors like `Seek`, `Flee`, and `Arrive`. Program 2 asks a different question: given only a series of waypoints, how do I make an agent move along that path smoothly, using nothing but local, frame-by-frame steering decisions — no global planner, no full lookahead, just the agent continuously figuring out where it is relative to the path and reacting to that.

I built this project to implement and validate that "Follow Path" steering behavior, and to reuse and extend the simulation/logging/plotting pipeline I had already built for Program 1, since the course requires character motion to come from discrete steering behaviors logged in a fixed record format so it can be replayed and graphed.

## 2. Problem and Approach

The assignment was to simulate a single character (id `2701`) following an 8-vertex path defined by `(x, z)` coordinates, using the "Follow Path" steering behavior (behavior code `11`), and to produce a trajectory log in the same 11-field CSV format I used for `Seek`/`Flee`/`Arrive` in Program 1, so I could reuse the same plotting tool.

I implemented the standard "chase the rabbit" technique:

1. **Project the character onto the path.** `Path.get_param()` finds the closest point on any segment of the polyline to the character's current position, using point-to-segment projection and clamping, then converts that closest point into a single normalized parameter `p` in `[0, 1]` representing how far along the whole path that point sits.
2. **Look slightly ahead.** I add a fixed `path_offset` of `0.04` (4% of the total path length) to that parameter to get a target parameter further down the path — the "rabbit" the character chases.
3. **Seek the target.** `Path.get_position()` converts the target parameter back into an actual `(x, z)` point by interpolating along the right segment, and I reuse my existing `Seek` behavior to steer the character toward it at maximum linear acceleration.
4. **Integrate motion.** A Newton–Euler-1 integrator (`integrate_NE1`) updates position from the previous velocity and velocity from the newly computed acceleration, then clamps speed to `max_speed` and zeroes out any velocity below a tiny threshold so it doesn't jitter in place.

I ran this loop every `DT = 0.5` s for `STOP_TIME = 125` s (250 steps), starting the character at `(20, 95)` — deliberately offset from the path's first vertex `(0, 90)` — so the log also shows it correcting onto the path before it starts tracking it.

## 3. Structure and Methodologies

**Data structures**

- `Path` — a polyline defined by parallel `x`/`z` coordinate lists. In the constructor I precompute cumulative segment distances (`cumdist`) and normalized arc-length parameters (`param`) at each vertex, so any point along the path can be addressed by a single scalar `p ∈ [0, 1]` instead of a segment index plus a local `t`.
- `Character` — a plain data object holding position, velocity, orientation, rotation, current acceleration (kept for logging), speed/acceleration caps, and a reference to the `Path` it follows plus its look-ahead offset.
- 2D vectors as plain `(x, z)` tuples rather than a dedicated class, with a small set of free functions (`v_add`, `v_sub`, `v_scale`, `v_len`, `v_norm`, `v_dot`) implementing the vector algebra I need for projection, distance, and normalization.

**Core algorithms/math**

- Point-to-segment projection (`closest_point_segment`) — dot-product projection of a point onto a line segment, clamped to the segment's endpoints, used to find the nearest point on the path.
- Arc-length parameterization — I map vertices to `[0, 1]` proportionally to cumulative Euclidean distance, so `get_position` and `get_param` act as inverses of each other along the piecewise-linear path.
- Newton–Euler-1 integration for updating position/orientation from velocity/rotation and then velocity/rotation from acceleration, with a speed cap and a "kill micro-jitter" threshold on very small velocities.

**Dependencies/libraries**

- The simulator (`WardCS330Program2.py`) uses only the Python 3 standard library (`math`) — no external simulation dependencies.
- The plotting script (`CS_330_Python_Plotter_withPath_v3_1.py`) uses `matplotlib` for visualization and the built-in `csv` module to parse the trajectory log. It plots position (red), velocity vectors (green), and linear acceleration vectors (blue) against the reference path, drawn as a green dashed line with each vertex labeled by coordinate.
- A flat, comma-separated log file (`dynamic_trajectories.txt`) is the interchange format between the two scripts, using the same fixed 11-column schema (`time, id, posx, posz, velx, velz, linx, linz, orientation, steer_code, collided`) I used across Program 1, so I can swap in a different steering behavior without touching the plotting tool.

## 4. Process

I built this in layers rather than writing it as one program from scratch:

1. **Reuse Program 1's primitives.** `steering_seek` is explicitly commented `# reuse P1` — I carried the `Seek` behavior and its behavior code (`SEEK = 6`) straight over from Program 1, along with the labels for `Flee` and `Arrive` still present in the plotting script's legend logic, even though only `Seek` and `Follow Path` (`FOLLOW_PATH = 11`) are exercised here. That let me focus this assignment purely on the path-following logic.
2. **Build the path abstraction.** I wrote `Path` to answer two questions independently: "where is the point at parameter p?" (`get_position`) and "what parameter is closest to this position?" (`get_param`). Structuring both around the same `param`/`cumdist` table is what makes "chase the rabbit" work — the character never needs to know which segment it's on, only its scalar progress along the whole path.
3. **Define the path.** I hardcoded the assignment's 8-vertex path as parallel `path_x`/`path_z` lists — `[0, -20, 20, -40, 40, -60, 60, 0]` and `[90, 65, 40, 15, -10, -35, -60, -85]` — and wrapped it in a `Path` object.
4. **Compose `Follow Path` out of `Seek`.** `steering_follow_path` calls `get_param`, adds the offset, calls `get_position`, and hands the result straight to `steering_seek` — a small stack of composable behaviors rather than a single monolithic controller.
5. **Simulate the agent following it.** `main()` runs a fixed-timestep loop with two phases per step — compute steering output for every character, then integrate every character — even though only one character (`ch2701`) is instantiated, so the loop generalizes beyond this single scenario.
6. **Log to `dynamic_trajectories.txt`.** `write_record` writes the initial snapshot at `t = 0`, then one row per step for all 250 steps, matching Program 1's exact 11-field format (down to the explicit negative-zero cleanup on `lin_acc[1]`) so the existing plotter could read it unmodified.
7. **Plot to `TrajectoryPlot.png`.** Running `CS_330_Python_Plotter_withPath_v3_1.py` against that log reads every row, groups it by mover id, and draws the traced position, velocity, and acceleration vectors over the labeled reference path — this is how I produced and visually checked `TrajectoryPlot.png`.

**How to run:** run `python WardCS330Program2.py` first to (re)generate `dynamic_trajectories.txt`, then run `python CS_330_Python_Plotter_withPath_v3_1.py` (requires `pip install matplotlib`) from the same folder to view/regenerate the plot.

## 5. Outcome

Over the full 125 s run (250 fixed 0.5 s steps), the character starts at `(20, 95)` — off the path's first vertex `(0, 90)` — and by `t = 125` s has traveled almost the entire path, ending at `(28.85, -79.18)`: close to, but not exactly on, the final vertex `(0, -85)`. `TrajectoryPlot.png` shows the traced red position curve staying tight against the green dashed reference path along every straight segment, while visibly rounding through each sharp vertex instead of hitting it exactly — a direct, observable consequence of chasing a target that is always 4% of the path length ahead rather than the vertex itself.

Working through this taught me:

- **How to turn a steering-behavior description into working numerical code.** Going from "project onto the path, look ahead, seek that point" to actual arc-length parameterization, segment projection, and clamped interpolation meant I had to get several small pieces of vector math right (projection, normalization, distance) for the system to converge onto the path instead of orbiting or drifting away from it.
- **How to design for composability.** Building `Follow Path` as a thin wrapper around my existing `Seek` behavior, and keeping the log format identical to Program 1, is what let me reuse the plotting tool without changes — a concrete demonstration of building on prior work instead of starting over each assignment.
- **How to debug through visualization instead of assertions.** There's no closed-form "correct" trajectory to check against here, so I had to judge correctness qualitatively — by plotting position, velocity, and acceleration together and checking that the shape tracks the path — which is a realistic taste of how motion/AI systems get validated in practice.
- **The real limits of a simple algorithm.** The visible corner-rounding in the output is a known limitation of fixed-offset path following on a piecewise-linear path, and catching it in my own plot (rather than assuming the character traces the path exactly) reflects a real understanding of what "chase the rabbit" does and doesn't guarantee.

Overall, this project showed me I could take a working steering-behavior pipeline from an earlier assignment and extend it with a genuinely different algorithm — one with global path state instead of a single target point — without having to rebuild the simulation loop, character model, or logging/plotting infrastructure from scratch.
