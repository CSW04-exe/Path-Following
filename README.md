# Path-Following

A Python simulation of a 2D autonomous character that steers itself along a hand-defined polyline path, using the classic "chase the rabbit" path-following algorithm on top of a Newton–Euler-1 physics integrator. The simulation writes every frame of motion to a text log, and a companion script plots the resulting trajectory, velocity, and acceleration vectors against the path itself.

## 1. Purpose

This project exists to answer a narrow but foundational question in game AI: given only a series of waypoints, how do you make an agent move along that path smoothly and believably, using nothing but local steering decisions made frame by frame? There is no global planner and no lookahead beyond the next target point — the character has to "discover" the path the same way a simple NPC or vehicle AI would, by continuously finding where it currently is relative to the path and reacting to that.

The repository is built for CS 330 (an AI-for-games course), where the assignment framework requires character motion to be produced by discrete steering behaviors and logged in a specific record format so it can be replayed and graphed. Beyond satisfying that assignment, the project is a small, self-contained testbed for understanding how steering behaviors compose: it reuses a `Seek` behavior from an earlier assignment (`Program 1`) as the inner behavior that `Follow Path` (`Program 2`) is built on top of, which is exactly how these behaviors are meant to be layered in a real steering system.

## 2. Problem and approach

The problem was assigned as part of a CS 330 programming series: simulate a character (id `2701`) that follows an 8-vertex path defined as a list of (x, z) coordinates, using the "Follow Path" steering behavior (behavior code `11`), and produce a trajectory log in the same 11-field CSV format used by the earlier `Seek`/`Flee`/`Arrive` assignments so the same plotting tooling could be reused.

The approach taken is the standard "chase the rabbit" technique:

1. **Project the character onto the path.** `Path.get_param()` finds the closest point on any segment of the polyline to the character's current position (via point-to-segment projection and clamping), and converts that closest point into a single normalized parameter `p` in `[0, 1]` representing how far along the whole path that point is.
2. **Look slightly ahead.** A fixed `path_offset` (0.04, i.e. 4% of total path length) is added to that parameter to produce a target parameter further down the path — the "rabbit" the character chases.
3. **Seek the target.** `Path.get_position()` converts that target parameter back into an actual (x, z) point by interpolating along the correct segment, and the existing `Seek` behavior steers the character toward it at maximum linear acceleration.
4. **Integrate motion.** A Newton–Euler-1 integrator (`integrate_NE1`) updates position from the previous velocity and updates velocity from the newly computed acceleration, then clamps speed to `max_speed` and zeroes out any velocity below a tiny threshold to avoid jitter.

This loop runs every `DT = 0.5` s for `STOP_TIME = 125` s (250 steps), starting the character at `(20, 95)` — deliberately off the path's starting vertex `(0, 90)` — so the log also shows the character correcting onto the path before it starts tracking it.

## 3. Structure and methodologies

**Data structures**

- `Path` — a polyline defined by parallel `x`/`z` coordinate lists, precomputing cumulative segment distances (`cumdist`) and normalized arc-length parameters (`param`) at each vertex so any point along it can be addressed by a single scalar `p ∈ [0, 1]` instead of a segment index and local `t`.
- `Character` — a plain data object holding position, velocity, orientation, rotation, acceleration, speed/acceleration caps, and a reference to the `Path` it follows plus its look-ahead offset.
- 2D vectors are represented as plain `(x, z)` tuples rather than a dedicated vector class, with a small set of free functions (`v_add`, `v_sub`, `v_scale`, `v_len`, `v_norm`, `v_dot`) implementing the vector algebra needed for projection, distance, and normalization.

**Core algorithms/math**

- Point-to-segment projection (`closest_point_segment`) — the standard dot-product projection of a point onto a line segment, clamped to the segment's endpoints, used to find the nearest point on the path.
- Arc-length parameterization — vertices are mapped to `[0, 1]` proportionally to cumulative Euclidean distance, so `get_position`/`get_param` are inverses of each other along piecewise-linear geometry.
- Newton–Euler-1 (semi-implicit-style) integration for updating position/orientation from velocity/rotation and then velocity/rotation from acceleration, with a speed cap and a "kill micro-jitter" threshold.

**Dependencies / frameworks**

- Pure Python 3 standard library (`math`) for the simulation itself (`WardCS330Program2.py`) — no external simulation dependencies.
- `matplotlib` and the built-in `csv` module for the visualization script (`CS_330_Python_Plotter_withPath_v3_1.py`), which reads the CSV trajectory log and plots position (red), velocity vectors (green), and linear acceleration vectors (blue) against the reference path (green dashed line with labeled vertices).
- A flat-file, comma-separated log format (`dynamic_trajectories.txt`) is used as the interchange format between the simulator and the plotter, matching the fixed 11-column schema (`time, id, posx, posz, velx, velz, linx, linz, orientation, steer_code, collided`) shared across the course's assignments so behaviors can be swapped without touching the plotting tool.

## 4. Process

Based on the code's structure and comments, the build followed a layered, assignment-by-assignment progression rather than being written from scratch as one program:

1. **Reuse the previous assignment's primitives.** The `steering_seek` function is explicitly commented `# reuse P1`, meaning the `Seek` steering behavior (and the behavior-code convention `SEEK = 6`) was carried over from an earlier "Program 1" that presumably implemented `Seek`, `Flee`, and `Arrive` (all of which still appear as labeled cases in the plotting script, even though only `Seek` and `Follow Path` are used here). This let the new assignment focus purely on the path-following logic rather than re-deriving basic steering.
2. **Build the path abstraction first.** The `Path` class was written to answer two questions independently: "where is the point at parameter p?" (`get_position`) and "what parameter is closest to this position?" (`get_param`). Structuring these as inverse operations around a shared `param`/`cumdist` table is what makes the "chase the rabbit" trick possible — the character never needs to know which segment it's on, only its scalar progress along the whole path.
3. **Compose `Follow Path` out of `Seek`.** `steering_follow_path` is a thin function that just calls `get_param`, adds the offset, calls `get_position`, and delegates to `steering_seek` — showing the intended design is a small stack of composable behaviors rather than a monolithic controller.
4. **Wire up the fixed-timestep simulation loop.** `main()` follows the same two-phase pattern (compute all steering outputs, then integrate all characters) that a multi-agent version of this simulation would need, even though only one character (`id 2701`) is instantiated here — a sign the loop was written to generalize beyond this single scenario.
5. **Match the existing output contract.** `write_record` reproduces the exact 11-field CSV schema (including quirky formatting like the explicit negative-zero cleanup on `lin_acc[1]`) used by earlier assignments, so the already-existing plotter script could be reused unmodified against the new behavior's output.
6. **Validate visually.** Running `WardCS330Program2.py` produces `dynamic_trajectories.txt`; running `CS_330_Python_Plotter_withPath_v3_1.py` against that file overlays the character's traced path, velocity, and acceleration vectors on the original 8-point path, which is how the included `TrajectoryPlot.png` was produced and how mismatches between intended and actual path-following (e.g. corner-cutting) would have been caught.

## 5. Outcome

Running the simulation for the full 125 seconds (250 fixed 0.5 s steps) produces a character that starts at `(20, 95)`, well off the path's first vertex `(0, 90)`, and by `t = 125 s` has traveled almost the entire ~519-unit length of the 8-vertex, 7-segment path, ending at `(28.85, -79.18)` — close to but not quite at the final vertex `(0, -85)`, since a fixed 4%-of-path-length look-ahead offset causes the character to visibly cut corners at each sharp vertex rather than tracing them exactly, a real and observable artifact of the "chase the rabbit" approach that the generated `TrajectoryPlot.png` makes visible.

Working through this demonstrates:

- **Translating a steering-behavior algorithm into working numerical code.** Going from the textbook description of "project onto the path, look ahead, seek that point" to actual arc-length parameterization, segment projection, and clamped interpolation required getting several small pieces of vector math (projection, normalization, distance) exactly right for the whole system to converge onto the path instead of orbiting or diverging from it.
- **Designing for composability.** Structuring `Follow Path` as a wrapper around a reusable `Seek` behavior, and keeping the output format identical to a prior assignment, reflects an understanding that steering behaviors and simulation infrastructure are meant to be built incrementally and reused, not rewritten per assignment.
- **Debugging through visualization rather than assertions.** Because there's no closed-form "correct" trajectory to assert against, correctness had to be judged qualitatively — by plotting position, velocity, and acceleration vectors together and checking that the character actually tracks the path shape — which is a realistic taste of how motion/AI systems in games are validated in practice.
- **Recognizing the limits of a simple algorithm.** The visible corner-cutting in the output is a genuine, well-known limitation of fixed-offset path following on piecewise-linear paths, and noticing it (rather than assuming the character perfectly traces the path) is itself evidence of understanding what the algorithm does and doesn't guarantee.
