# Path-Following — Chase the Rabbit

**Type:** Individual project
**Contributor:** Carter Ward
**Course:** CS 330-1 (Artificial Intelligence / Game AI) — Program 2
**Completed:** 10/13/2025

## Purpose

This is my second AI-for-games assignment for CS 330, following on from the dynamic-movement project (Program 1) where I built `Seek`, `Flee`, and `Arrive`. Program 2 asks how to move an agent smoothly along a series of waypoints using only local, frame-by-frame steering decisions — no global planner or full lookahead. I implemented the "Follow Path" steering behavior and reused the simulation/logging/plotting pipeline from Program 1.

## Problem and Approach

The task was to simulate a character following an 8-vertex path using "Follow Path" (behavior code `11`) and log the trajectory in the same 11-field CSV format used for `Seek`/`Flee`/`Arrive`, so it could be reused with the same plotter. I implemented the classic "chase the rabbit" technique: project the character's position onto the path to get a normalized arc-length parameter `p`, offset `p` slightly forward to get a look-ahead target, convert that back to an `(x, z)` point, and `Seek` it. A Newton–Euler-1 integrator updates position/velocity each step, with speed capping and micro-jitter cleanup.

## Structure and Methodologies

- `Path` class — polyline defined by `x`/`z` lists with precomputed cumulative distances and arc-length parameters, so any point is addressable by a single scalar `p ∈ [0, 1]` (`get_position`/`get_param` as inverses).
- `Character` class — holds position, velocity, orientation, acceleration, caps, and a reference to its `Path` plus look-ahead offset.
- `steering_follow_path` composed as a thin wrapper around the reused `steering_seek` from Program 1.
- Vectors as plain `(x, z)` tuples with small free functions for the needed algebra.
- Dependencies: stdlib `math` only for the simulator; `matplotlib` + `csv` for the plotting script.

## Process

1. Reused Program 1's `Seek` behavior and log format as a base.
2. Built the `Path` abstraction around a shared `param`/`cumdist` table.
3. Hardcoded the assignment's 8-vertex path.
4. Composed `Follow Path` from projection + offset + `Seek`.
5. Ran a fixed-timestep (`DT=0.5s`, 250 steps) simulation loop.
6. Logged every frame to `dynamic_trajectories.txt` in Program 1's format.
7. Plotted the result to `TrajectoryPlot.png` to visually validate it.

## Outcome

Over the 125 s run, the character starts off-path at `(20, 95)` and ends at `(28.85, -79.18)`, close to but not exactly on the final vertex `(0, -85)`. The plotted trajectory hugs the reference path tightly along straight segments but visibly rounds each sharp corner rather than hitting it exactly — an expected effect of always chasing a point 4% ahead rather than the vertex itself. This project taught me to turn a steering description into working arc-length/projection math, to design for composability (reusing `Seek` and the log format), and to validate motion systems visually rather than against a closed-form answer.

**How to run:** `python WardCS330Program2.py` to generate `dynamic_trajectories.txt`, then `python CS_330_Python_Plotter_withPath_v3_1.py` (requires `pip install matplotlib`) to view the plot.
