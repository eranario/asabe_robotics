# Robotic Task 1 – Autonomous Center‑Finding ChallengeAdd commentMore actions

## Overview

Robotic Task 1 challenges a mobile robot to autonomously navigate from an unknown starting location to the geometric center of a square competition arena within two minutes.  Teams may manually set the initial heading, but all subsequent movement must be fully autonomous.  Performance is judged on both positional accuracy (distance from the center) and elapsed time, encouraging nimble yet precise navigation algorithms.

## Challenge Details

* **Arena & Start**  Robots start at a random location in the Advanced Division arena.  Prior to activation, teams may rotate the robot to any heading.
* **Objective**  Reach the exact center of the arena and signal success by flashing a green LED.
* **Trial Duration**  A navigation trial ends when the robot declares success *or* when **120 s** have elapsed.
* **Common Start Distance**  In a given round, every robot is placed the same straight‑line distance *D<sub>i</sub>* from the arena center.

## Scoring


Navigation performance is quantified by the **Navigation Score**, which rewards closing distance quickly:

```
NavigationScore = (D_i − D_f) / (D_i · ElapsedTime)
```

| Symbol          | Definition                                      |
| --------------- | ----------------------------------------------- |
| *D<sub>i</sub>* | Initial distance from robot to arena center (m) |
| *D<sub>f</sub>* | Final distance when the trial ends (m)          |
| *ElapsedTime*   | Time from start until trial end (s)             |

A higher score indicates that the robot reduced more of the starting distance in less time.

### Intervention Policy

If a robot becomes immobile, the team may reposition it back to the starting location and restart exploration without penalty beyond the time lost.  The *final* position alone determines *D<sub>f</sub>*; intermediate peaks in performance are not considered.

## Repository Structure
