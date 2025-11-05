# Two-Robot Warehouse Path Planning

This project applies **graph search algorithms** to coordinate two autonomous robots in a **simulated warehouse environment**.  
The robots are tasked with retrieving specific shelves and delivering them to designated packing stations — similar in concept to [**Amazon’s order-picking robots**](https://www.youtube.com/watch?v=Ox05Bks2Q3s)
.

---

## Overview

The environment is a **10×15 discrete grid** containing:

- 16 uniquely identified shelves (`S1–S16`)
- Two robots (`R1`, `R2`)
- Two fixed packing locations (`P`)
- Static obstacles

Each robot can move, pick up, and put down shelves within valid states (no collisions or out-of-bounds movement).

**Goal state:** shelf `S6` and shelf `S10` are placed at the packing station cells.

---

## Search and Heuristic

A **graph search approach** (e.g., A\*) is used to find the optimal sequence of joint actions minimizing total cost.

The **heuristic function** combines:

- Manhattan distances between robots and target shelves  
- Manhattan distances between shelves and packing stations  
- A small additive term for each shelf not yet picked up  

This heuristic is **admissible and consistent**, ensuring optimality.

---

The included video demonstrates the robots executing the computed path.

https://github.com/user-attachments/assets/c3a9c142-700c-4b2e-b9d4-c0b6269d9959

**Grid Legend:**

- R1, R2 = Robots
- S# = Shelf (e.g., S6)
- P = Packing station
- X = Obstacle
- R#/S# = Robot carrying a shelf
- R#↓S# = Robot under a shelf


Each frame represents a single time step in the plan.



