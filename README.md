# 🧬 IRIN O2 — Evolved Neural Controller (UPM, 2019)

[![Python](https://img.shields.io/badge/Python-3.9%2B-3776AB?style=flat-square)](https://www.python.org/)
[![Tests](https://img.shields.io/badge/tests-9%2F9-22C55E?style=flat-square)](tests/)
[![Play](https://img.shields.io/badge/▶%20Play-Interactive%20Web%20Edition-8B5CF6?style=flat-square)](https://alejp1998.github.io/irin_o2/)

> **▶️ Play it live:** <https://alejp1998.github.io/irin_o2/> — evolve your own e-puck brain in the browser.

*Trabajo Obligatorio 2 de IRIN* (Inteligencia Robótica, UPM, 2019): an e-puck
robot in Webots whose **neural controller is evolved by a genetic algorithm**
instead of being hand-programmed. Two controller variants were developed —
**ANN** (`nndistributedcontroller.cpp`) and **CTRNN**
(`ctrnndistributedcontroller.cpp`) — with a distributed layer architecture,
fitness evaluation per episode, and hyper-parameter files in `paramFiles/`.

## 🧮 The fitness (Nolfi–Floreano)

From *Evolutionary Robotics* (Nolfi & Floreano), computed every step in
`irifitnessfunction.cpp`:

**f = V · (1 − √Δv) · (1 − i)**

| term | meaning |
|---|---|
| `V` | wheel speed — drive fast |
| `Δv` | left/right speed difference — drive straight |
| `i` | max proximity sensor — avoid the walls |

## 🧪 Extracted, testable model

`ga_nn.py` mirrors the Webots stack in pure Python:

- **Neural controller** — MLP `8 proximity rays → 4 hidden → 2 wheels` (tanh)
- **Arena simulator** — 2D e-puck-lite with ray-cast proximity sensors
- **GA** — tournament selection, uniform crossover, gaussian mutation, elitism

```bash
pip install -e ".[dev]"
pytest -q      # 9 tests: sensors, dynamics, fitness ordering, GA improvement
```

The JavaScript port (`webgame/js/ga-core.js`, 8 tests) drives the interactive
page with identical math.

## 🎮 Interactive web edition

`webgame/` — watch evolution happen live:

- **▶ Evolve** — the GA runs generation by generation; the best robot so far
  drives in the arena with its proximity rays visible, and the best-fitness
  curve climbs in real time
- sliders for **population size**, **mutation σ** and **generations/second**
- the fitness formula, controller topology and GA operators are documented in
  the guide modal

## 📁 Layout

```
controllers/    ANN + CTRNN distributed controllers
experiments/    test experiment (GA driver)
fitnessfunctions/ irifitnessfunction.cpp (Nolfi-Floreano)
objects/        light objects for the arena
paramFiles/     ANN/CTRNN hyper-parameters
ga_nn.py + tests/  extracted, unit-tested model
webgame/        interactive neuroevolution lab
```
