
"""
ga_nn.py — neural controller evolved by a genetic algorithm, faithful in spirit
to IRIN O2 (UPM, 2019): the e-puck's distributed NN controllers were evolved in
Webots with the Nolfi-Floreano fitness f = V * (1 - sqrt(Delta(v))) * (1 - i).

This module provides:
- a compact MLP controller (sensors -> hidden -> wheel speeds)
- a 2D arena simulator for the e-puck (8 proximity rays, walls)
- the GA (tournament selection, crossover, gaussian mutation, elitism)
All pure Python, deterministic-seedable, testable.
"""

from __future__ import annotations

import math
import random


# ----------------------------------------------------------------------
# Neural controller
# ----------------------------------------------------------------------
class NeuralController:
    """MLP: n_inputs -> n_hidden -> 2 (wheel speeds). Weights in one flat genome."""

    def __init__(self, n_inputs: int, n_hidden: int, weights: list[float] | None = None):
        self.n_inputs = n_inputs
        self.n_hidden = n_hidden
        if weights is None:
            weights = [0.0] * NeuralController.genome_size(n_inputs, n_hidden)
        self.w1 = [weights[i * n_inputs:(i + 1) * n_inputs] for i in range(n_hidden)]
        off = n_inputs * n_hidden
        self.b1 = weights[off:off + n_hidden]
        off += n_hidden
        self.w2 = [weights[off + i * n_hidden:off + (i + 1) * n_hidden] for i in range(2)]
        off += 2 * n_hidden
        self.b2 = weights[off:off + 2]

    @staticmethod
    def genome_size(n_inputs: int, n_hidden: int) -> int:
        return n_inputs * n_hidden + n_hidden + 2 * n_hidden + 2

    def forward(self, sensors: list[float]) -> list[float]:
        hidden = [math.tanh(sum(self.w1[j][i] * sensors[i] for i in range(self.n_inputs)) + self.b1[j])
                  for j in range(self.n_hidden)]
        out = [math.tanh(sum(self.w2[j][k] * hidden[k] for k in range(self.n_hidden)) + self.b2[j])
               for j in range(2)]
        return out


# ----------------------------------------------------------------------
# Arena simulator (e-puck-lite)
# ----------------------------------------------------------------------
class Arena:
    """2D box arena with wall segments; the robot senses proximity rays."""

    def __init__(self, w: float = 1.0, h: float = 1.0, walls: list[tuple] | None = None, seed: int = 1):
        self.w = w
        self.h = h
        self.walls = walls if walls is not None else [(0, 0, w, 0), (0, h, w, h), (0, 0, 0, h), (w, 0, w, h)]
        self.rng = random.Random(seed)
        self.x = self.rng.uniform(0.2, 0.8)
        self.y = self.rng.uniform(0.2, 0.8)
        self.angle = self.rng.uniform(0, 2 * math.pi)
        self.max_speed = 0.05

    def reset(self):
        self.x = self.rng.uniform(0.2, 0.8)
        self.y = self.rng.uniform(0.2, 0.8)
        self.angle = self.rng.uniform(0, 2 * math.pi)

    def _ray_hit(self, ox: float, oy: float, dx: float, dy: float, max_d: float) -> float:
        """Distance to the nearest wall along the ray (normalized 0..1)."""
        best = max_d
        for (x1, y1, x2, y2) in self.walls:
            # segment intersection
            rx, ry = dx, dy
            sx, sy = x2 - x1, y2 - y1
            denom = rx * sy - ry * sx
            if abs(denom) < 1e-12:
                continue
            t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
            u = ((x1 - ox) * ry - (y1 - oy) * rx) / denom
            if t >= 0 and 0 <= u <= 1:
                d = t
                best = min(best, d)
        return max(0.0, 1.0 - best / max_d)  # 1 = very close to a wall

    def sensors(self, n_rays: int = 8) -> list[float]:
        vals = []
        for i in range(n_rays):
            a = self.angle + (2 * math.pi * i / n_rays)
            vals.append(self._ray_hit(self.x, self.y, math.cos(a), math.sin(a), 0.25))
        return vals

    def step(self, left: float, right: float):
        """Differential drive with the given wheel speeds (normalized -1..1)."""
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        v = self.max_speed * (left + right) / 2
        omega = self.max_speed * 2 * (right - left)
        self.angle += omega
        nx = self.x + v * math.cos(self.angle)
        ny = self.y + v * math.sin(self.angle)
        # clamp inside the box
        self.x = max(0.02, min(self.w - 0.02, nx))
        self.y = max(0.02, min(self.h - 0.02, ny))

    def fitness_step(self, left: float, right: float) -> float:
        """Nolfi-Floreano per-step fitness: V * (1 - sqrt(Delta(v))) * (1 - i)."""
        l = 0.5 + left / 2
        r = 0.5 + right / 2
        v = max(abs(l), abs(r))
        same_dir = 1 - math.sqrt(abs(l - r))
        i = max(self.sensors())
        return v * same_dir * (1 - i)


def evaluate(genome: list[float], arena: Arena, steps: int = 200, n_inputs: int = 8,
             n_hidden: int = 4) -> float:
    """Run one episode and return the mean fitness (0 if the robot stalls)."""
    ctrl = NeuralController(n_inputs, n_hidden, genome)
    arena.reset()
    total = 0.0
    for _ in range(steps):
        sens = arena.sensors()
        l, r = ctrl.forward(sens)
        arena.step(l, r)
        total += arena.fitness_step(l, r)
    return total / steps


# ----------------------------------------------------------------------
# Genetic algorithm
# ----------------------------------------------------------------------
def random_genome(size: int, rng: random.Random, scale: float = 1.0) -> list[float]:
    return [rng.uniform(-scale, scale) for _ in range(size)]


def tournament(pop: list[list[float]], fits: list[float], k: int, rng: random.Random) -> list[float]:
    best = None
    best_f = -1e18
    for _ in range(k):
        idx = rng.randrange(len(pop))
        if fits[idx] > best_f:
            best_f = fits[idx]
            best = pop[idx]
    return best


def crossover(a: list[float], b: list[float], rng: random.Random) -> list[float]:
    child = []
    for i in range(len(a)):
        child.append(a[i] if rng.random() < 0.5 else b[i])
    return child


def mutate(genome: list[float], rate: float, sigma: float, rng: random.Random) -> list[float]:
    return [g + rng.gauss(0, sigma) if rng.random() < rate else g for g in genome]


def evolve(population: list[list[float]], fits: list[float], rng: random.Random,
           elitism: int = 2, k: int = 3, rate: float = 0.1, sigma: float = 0.2) -> list[list[float]]:
    """One generation: keep the elites, breed the rest."""
    order = sorted(range(len(fits)), key=lambda i: fits[i], reverse=True)
    nxt = [population[order[i]] for i in range(elitism)]
    while len(nxt) < len(population):
        a = tournament(population, fits, k, rng)
        b = tournament(population, fits, k, rng)
        child = crossover(a, b, rng)
        nxt.append(mutate(child, rate, sigma, rng))
    return nxt
