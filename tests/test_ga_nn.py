"""Tests for the IRIN O2 GA + neural controller (ga_nn.py)."""

import random

import ga_nn


def test_genome_size_and_forward():
    ctrl = ga_nn.NeuralController(8, 4, [0.5] * ga_nn.NeuralController.genome_size(8, 4))
    out = ctrl.forward([0.1] * 8)
    assert len(out) == 2
    assert all(-1 <= v <= 1 for v in out)


def test_zero_genome_gives_zero_outputs():
    ctrl = ga_nn.NeuralController(8, 4, [0.0] * ga_nn.NeuralController.genome_size(8, 4))
    out = ctrl.forward([0.5, 0.2, 0, 0, 0, 0, 0, 0])
    assert all(abs(v) < 1e-9 for v in out)


def test_arena_sensors_close_to_wall():
    import math

    a = ga_nn.Arena(seed=3)
    a.x, a.y, a.angle = 0.03, 0.5, math.pi  # facing the left wall (x=0)
    sens = a.sensors()
    assert sens[0] > 0.8  # ray 0 points left -> wall very close


def test_arena_step_clamps_inside():
    a = ga_nn.Arena(seed=5)
    a.x, a.y = 0.01, 0.01
    a.angle = math_angle_neg()
    for _ in range(500):
        a.step(1.0, 1.0)  # full forward
        assert 0.02 <= a.x <= a.w - 0.02
        assert 0.02 <= a.y <= a.h - 0.02


def math_angle_neg():
    import math
    return math.pi * 0.25  # toward bottom-right corner


def test_fitness_step_formula():
    a = ga_nn.Arena(seed=7)
    a.x, a.y = 0.5, 0.5
    a.angle = 0.0
    # moving straight and fast away from walls scores higher than stalled
    f_straight = a.fitness_step(1.0, 1.0)
    f_stalled = a.fitness_step(0.0, 0.0)
    assert f_straight > f_stalled
    # hitting a wall (sensor high) lowers the score
    a.x = 0.03
    a.angle = 0.0
    f_wall = a.fitness_step(1.0, 1.0)
    assert f_wall < f_straight


def test_tournament_picks_best():
    rng = random.Random(1)
    pop = [[0], [1], [2], [3]]
    fits = [0.1, 0.5, 0.9, 0.2]
    # with k=4 the best is always in the tournament
    winner = ga_nn.tournament(pop, fits, 4, rng)
    assert winner == [2]


def test_crossover_and_mutation():
    rng = random.Random(2)
    a = [0.0] * 20
    b = [1.0] * 20
    child = ga_nn.crossover(a, b, rng)
    assert set(child) <= {0.0, 1.0}
    mutated = ga_nn.mutate(child, rate=1.0, sigma=0.1, rng=rng)
    assert mutated != child


def test_evolution_improves_best_fitness():
    """Evolve a tiny population on a toy task: genome value -> fitness = value."""
    rng = random.Random(42)
    pop = [ga_nn.random_genome(4, rng) for _ in range(12)]

    def fits_of(p):
        return [sum(g) for g in p]

    best0 = max(fits_of(pop))
    for _ in range(40):
        fits = fits_of(pop)
        pop = ga_nn.evolve(pop, fits, rng, elitism=2, k=3, rate=0.3, sigma=0.05)
    best1 = max(fits_of(pop))
    assert best1 > best0


def test_ga_evolves_robot_to_drive_straight():
    """End-to-end (fast): 15 generations of 10 robots on the arena improve fitness."""
    rng = random.Random(7)
    size = ga_nn.NeuralController.genome_size(8, 4)
    pop = [ga_nn.random_genome(size, rng, scale=1.5) for _ in range(10)]
    arena = ga_nn.Arena(seed=7)

    fits = [ga_nn.evaluate(g, arena, steps=60) for g in pop]
    best0 = max(fits)
    for _ in range(15):
        fits = [ga_nn.evaluate(g, arena, steps=60) for g in pop]
        pop = ga_nn.evolve(pop, fits, rng, elitism=1, k=2, rate=0.2, sigma=0.3)
    best1 = max(fits)
    assert best1 > best0
