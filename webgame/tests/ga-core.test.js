/**
 * Node tests for the IRIN O2 web core (matches the Python ga_nn.py tests).
 */
const { test } = require("node:test");
const assert = require("node:assert");

const G = require("../js/ga-core.js");

test("controller forward: 8 inputs -> 2 outputs in [-1, 1]", () => {
  const size = G.genomeSize(8, 4);
  const ctrl = G.makeController(8, 4, new Array(size).fill(0.5));
  const out = ctrl.forward(new Array(8).fill(0.1));
  assert.strictEqual(out.length, 2);
  assert.ok(out.every((v) => v >= -1 && v <= 1));
});

test("zero genome -> zero outputs", () => {
  const size = G.genomeSize(8, 4);
  const ctrl = G.makeController(8, 4, new Array(size).fill(0));
  const out = ctrl.forward([0.5, 0.2, 0, 0, 0, 0, 0, 0]);
  assert.ok(out.every((v) => Math.abs(v) < 1e-9));
});

test("arena sensor fires near a wall", () => {
  const a = G.makeArena(1, 1, null, 3);
  a.x = 0.03;
  a.y = 0.5;
  a.angle = Math.PI; // facing the left wall
  assert.ok(a.sensors()[0] > 0.8);
});

test("arena step clamps inside the box", () => {
  const a = G.makeArena(1, 1, null, 5);
  a.x = 0.01;
  a.y = 0.01;
  a.angle = Math.PI * 0.25;
  for (let i = 0; i < 500; i++) {
    a.step(1, 1);
    assert.ok(a.x >= 0.02 && a.x <= 0.98);
    assert.ok(a.y >= 0.02 && a.y <= 0.98);
  }
});

test("straight fast motion scores higher than stalled", () => {
  const a = G.makeArena(1, 1, null, 7);
  a.x = 0.5;
  a.y = 0.5;
  a.angle = 0;
  const straight = a.fitnessStep(1, 1);
  const stalled = a.fitnessStep(0, 0);
  assert.ok(straight > stalled);
});

test("tournament picks the best with k=size", () => {
  const rng = G.mulberry32(1);
  const pop = [[0], [1], [2], [3]];
  const fits = [0.1, 0.5, 0.9, 0.2];
  assert.deepStrictEqual(G.tournament(pop, fits, 4, rng), [2]);
});

test("evolution improves best fitness on a toy task", () => {
  const rng = G.mulberry32(42);
  let pop = [];
  for (let i = 0; i < 12; i++) pop.push(G.randomGenome(4, rng));
  const fitsOf = (p) => p.map((g) => g.reduce((a, b) => a + b, 0));
  const best0 = Math.max.apply(null, fitsOf(pop));
  for (let gen = 0; gen < 40; gen++) {
    const fits = fitsOf(pop);
    pop = G.evolve(pop, fits, rng, {
      elitism: 2,
      k: 3,
      rate: 0.3,
      sigma: 0.05,
    });
  }
  assert.ok(Math.max.apply(null, fitsOf(pop)) > best0);
});

test("GA evolves the robot to drive better on the arena", () => {
  const rng = G.mulberry32(7);
  const size = G.genomeSize(8, 4);
  let pop = [];
  for (let i = 0; i < 10; i++) pop.push(G.randomGenome(size, rng, 1.5));
  const arena = G.makeArena(1, 1, null, 7);
  let fits = pop.map((g) => G.evaluate(g, arena, 60));
  const best0 = Math.max.apply(null, fits);
  for (let gen = 0; gen < 15; gen++) {
    fits = pop.map((g) => G.evaluate(g, arena, 60));
    pop = G.evolve(pop, fits, rng, { elitism: 1, k: 2, rate: 0.2, sigma: 0.3 });
  }
  assert.ok(Math.max.apply(null, fits) > best0);
});
