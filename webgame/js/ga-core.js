/**
 * ga-core.js — IRIN O2 web core: neural controller + arena + GA,
 * 1:1 port of ga_nn.py. Pure JS (browser + node:test).
 */
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();
  else root.GACore = factory();
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";

  // ------------------------------------------------------------ controller
  function genomeSize(nInputs, nHidden) {
    return nInputs * nHidden + nHidden + 2 * nHidden + 2;
  }

  function makeController(nInputs, nHidden, weights) {
    var w1 = [];
    var o = 0;
    for (var j = 0; j < nHidden; j++) {
      w1.push(weights.slice(o, o + nInputs));
      o += nInputs;
    }
    var b1 = weights.slice(o, o + nHidden);
    o += nHidden;
    var w2 = [];
    for (var k = 0; k < 2; k++) {
      w2.push(weights.slice(o, o + nHidden));
      o += nHidden;
    }
    var b2 = weights.slice(o, o + 2);
    return {
      forward: function (sensors) {
        var hidden = [];
        for (var j = 0; j < nHidden; j++) {
          var s = b1[j];
          for (var i = 0; i < nInputs; i++) s += w1[j][i] * sensors[i];
          hidden.push(Math.tanh(s));
        }
        var out = [];
        for (var k = 0; k < 2; k++) {
          var s2 = b2[k];
          for (var i2 = 0; i2 < nHidden; i2++) s2 += w2[k][i2] * hidden[i2];
          out.push(Math.tanh(s2));
        }
        return out;
      },
    };
  }

  // --------------------------------------------------------------- arena
  function makeArena(w, h, walls, seed) {
    var rng = mulberry32(seed || 1);
    var state = {
      w: w || 1,
      h: h || 1,
      walls: walls || [
        [0, 0, w, 0],
        [0, h, w, h],
        [0, 0, 0, h],
        [w, 0, w, h],
      ],
      maxSpeed: 0.05,
      x: 0.5,
      y: 0.5,
      angle: 0,
    };
    state.reset = function () {
      state.x = rng() * 0.6 + 0.2;
      state.y = rng() * 0.6 + 0.2;
      state.angle = rng() * Math.PI * 2;
    };
    state.reset();

    function rayHit(ox, oy, dx, dy, maxD) {
      var best = maxD;
      for (var i = 0; i < state.walls.length; i++) {
        var wl = state.walls[i];
        var x1 = wl[0],
          y1 = wl[1],
          x2 = wl[2],
          y2 = wl[3];
        var rx = dx,
          ry = dy,
          sx = x2 - x1,
          sy = y2 - y1;
        var denom = rx * sy - ry * sx;
        if (Math.abs(denom) < 1e-12) continue;
        var t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom;
        var u = ((x1 - ox) * ry - (y1 - oy) * rx) / denom;
        if (t >= 0 && u >= 0 && u <= 1 && t < best) best = t;
      }
      return Math.max(0, 1 - best / maxD);
    }

    state.sensors = function (nRays) {
      nRays = nRays || 8;
      var vals = [];
      for (var i = 0; i < nRays; i++) {
        var a = state.angle + (2 * Math.PI * i) / nRays;
        vals.push(rayHit(state.x, state.y, Math.cos(a), Math.sin(a), 0.25));
      }
      return vals;
    };

    state.step = function (left, right) {
      left = Math.max(-1, Math.min(1, left));
      right = Math.max(-1, Math.min(1, right));
      var v = (state.maxSpeed * (left + right)) / 2;
      var omega = state.maxSpeed * 2 * (right - left);
      state.angle += omega;
      var nx = state.x + v * Math.cos(state.angle);
      var ny = state.y + v * Math.sin(state.angle);
      state.x = Math.max(0.02, Math.min(state.w - 0.02, nx));
      state.y = Math.max(0.02, Math.min(state.h - 0.02, ny));
    };

    state.fitnessStep = function (left, right) {
      var l = 0.5 + left / 2;
      var r = 0.5 + right / 2;
      var v = Math.max(Math.abs(l), Math.abs(r));
      var sameDir = 1 - Math.sqrt(Math.abs(l - r));
      var i = Math.max.apply(null, state.sensors());
      return v * sameDir * (1 - i);
    };

    return state;
  }

  function evaluate(genome, arena, steps, nInputs, nHidden) {
    steps = steps || 200;
    nInputs = nInputs || 8;
    nHidden = nHidden || 4;
    var ctrl = makeController(nInputs, nHidden, genome);
    arena.reset();
    var total = 0;
    for (var s = 0; s < steps; s++) {
      var sens = arena.sensors();
      var out = ctrl.forward(sens);
      arena.step(out[0], out[1]);
      total += arena.fitnessStep(out[0], out[1]);
    }
    return total / steps;
  }

  // ------------------------------------------------------------------- GA
  function mulberry32(seed) {
    var a = seed >>> 0;
    return function () {
      a |= 0;
      a = (a + 0x6d2b79f5) | 0;
      var t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function randomGenome(size, rng, scale) {
    scale = scale || 1;
    var g = [];
    for (var i = 0; i < size; i++) g.push((rng() * 2 - 1) * scale);
    return g;
  }

  function tournament(pop, fits, k, rng) {
    var best = null,
      bestF = -1e18;
    for (var i = 0; i < k; i++) {
      var idx = Math.floor(rng() * pop.length);
      if (fits[idx] > bestF) {
        bestF = fits[idx];
        best = pop[idx];
      }
    }
    return best;
  }

  function crossover(a, b, rng) {
    var child = [];
    for (var i = 0; i < a.length; i++) child.push(rng() < 0.5 ? a[i] : b[i]);
    return child;
  }

  function mutate(genome, rate, sigma, rng) {
    return genome.map(function (g) {
      return rng() < rate ? g + gauss(rng) * sigma : g;
    });
  }

  function gauss(rng) {
    // Box-Muller
    var u = 0,
      v = 0;
    while (u === 0) u = rng();
    while (v === 0) v = rng();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  function evolve(pop, fits, rng, opts) {
    opts = opts || {};
    var elitism = opts.elitism || 2;
    var k = opts.k || 3;
    var rate = opts.rate || 0.1;
    var sigma = opts.sigma || 0.2;
    var order = fits
      .map(function (f, i) {
        return i;
      })
      .sort(function (a, b) {
        return fits[b] - fits[a];
      });
    var nxt = [];
    for (var e = 0; e < elitism; e++) nxt.push(pop[order[e]].slice());
    while (nxt.length < pop.length) {
      var a = tournament(pop, fits, k, rng);
      var b = tournament(pop, fits, k, rng);
      nxt.push(mutate(crossover(a, b, rng), rate, sigma, rng));
    }
    return nxt;
  }

  return {
    genomeSize: genomeSize,
    makeController: makeController,
    makeArena: makeArena,
    evaluate: evaluate,
    mulberry32: mulberry32,
    randomGenome: randomGenome,
    tournament: tournament,
    crossover: crossover,
    mutate: mutate,
    evolve: evolve,
  };
});
