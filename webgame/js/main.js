/**
 * IRIN O2 — neuroevolution cockpit: GA evolves an e-puck neural controller live.
 */
(function () {
  "use strict";

  var G = window.GACore;
  var canvas = document.getElementById("view");
  var ctx = canvas.getContext("2d");
  var $id = function (id) {
    return document.getElementById(id);
  };

  var NI = 8,
    NH = 4;
  var SIZE = G.genomeSize(NI, NH);

  var PAL = {
    dark: {
      bg: "#0b0f19",
      floor: "#101a2c",
      wall: "#334155",
      robot: "#22d3ee",
      robotEdge: "#0e7490",
      ray: "rgba(34,211,238,0.35)",
      text: "#e2e8f0",
      muted: "#94a3b8",
      chart: "#22d3ee",
      chartGrid: "rgba(148,163,184,0.15)",
      ok: "#34d399",
      bad: "#f87171",
      panel: "#111c30",
      border: "rgba(255,255,255,0.1)",
    },
    light: {
      bg: "#f1f5f9",
      floor: "#ffffff",
      wall: "#475569",
      robot: "#0891b2",
      robotEdge: "#155e75",
      ray: "rgba(8,145,178,0.3)",
      text: "#0f172a",
      muted: "#475569",
      chart: "#0891b2",
      chartGrid: "rgba(51,65,85,0.12)",
      ok: "#059669",
      bad: "#dc2626",
      panel: "#ffffff",
      border: "rgba(15,23,42,0.12)",
    },
  };
  function pal() {
    var t = document.documentElement.getAttribute("data-theme");
    return PAL[t === "dark" ? "dark" : "light"];
  }

  // ---------------------------------------------------------------- state
  var arena = G.makeArena(1, 1, null, 7);
  var pop = [];
  var fits = [];
  var gen = 0;
  var bestFit = -1;
  var history = [];
  var running = false;
  var speed = 4;
  var rng = G.mulberry32(Math.floor(Math.random() * 1e9));
  var ctrl = null; // best-so-far controller for the live demo robot
  var bestGenome = null;

  function log(msg) {
    var box = $id("log");
    var div = document.createElement("div");
    div.textContent = "› " + msg;
    box.appendChild(div);
    while (box.children.length > 60) box.removeChild(box.firstChild);
    box.scrollTop = box.scrollHeight;
  }

  function initRun() {
    var n = Number($id("pop").value);
    rng = G.mulberry32(Math.floor(Math.random() * 1e9));
    pop = [];
    for (var i = 0; i < n; i++) pop.push(G.randomGenome(SIZE, rng, 1.5));
    fits = new Array(n).fill(0);
    gen = 0;
    bestFit = -1;
    history = [];
    bestGenome = null;
    ctrl = null;
    $id("hud-gen").textContent = "0";
    $id("hud-best").textContent = "—";
    $id("hud-result").className = "hud-result hidden";
    log(
      "🧬 New run: population " + n + ", genome " + SIZE + " weights (8→4→2).",
    );
  }

  function oneGeneration() {
    var n = pop.length;
    for (var i = 0; i < n; i++) {
      fits[i] = G.evaluate(pop[i], arena, 120, NI, NH);
    }
    var bestIdx = 0;
    for (var j = 1; j < n; j++) if (fits[j] > fits[bestIdx]) bestIdx = j;
    if (fits[bestIdx] > bestFit) {
      bestFit = fits[bestIdx];
      bestGenome = pop[bestIdx].slice();
      ctrl = G.makeController(NI, NH, bestGenome);
      if (
        bestFit > 0.45 &&
        history.length &&
        bestFit > history[history.length - 1] + 0.05
      ) {
        log("🔥 New best: " + bestFit.toFixed(3) + " (gen " + gen + ")");
      }
    }
    history.push(bestFit);
    pop = G.evolve(pop, fits, rng, {
      elitism: 2,
      k: 3,
      rate: 0.15,
      sigma: Number($id("sigma").value),
    });
    gen++;
    $id("hud-gen").textContent = String(gen);
    $id("hud-best").textContent = bestFit.toFixed(3);
    if (bestFit > 0.6 && gen > 40 && running) {
      running = false;
      log(
        "🏁 Converged! Best fitness " +
          bestFit.toFixed(3) +
          " after " +
          gen +
          " generations.",
      );
      setResult(
        "🧠 Brain evolved!",
        "Best fitness " + bestFit.toFixed(3) + " · gen " + gen,
        "win",
      );
      $id("btn-run").textContent = "▶ Evolve";
    }
  }

  function setResult(title, sub, cls) {
    var r = $id("hud-result");
    r.className = "hud-result " + cls;
    r.innerHTML =
      '<div class="hud-result-title">' +
      title +
      "</div>" +
      '<div class="hud-result-sub">' +
      sub +
      "</div>";
  }

  // ---------------------------------------------------------------- render
  function sizeCanvas() {
    var panel = $id("stage-panel");
    var w = panel.clientWidth;
    var h = panel.clientHeight;
    var chartEl = document.getElementById("fit-chart");
    if (chartEl) {
      h -= chartEl.offsetHeight + 14; // chart box + its margin
    }
    var dpr = Math.max(1, window.devicePixelRatio || 1);
    canvas.width = Math.floor(w * dpr);
    canvas.height = Math.floor(h * dpr);
    canvas.style.width = w + "px";
    canvas.style.height = h + "px";
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function render() {
    var p = pal();
    var w = canvas.width / Math.max(1, window.devicePixelRatio || 1);
    var h = canvas.height / Math.max(1, window.devicePixelRatio || 1);
    ctx.fillStyle = p.bg;
    ctx.fillRect(0, 0, w, h);

    var aw = w - 40;
    var ah = h - 64;
    var ax = 20;
    var ay = 16;

    // arena
    ctx.fillStyle = p.floor;
    ctx.strokeStyle = p.wall;
    ctx.lineWidth = 3;
    ctx.fillRect(ax, ay, aw, ah);
    ctx.strokeRect(ax, ay, aw, ah);

    // best robot (demo run with the current best brain)
    if (ctrl) {
      // run a few steps for animation
      var sens = arena.sensors(NI);
      var out = ctrl.forward(sens);
      arena.step(out[0], out[1]);
      var rx = ax + arena.x * aw;
      var ry = ay + ah - arena.y * ah;
      // proximity rays
      for (var i = 0; i < NI; i++) {
        var a = arena.angle + (2 * Math.PI * i) / NI;
        var d = sens[i] * 0.25;
        ctx.strokeStyle = p.ray;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(rx, ry);
        ctx.lineTo(
          rx + Math.cos(a) * d * aw * 0.9,
          ry - Math.sin(a) * d * ah * 0.9,
        );
        ctx.stroke();
      }
      // robot body + facing tick
      ctx.fillStyle = p.robot;
      ctx.strokeStyle = p.robotEdge;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(rx, ry, 10, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(
        rx + Math.cos(arena.angle) * 10,
        ry - Math.sin(arena.angle) * 10,
      );
      ctx.stroke();
    }

    ctx.fillStyle = p.muted;
    ctx.font = "600 12px system-ui";
    ctx.textAlign = "left";
    ctx.fillText("arena — the best evolved controller so far", 16, 12);
  }

  function wire() {
    $id("btn-run").addEventListener("click", function () {
      running = !running;
      this.textContent = running ? "⏸ Pause" : "▶ Evolve";
      if (running && history.length === 0) log("▶ Evolution started.");
    });
    $id("btn-pause").addEventListener("click", function () {
      running = false;
      $id("btn-run").textContent = "▶ Evolve";
    });
    $id("btn-restart").addEventListener("click", function () {
      running = false;
      $id("btn-run").textContent = "▶ Evolve";
      initRun();
    });
    ["pop", "sigma", "speed"].forEach(function (id) {
      $id(id).addEventListener("input", function () {
        var suffix = id === "speed" ? "×" : "";
        $id(id + "-v").textContent = this.value + suffix;
        if (id === "pop") {
          $id("hud-pop").textContent = this.value;
        }
        if (id === "speed") speed = Number(this.value);
      });
    });
    $id("btn-theme").addEventListener("click", function () {
      var t =
        document.documentElement.getAttribute("data-theme") === "dark"
          ? "light"
          : "dark";
      document.documentElement.setAttribute("data-theme", t);
      try {
        localStorage.setItem("theme", t);
      } catch (e) {}
      applyTheme();
    });
    window
      .matchMedia("(prefers-color-scheme: dark)")
      .addEventListener("change", function (ev) {
        if (localStorage.getItem("theme")) return;
        document.documentElement.setAttribute(
          "data-theme",
          ev.matches ? "dark" : "light",
        );
        applyTheme();
      });
    window.addEventListener("resize", function () {
      sizeCanvas();
    });
  }

  function applyTheme() {
    var t = document.documentElement.getAttribute("data-theme");
    $id("btn-theme").textContent = t === "dark" ? "☀️" : "🌙";
  }

  var guideOpen = false;
  function wireGuide() {
    var guide = $id("guide");
    function open() {
      guideOpen = true;
      guide.classList.remove("hidden");
    }
    function close() {
      guideOpen = false;
      guide.classList.add("hidden");
    }
    $id("btn-guide").addEventListener("click", open);
    guide.querySelectorAll("[data-close-guide]").forEach(function (el) {
      el.addEventListener("click", close);
    });
    document.addEventListener("keydown", function (e) {
      if (e.code === "Escape" && guideOpen) close();
    });
  }

  // ---------------------------------------------------------------- loop
  // D3 fitness chart (own component, SVG — not canvas/Pixi)
  var fitChart = null;
  var lastChartT = 0;
  function initChart() {
    var el = document.getElementById("fit-chart");
    if (!el || !window.MiniChart) return;
    fitChart = window.MiniChart(el, {
      height: 148,
      title: "best fitness per generation · f = V·(1−√Δv)·(1−i)",
      emptyText: "press ▶ Evolve to start",
      pad: 0.02,
      getData: function () {
        return history.slice();
      },
      color: function () {
        return pal().chart;
      },
    });
  }

  function init() {
    initChart();
    applyTheme();
    wireGuide();
    wire();
    sizeCanvas();
    initRun();
    log(
      "📚 IRIN O2: NN controller + GA (tournament, crossover, mutation, elitism).",
    );
    log("🎯 Fitness: drive fast & straight while avoiding the walls.");

    function loop() {
      if (running) {
        for (var i = 0; i < speed; i++) oneGeneration();
      }
      if (fitChart && Date.now() - lastChartT > 250) {
        fitChart.update();
        lastChartT = Date.now();
      }
      render();
      requestAnimationFrame(loop);
    }
    requestAnimationFrame(loop);
  }

  init();
})();
