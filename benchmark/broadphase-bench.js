// benchmark-broadphase.js — Compare spatial hash grid vs naive N² collision detection
const { Body, World, Vec2 } = require('../src/index.js');

function runBenchmark(label, bodyCount, steps, useBroadphase) {
  const world = new World({ broadphase: useBroadphase });
  world.gravity = new Vec2(0, 0);

  for (let i = 0; i < bodyCount; i++) {
    const isCircle = Math.random() > 0.5;
    const shape = isCircle
      ? { type: 'circle', radius: 5 + Math.random() * 10 }
      : { type: 'aabb', width: 10 + Math.random() * 15, height: 10 + Math.random() * 15 };
    const body = new Body({
      x: Math.random() * 1000,
      y: Math.random() * 1000,
      shape,
      mass: 1
    });
    body.vx = (Math.random() - 0.5) * 50;
    body.vy = (Math.random() - 0.5) * 50;
    world.addBody(body);
  }

  const start = performance.now();
  for (let s = 0; s < steps; s++) {
    world.step(1 / 60);
  }
  const elapsed = performance.now() - start;
  const stats = world.stats;

  return {
    label,
    bodyCount,
    steps,
    elapsed: Math.round(elapsed),
    msPerStep: (elapsed / steps).toFixed(2),
    broadphaseChecks: stats.broadphaseChecks,
    collisions: stats.collisionCount
  };
}

console.log('=== Broadphase Performance Benchmark ===\n');

const configs = [
  { bodies: 50, steps: 100 },
  { bodies: 100, steps: 100 },
  { bodies: 200, steps: 50 },
  { bodies: 500, steps: 20 },
];

console.log('Bodies | Steps | Naive (ms) | SpatialHash (ms) | Speedup');
console.log('-------|-------|------------|------------------|--------');

for (const { bodies, steps } of configs) {
  const naive = runBenchmark(`Naive ${bodies}`, bodies, steps, false);
  const spatial = runBenchmark(`Spatial ${bodies}`, bodies, steps, true);
  const speedup = (naive.elapsed / spatial.elapsed).toFixed(2);

  console.log(
    `${String(bodies).padStart(6)} | ${String(steps).padStart(5)} | ` +
    `${String(naive.elapsed).padStart(10)} | ${String(spatial.elapsed).padStart(16)} | ${speedup}x`
  );
}

console.log('\nDetailed stats:');
for (const { bodies, steps } of configs) {
  const naive = runBenchmark(`Naive ${bodies}`, bodies, steps, false);
  const spatial = runBenchmark(`Spatial ${bodies}`, bodies, steps, true);
  console.log(`\n--- ${bodies} bodies, ${steps} steps ---`);
  console.log(`  Naive:   ${naive.elapsed}ms (${naive.msPerStep}ms/step), ${naive.broadphaseChecks} checks`);
  console.log(`  Spatial: ${spatial.elapsed}ms (${spatial.msPerStep}ms/step), ${spatial.broadphaseChecks} checks`);
}
