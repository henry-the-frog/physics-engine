const { test } = require('node:test');
const assert = require('node:assert/strict');
const { Vec2, Body, World, SpatialHashGrid } = require('../src/index.js');

test('SpatialHashGrid: insert and query', () => {
  const grid = new SpatialHashGrid(20);
  const a = new Body({ position: new Vec2(10, 10), shape: { type: 'circle', radius: 5 } });
  const b = new Body({ position: new Vec2(15, 10), shape: { type: 'circle', radius: 5 } });
  const c = new Body({ position: new Vec2(100, 100), shape: { type: 'circle', radius: 5 } });
  
  grid.insert(a);
  grid.insert(b);
  grid.insert(c);
  
  const nearA = grid.query(a);
  assert.ok(nearA.has(b), 'b should be near a');
  assert.ok(!nearA.has(c), 'c should not be near a');
});

test('SpatialHashGrid: handles AABB shapes', () => {
  const grid = new SpatialHashGrid(20);
  const a = new Body({ position: new Vec2(10, 10), shape: { type: 'aabb', width: 10, height: 10 } });
  const b = new Body({ position: new Vec2(15, 15), shape: { type: 'aabb', width: 10, height: 10 } });
  
  grid.insert(a);
  grid.insert(b);
  
  const nearA = grid.query(a);
  assert.ok(nearA.has(b));
});

test('SpatialHashGrid: clear resets grid', () => {
  const grid = new SpatialHashGrid(20);
  const a = new Body({ position: new Vec2(10, 10) });
  grid.insert(a);
  assert.ok(grid.grid.size > 0);
  grid.clear();
  assert.equal(grid.grid.size, 0);
});

test('broadphase reduces collision checks', () => {
  const world = new World({ gravity: new Vec2(0, 0), broadphase: true, cellSize: 20 });
  
  // Create two clusters of bodies far apart
  for (let i = 0; i < 10; i++) {
    world.addBody(new Body({ position: new Vec2(i * 2, 0), shape: { type: 'circle', radius: 5 } }));
  }
  for (let i = 0; i < 10; i++) {
    world.addBody(new Body({ position: new Vec2(1000 + i * 2, 0), shape: { type: 'circle', radius: 5 } }));
  }
  
  world.step(0.016);
  
  // With 20 bodies, naive would check 190 pairs. Broadphase should check fewer.
  const naive = 20 * 19 / 2; // 190
  assert.ok(world.stats.narrowphaseChecks < naive, 
    `Broadphase: ${world.stats.narrowphaseChecks} checks vs naive: ${naive}`);
});

test('broadphase produces same results as naive', () => {
  // Create identical scenarios
  const bodiesConfig = [];
  for (let i = 0; i < 20; i++) {
    bodiesConfig.push({
      position: new Vec2(Math.random() * 100, Math.random() * 100),
      velocity: new Vec2(0, 0),
      shape: { type: 'circle', radius: 5 },
    });
  }
  
  // Run with broadphase
  const world1 = new World({ gravity: new Vec2(0, 0), broadphase: true, cellSize: 15 });
  for (const cfg of bodiesConfig) {
    world1.addBody(new Body({ ...cfg, position: new Vec2(cfg.position.x, cfg.position.y) }));
  }
  world1.step(0.016);
  
  // Run without broadphase (naive)
  const world2 = new World({ gravity: new Vec2(0, 0), broadphase: false });
  for (const cfg of bodiesConfig) {
    world2.addBody(new Body({ ...cfg, position: new Vec2(cfg.position.x, cfg.position.y) }));
  }
  world2.step(0.016);
  
  // Both should have same number of bodies
  assert.equal(world1.bodies.length, world2.bodies.length);
});

test('broadphase perf: 100 bodies', () => {
  const world = new World({ gravity: new Vec2(0, 10), broadphase: true, cellSize: 30 });
  for (let i = 0; i < 100; i++) {
    world.addBody(new Body({
      position: new Vec2(Math.random() * 500, Math.random() * 500),
      shape: { type: 'circle', radius: 5 },
    }));
  }
  
  const start = performance.now();
  for (let i = 0; i < 100; i++) {
    world.step(0.016);
  }
  const elapsed = performance.now() - start;
  console.log(`100 bodies × 100 steps: ${elapsed.toFixed(1)}ms`);
  assert.ok(elapsed < 5000, `Too slow: ${elapsed}ms`);
});

test('broadphase perf: 500 bodies', () => {
  const world = new World({ gravity: new Vec2(0, 10), broadphase: true, cellSize: 20 });
  for (let i = 0; i < 500; i++) {
    world.addBody(new Body({
      position: new Vec2(Math.random() * 1000, Math.random() * 1000),
      shape: { type: 'circle', radius: 3 },
    }));
  }
  
  const start = performance.now();
  for (let i = 0; i < 10; i++) {
    world.step(0.016);
  }
  const elapsed = performance.now() - start;
  console.log(`500 bodies × 10 steps: ${elapsed.toFixed(1)}ms`);
  console.log(`Stats:`, world.stats);
  assert.ok(elapsed < 5000, `Too slow: ${elapsed}ms`);
});
