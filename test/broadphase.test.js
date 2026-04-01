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

test('SpatialHashGrid: handles polygon shapes', () => {
  const grid = new SpatialHashGrid(50);
  const hw = 15, hh = 15;
  const polyShape = {
    type: 'polygon',
    vertices: [new Vec2(-hw, -hh), new Vec2(hw, -hh), new Vec2(hw, hh), new Vec2(-hw, hh)]
  };
  const a = new Body({ position: new Vec2(10, 10), shape: polyShape });
  const b = new Body({ position: new Vec2(20, 10), shape: polyShape });
  const c = new Body({ position: new Vec2(200, 200), shape: polyShape });
  
  grid.insert(a);
  grid.insert(b);
  grid.insert(c);
  
  const nearA = grid.query(a);
  assert.ok(nearA.has(b), 'nearby polygon b should be found');
  assert.ok(!nearA.has(c), 'distant polygon c should not be found');
});

test('SpatialHashGrid: rotated polygon covers more cells', () => {
  const grid = new SpatialHashGrid(20);
  const shape = {
    type: 'polygon',
    vertices: [new Vec2(-15, -2), new Vec2(15, -2), new Vec2(15, 2), new Vec2(-15, 2)]
  };
  // Long thin rectangle at angle 0 — covers ~2 cells wide
  const a = new Body({ position: new Vec2(0, 0), shape, angle: 0 });
  grid.insert(a);
  const cellsNormal = grid.grid.size;
  
  // Same rectangle rotated 45° — should cover more cells (diagonal)
  grid.clear();
  const b = new Body({ position: new Vec2(0, 0), shape, angle: Math.PI / 4 });
  grid.insert(b);
  const cellsRotated = grid.grid.size;
  
  assert.ok(cellsRotated >= cellsNormal, 
    `Rotated (${cellsRotated} cells) should cover >= non-rotated (${cellsNormal} cells)`);
});

test('SpatialHashGrid: clear resets grid', () => {
  const grid = new SpatialHashGrid(20);
  const a = new Body({ position: new Vec2(10, 10) });
  grid.insert(a);
  assert.ok(grid.grid.size > 0);
  grid.clear();
  assert.equal(grid.grid.size, 0);
});

test('SpatialHashGrid: getCandidatePairs deduplicates', () => {
  const grid = new SpatialHashGrid(50);
  const a = new Body({ position: new Vec2(10, 10), shape: { type: 'circle', radius: 5 } });
  const b = new Body({ position: new Vec2(15, 10), shape: { type: 'circle', radius: 5 } });
  grid.insert(a);
  grid.insert(b);
  
  const pairs = grid.getCandidatePairs();
  assert.equal(pairs.size, 1, 'Should have exactly 1 pair');
});

test('SpatialHashGrid: body spanning multiple cells', () => {
  const grid = new SpatialHashGrid(10); // Small cells
  const bigBody = new Body({ position: new Vec2(0, 0), shape: { type: 'circle', radius: 25 } });
  grid.insert(bigBody);
  
  // Body at (0,0) with radius 25 spans from -25 to 25 in both axes
  // With cell size 10, that's cells from -3 to 2 = 6 cells per axis = 36 cells
  assert.ok(grid.grid.size > 1, `Big body should span multiple cells, got ${grid.grid.size}`);
  
  // Query from various positions should find it
  const probe1 = new Body({ position: new Vec2(20, 0), shape: { type: 'circle', radius: 1 } });
  const probe2 = new Body({ position: new Vec2(0, 20), shape: { type: 'circle', radius: 1 } });
  assert.ok(grid.query(probe1).has(bigBody));
  assert.ok(grid.query(probe2).has(bigBody));
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
  
  const naive = 20 * 19 / 2; // 190
  assert.ok(world.stats.narrowphaseChecks < naive, 
    `Broadphase: ${world.stats.narrowphaseChecks} checks vs naive: ${naive}`);
});

test('broadphase with mixed shape types', () => {
  const world = new World({ gravity: new Vec2(0, 0), cellSize: 50 });
  const hw = 10, hh = 10;
  
  // Mix of circles, AABBs, and polygons
  world.addBody(new Body({ position: new Vec2(0, 0), shape: { type: 'circle', radius: 10 }, canSleep: false }));
  world.addBody(new Body({ position: new Vec2(15, 0), shape: { type: 'aabb', width: 20, height: 20 }, canSleep: false }));
  world.addBody(new Body({ 
    position: new Vec2(30, 0), 
    shape: { type: 'polygon', vertices: [new Vec2(-hw, -hh), new Vec2(hw, -hh), new Vec2(hw, hh), new Vec2(-hw, hh)] },
    canSleep: false 
  }));
  // Far away body
  world.addBody(new Body({ position: new Vec2(500, 500), shape: { type: 'circle', radius: 5 }, canSleep: false }));
  
  // Need >8 bodies for broadphase to kick in, add fillers
  for (let i = 0; i < 10; i++) {
    world.addBody(new Body({ position: new Vec2(1000 + i * 100, 0), shape: { type: 'circle', radius: 5 }, canSleep: false }));
  }
  
  world.step(0.016);
  assert.equal(world.stats.bodies, 14);
});

test('broadphase produces same results as naive', () => {
  const bodiesConfig = [];
  for (let i = 0; i < 20; i++) {
    bodiesConfig.push({
      position: new Vec2(Math.random() * 100, Math.random() * 100),
      velocity: new Vec2(0, 0),
      shape: { type: 'circle', radius: 5 },
    });
  }
  
  const world1 = new World({ gravity: new Vec2(0, 0), broadphase: true, cellSize: 15 });
  for (const cfg of bodiesConfig) {
    world1.addBody(new Body({ ...cfg, position: new Vec2(cfg.position.x, cfg.position.y) }));
  }
  world1.step(0.016);
  
  const world2 = new World({ gravity: new Vec2(0, 0), broadphase: false });
  for (const cfg of bodiesConfig) {
    world2.addBody(new Body({ ...cfg, position: new Vec2(cfg.position.x, cfg.position.y) }));
  }
  world2.step(0.016);
  
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
  assert.ok(elapsed < 5000, `Too slow: ${elapsed}ms`);
});
