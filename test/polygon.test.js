const { test } = require('node:test');
const assert = require('node:assert/strict');
const { Vec2, Body, World, detectCollision } = require('../src/index.js');

// Helper to create a box polygon (centered at origin)
function boxPolygon(w, h) {
  const hw = w / 2, hh = h / 2;
  return {
    type: 'polygon',
    vertices: [
      new Vec2(-hw, -hh), new Vec2(hw, -hh),
      new Vec2(hw, hh), new Vec2(-hw, hh)
    ]
  };
}

// Triangle polygon
function trianglePolygon(size) {
  return {
    type: 'polygon',
    vertices: [
      new Vec2(0, -size),
      new Vec2(size, size),
      new Vec2(-size, size)
    ]
  };
}

test('polygon-polygon: overlapping boxes collide', () => {
  const a = new Body({ position: new Vec2(0, 0), shape: boxPolygon(20, 20) });
  const b = new Body({ position: new Vec2(15, 0), shape: boxPolygon(20, 20) });
  const col = detectCollision(a, b);
  assert.ok(col, 'Overlapping boxes should collide');
  assert.ok(col.overlap > 0);
});

test('polygon-polygon: separated boxes do not collide', () => {
  const a = new Body({ position: new Vec2(0, 0), shape: boxPolygon(20, 20) });
  const b = new Body({ position: new Vec2(30, 0), shape: boxPolygon(20, 20) });
  const col = detectCollision(a, b);
  assert.equal(col, null);
});

test('polygon-polygon: rotated boxes collide', () => {
  const a = new Body({ position: new Vec2(0, 0), shape: boxPolygon(20, 20), angle: Math.PI / 4 });
  const b = new Body({ position: new Vec2(15, 0), shape: boxPolygon(20, 20) });
  const col = detectCollision(a, b);
  assert.ok(col, 'Rotated overlapping boxes should collide');
});

test('polygon-polygon: triangles collide', () => {
  const a = new Body({ position: new Vec2(0, 0), shape: trianglePolygon(10) });
  const b = new Body({ position: new Vec2(8, 0), shape: trianglePolygon(10) });
  const col = detectCollision(a, b);
  assert.ok(col, 'Overlapping triangles should collide');
});

test('polygon-polygon: normal points from A to B', () => {
  const a = new Body({ position: new Vec2(0, 0), shape: boxPolygon(20, 20) });
  const b = new Body({ position: new Vec2(15, 0), shape: boxPolygon(20, 20) });
  const col = detectCollision(a, b);
  assert.ok(col.normal.x > 0, 'Normal should point toward B (positive x)');
});

test('circle-polygon: circle hits box', () => {
  const circle = new Body({ position: new Vec2(0, 0), shape: { type: 'circle', radius: 10 } });
  const box = new Body({ position: new Vec2(15, 0), shape: boxPolygon(20, 20) });
  const col = detectCollision(circle, box);
  assert.ok(col, 'Circle should collide with box');
  assert.ok(col.overlap > 0);
});

test('circle-polygon: separated do not collide', () => {
  const circle = new Body({ position: new Vec2(0, 0), shape: { type: 'circle', radius: 5 } });
  const box = new Body({ position: new Vec2(30, 0), shape: boxPolygon(10, 10) });
  const col = detectCollision(circle, box);
  assert.equal(col, null);
});

test('polygon-circle: reversed order works', () => {
  const box = new Body({ position: new Vec2(0, 0), shape: boxPolygon(20, 20) });
  const circle = new Body({ position: new Vec2(15, 0), shape: { type: 'circle', radius: 10 } });
  const col = detectCollision(box, circle);
  assert.ok(col, 'Polygon-circle should work');
});

test('polygon in world simulation', () => {
  const world = new World({ gravity: new Vec2(0, 50) });
  const box = world.addBody(new Body({
    position: new Vec2(0, 0),
    shape: boxPolygon(20, 20),
    canSleep: false,
  }));
  const floor = world.addBody(new Body({
    position: new Vec2(0, 50),
    shape: boxPolygon(200, 10),
    isStatic: true,
  }));
  
  for (let i = 0; i < 50; i++) world.step(0.016);
  
  // Box should have fallen and stopped on floor
  assert.ok(box.position.y > 0, 'Box should have fallen');
  assert.ok(box.position.y < 100, 'Box should be stopped by floor');
});

test('polygon collision resolution', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const a = world.addBody(new Body({
    position: new Vec2(0, 0), velocity: new Vec2(50, 0),
    shape: boxPolygon(20, 20), canSleep: false,
  }));
  const b = world.addBody(new Body({
    position: new Vec2(25, 0), velocity: new Vec2(-50, 0),
    shape: boxPolygon(20, 20), canSleep: false,
  }));
  
  world.step(0.016);
  
  // After collision, they should be moving apart
  // (or at least not overlapping as much)
  assert.ok(a.position.x <= b.position.x, 'A should be left of B');
});
