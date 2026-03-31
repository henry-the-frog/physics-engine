const { test } = require('node:test');
const assert = require('node:assert/strict');
const { Vec2, Body, raycast } = require('../src/index.js');

test('raycast hits circle', () => {
  const bodies = [new Body({ position: new Vec2(10, 0), shape: { type: 'circle', radius: 3 } })];
  const hit = raycast(new Vec2(0, 0), new Vec2(1, 0), bodies);
  assert.ok(hit, 'Should hit circle');
  assert.ok(Math.abs(hit.distance - 7) < 0.1, `Distance: ${hit.distance}`);
  assert.equal(hit.body, bodies[0]);
});

test('raycast misses circle', () => {
  const bodies = [new Body({ position: new Vec2(10, 10), shape: { type: 'circle', radius: 3 } })];
  const hit = raycast(new Vec2(0, 0), new Vec2(1, 0), bodies);
  assert.equal(hit, null);
});

test('raycast respects maxDist', () => {
  const bodies = [new Body({ position: new Vec2(100, 0), shape: { type: 'circle', radius: 5 } })];
  const hit = raycast(new Vec2(0, 0), new Vec2(1, 0), bodies, 50);
  assert.equal(hit, null);
});

test('raycast returns closest hit', () => {
  const bodies = [
    new Body({ position: new Vec2(20, 0), shape: { type: 'circle', radius: 3 } }),
    new Body({ position: new Vec2(10, 0), shape: { type: 'circle', radius: 3 } }),
  ];
  const hit = raycast(new Vec2(0, 0), new Vec2(1, 0), bodies);
  assert.equal(hit.body, bodies[1], 'Should hit closer body');
});

test('raycast hits AABB', () => {
  const bodies = [new Body({ 
    position: new Vec2(10, 0), 
    shape: { type: 'aabb', width: 6, height: 6 } 
  })];
  const hit = raycast(new Vec2(0, 0), new Vec2(1, 0), bodies);
  assert.ok(hit, 'Should hit AABB');
  assert.ok(hit.distance > 0);
});

test('raycast hits polygon', () => {
  const bodies = [new Body({
    position: new Vec2(10, 0),
    shape: {
      type: 'polygon',
      vertices: [new Vec2(-3, -3), new Vec2(3, -3), new Vec2(3, 3), new Vec2(-3, 3)]
    }
  })];
  const hit = raycast(new Vec2(0, 0), new Vec2(1, 0), bodies);
  assert.ok(hit, 'Should hit polygon');
  assert.ok(hit.distance > 0);
});

test('raycast returns hit normal', () => {
  const bodies = [new Body({ position: new Vec2(10, 0), shape: { type: 'circle', radius: 3 } })];
  const hit = raycast(new Vec2(0, 0), new Vec2(1, 0), bodies);
  assert.ok(hit.normal, 'Should have normal');
  // Normal should point back toward the ray origin
  assert.ok(hit.normal.x < 0, `Normal should face ray: nx=${hit.normal.x}`);
});

test('raycast with diagonal direction', () => {
  const bodies = [new Body({ position: new Vec2(10, 10), shape: { type: 'circle', radius: 5 } })];
  const dir = new Vec2(1, 1);
  const hit = raycast(new Vec2(0, 0), dir, bodies);
  assert.ok(hit, 'Should hit diagonal target');
});

test('raycast with no bodies', () => {
  const hit = raycast(new Vec2(0, 0), new Vec2(1, 0), []);
  assert.equal(hit, null);
});

test('raycast skips bodies behind origin', () => {
  const bodies = [new Body({ position: new Vec2(-10, 0), shape: { type: 'circle', radius: 3 } })];
  const hit = raycast(new Vec2(0, 0), new Vec2(1, 0), bodies);
  assert.equal(hit, null, 'Should not hit body behind ray');
});
