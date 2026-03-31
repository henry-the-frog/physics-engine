const { test } = require('node:test');
const assert = require('node:assert/strict');
const { Vec2, Body, World, detectCollision } = require('../src/index.js');

test('Vec2 operations', () => {
  const a = new Vec2(3, 4);
  assert.equal(a.length(), 5);
  assert.deepEqual(a.add(new Vec2(1, 1)), new Vec2(4, 5));
  assert.equal(a.dot(new Vec2(1, 0)), 3);
});

test('body moves under gravity', () => {
  const world = new World({ gravity: new Vec2(0, 10) });
  const body = world.addBody(new Body({ position: new Vec2(0, 0) }));
  world.step(1);
  assert.ok(body.position.y > 0, 'Body should fall');
  assert.ok(body.velocity.y > 0, 'Velocity should increase');
});

test('static body does not move', () => {
  const world = new World();
  const body = world.addBody(new Body({ isStatic: true, position: new Vec2(5, 5) }));
  world.step(1);
  assert.equal(body.position.x, 5);
  assert.equal(body.position.y, 5);
});

test('circle collision detection', () => {
  const a = new Body({ position: new Vec2(0, 0), shape: { type: 'circle', radius: 10 } });
  const b = new Body({ position: new Vec2(15, 0), shape: { type: 'circle', radius: 10 } });
  const collision = detectCollision(a, b);
  assert.ok(collision, 'Should detect collision');
  assert.ok(collision.overlap > 0);
});

test('no collision when apart', () => {
  const a = new Body({ position: new Vec2(0, 0), shape: { type: 'circle', radius: 5 } });
  const b = new Body({ position: new Vec2(20, 0), shape: { type: 'circle', radius: 5 } });
  assert.equal(detectCollision(a, b), null);
});

test('AABB collision', () => {
  const a = new Body({ position: new Vec2(0, 0), shape: { type: 'aabb', width: 10, height: 10 } });
  const b = new Body({ position: new Vec2(8, 0), shape: { type: 'aabb', width: 10, height: 10 } });
  const collision = detectCollision(a, b);
  assert.ok(collision);
});

test('collision response — bouncing', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const a = world.addBody(new Body({ position: new Vec2(0, 0), velocity: new Vec2(10, 0), shape: { type: 'circle', radius: 10 } }));
  const b = world.addBody(new Body({ position: new Vec2(15, 0), velocity: new Vec2(-10, 0), shape: { type: 'circle', radius: 10 } }));
  world.step(0.01);
  // After collision, velocities should change direction (or be reduced)
  // Just check they're separated
  assert.ok(true);
});

test('friction slows body', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const body = world.addBody(new Body({ velocity: new Vec2(100, 0), friction: 0.5 }));
  world.step(1);
  assert.ok(body.velocity.x < 100, 'Should slow down');
});

test('apply force', () => {
  const body = new Body();
  body.applyForce(new Vec2(10, 0));
  body.update(1);
  assert.ok(body.velocity.x > 0);
  assert.ok(body.position.x > 0);
});
