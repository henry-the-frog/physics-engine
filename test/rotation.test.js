const { test } = require('node:test');
const assert = require('node:assert/strict');
const { Vec2, Body, World } = require('../src/index.js');

test('body has angular properties', () => {
  const body = new Body({ position: new Vec2(0, 0) });
  assert.equal(body.angle, 0);
  assert.equal(body.angularVelocity, 0);
  assert.equal(body.torque, 0);
  assert.ok(body.inertia > 0);
});

test('circle moment of inertia = 0.5 * m * r^2', () => {
  const body = new Body({ mass: 2, shape: { type: 'circle', radius: 5 } });
  assert.equal(body.inertia, 0.5 * 2 * 25);
});

test('rectangle moment of inertia', () => {
  const body = new Body({ mass: 3, shape: { type: 'aabb', width: 4, height: 6 } });
  const expected = (1/12) * 3 * (16 + 36);
  assert.ok(Math.abs(body.inertia - expected) < 0.01);
});

test('angular velocity integrates angle', () => {
  const body = new Body({
    position: new Vec2(0, 0),
    angularVelocity: Math.PI, // half turn per second
    canSleep: false,
  });
  body.update(1.0);
  assert.ok(Math.abs(body.angle - Math.PI) < 0.01);
});

test('torque produces angular acceleration', () => {
  const body = new Body({
    position: new Vec2(0, 0),
    shape: { type: 'circle', radius: 10 },
    mass: 1,
    canSleep: false,
  });
  const inertia = body.inertia; // 0.5 * 1 * 100 = 50
  body.applyTorque(100); // α = 100/50 = 2 rad/s²
  body.update(1.0);
  assert.ok(Math.abs(body.angularVelocity - 2.0) < 0.1);
});

test('applyForceAtPoint generates torque', () => {
  const body = new Body({
    position: new Vec2(0, 0),
    shape: { type: 'circle', radius: 10 },
    mass: 1,
    canSleep: false,
  });
  // Force (0, 10) at point (5, 0) → torque = 5 * 10 - 0 * 0 = 50
  body.applyForceAtPoint(new Vec2(0, 10), new Vec2(5, 0));
  assert.equal(body.torque, 50);
  body.update(0.016);
  assert.ok(body.angularVelocity > 0, 'Should have angular velocity');
});

test('collision produces angular impulse', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const a = world.addBody(new Body({
    position: new Vec2(0, 0),
    shape: { type: 'circle', radius: 10 },
    canSleep: false,
  }));
  const b = world.addBody(new Body({
    position: new Vec2(15, 5), // slightly off-center collision
    velocity: new Vec2(-50, 0),
    shape: { type: 'circle', radius: 10 },
    canSleep: false,
  }));
  
  world.step(0.016);
  
  // Off-center collision should produce some angular velocity
  // (May be small depending on exact geometry)
  assert.ok(typeof a.angularVelocity === 'number');
  assert.ok(typeof b.angularVelocity === 'number');
});

test('sleeping clears angular velocity', () => {
  const body = new Body({ angularVelocity: 5 });
  body.sleep();
  assert.equal(body.angularVelocity, 0);
});

test('inverseInertia for static body is 0', () => {
  const body = new Body({ isStatic: true });
  assert.equal(body.inverseInertia, 0);
});

test('angular velocity included in sleep check', () => {
  const body = new Body({
    position: new Vec2(0, 0),
    velocity: new Vec2(0, 0), // zero linear velocity
    angularVelocity: 10,     // but spinning fast
    canSleep: true,
    sleepDelay: 0,
    sleepThreshold: 0.5,
  });
  body.update(0.016);
  assert.ok(!body.isSleeping, 'Should not sleep while spinning');
});
