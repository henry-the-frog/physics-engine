const { test } = require('node:test');
const assert = require('node:assert/strict');
const { Vec2, Body, World } = require('../src/index.js');

test('collision event fires on impact', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const a = world.addBody(new Body({
    position: new Vec2(0, 0), velocity: new Vec2(50, 0),
    shape: { type: 'circle', radius: 10 }, canSleep: false,
  }));
  const b = world.addBody(new Body({
    position: new Vec2(15, 0),
    shape: { type: 'circle', radius: 10 }, canSleep: false,
  }));
  
  const events = [];
  world.on('collision', (e) => events.push(e));
  
  world.step(0.016);
  
  assert.ok(events.length > 0, 'Should fire collision event');
  assert.ok(events.some(e => e.type === 'start'), 'Should have start event');
});

test('collision end event fires when bodies separate', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const a = world.addBody(new Body({
    position: new Vec2(0, 0), velocity: new Vec2(50, 0),
    shape: { type: 'circle', radius: 10 }, canSleep: false,
    restitution: 1,
  }));
  const b = world.addBody(new Body({
    position: new Vec2(15, 0), velocity: new Vec2(-50, 0),
    shape: { type: 'circle', radius: 10 }, canSleep: false,
    restitution: 1,
  }));
  
  const endEvents = [];
  world.on('collision', (e) => { if (e.type === 'end') endEvents.push(e); });
  
  // Step multiple times — collision should start then end
  for (let i = 0; i < 20; i++) world.step(0.016);
  
  assert.ok(endEvents.length > 0, 'Should fire end event when bodies separate');
});

test('trigger body fires trigger event', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const sensor = world.addBody(new Body({
    position: new Vec2(0, 0),
    shape: { type: 'circle', radius: 10 },
    isTrigger: true, isStatic: true,
  }));
  const ball = world.addBody(new Body({
    position: new Vec2(15, 0), velocity: new Vec2(-50, 0),
    shape: { type: 'circle', radius: 5 }, canSleep: false,
  }));
  
  const triggers = [];
  world.on('trigger', (e) => triggers.push(e));
  
  world.step(0.016);
  
  assert.ok(triggers.length > 0, 'Should fire trigger event');
  // Trigger should not resolve physics (ball passes through)
  assert.ok(ball.velocity.x < 0, 'Ball should continue moving (no physics resolution)');
});

test('collision event contains body references', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const a = world.addBody(new Body({
    position: new Vec2(0, 0), velocity: new Vec2(50, 0),
    shape: { type: 'circle', radius: 10 }, canSleep: false,
  }));
  const b = world.addBody(new Body({
    position: new Vec2(15, 0),
    shape: { type: 'circle', radius: 10 }, canSleep: false,
  }));
  
  let event = null;
  world.on('collision', (e) => { if (!event) event = e; });
  
  world.step(0.016);
  
  assert.ok(event.bodyA);
  assert.ok(event.bodyB);
  assert.ok(event.normal);
});

test('off removes listener', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  let count = 0;
  const cb = () => count++;
  
  world.on('collision', cb);
  world.off('collision', cb);
  
  world.addBody(new Body({ position: new Vec2(0, 0), velocity: new Vec2(50, 0), shape: { type: 'circle', radius: 10 }, canSleep: false }));
  world.addBody(new Body({ position: new Vec2(15, 0), shape: { type: 'circle', radius: 10 }, canSleep: false }));
  world.step(0.016);
  
  assert.equal(count, 0, 'Removed listener should not fire');
});

test('multiple listeners', () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  let count1 = 0, count2 = 0;
  world.on('collision', () => count1++);
  world.on('collision', () => count2++);
  
  world.addBody(new Body({ position: new Vec2(0, 0), velocity: new Vec2(50, 0), shape: { type: 'circle', radius: 10 }, canSleep: false }));
  world.addBody(new Body({ position: new Vec2(15, 0), shape: { type: 'circle', radius: 10 }, canSleep: false }));
  world.step(0.016);
  
  assert.ok(count1 > 0);
  assert.ok(count2 > 0);
  assert.equal(count1, count2);
});
