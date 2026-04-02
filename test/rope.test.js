// rope.test.js — Tests for RopeConstraint
const assert = require('node:assert/strict');
const { describe, it } = require('node:test');
const { Vec2, Body, RopeConstraint } = require('../src/index.js');

describe('RopeConstraint', () => {
  function makeBodies(n, spacing = 20) {
    const bodies = [];
    for (let i = 0; i < n; i++) {
      bodies.push(new Body({
        position: new Vec2(i * spacing, 0),
        mass: 1,
        shape: { type: 'circle', radius: 5 }
      }));
    }
    return bodies;
  }

  it('should create a rope from multiple bodies', () => {
    const bodies = makeBodies(5);
    const rope = new RopeConstraint(bodies);

    assert.equal(rope.segmentCount, 4);
    assert.equal(rope.bodies.length, 5);
    assert.ok(rope.length > 0);
  });

  it('should throw with fewer than 2 bodies', () => {
    assert.throws(() => new RopeConstraint([new Body()]), /at least 2 bodies/);
    assert.throws(() => new RopeConstraint([]), /at least 2 bodies/);
  });

  it('should maintain segment lengths after solving', () => {
    const bodies = makeBodies(4, 20);
    const rope = new RopeConstraint(bodies, { segmentLength: 20, iterations: 20 });

    // Displace a body
    bodies[2].position = new Vec2(100, 50);
    rope.solve();

    // Check that adjacent distances are close to segmentLength
    for (let i = 0; i < bodies.length - 1; i++) {
      const dist = bodies[i].position.distance(bodies[i + 1].position);
      assert.ok(Math.abs(dist - 20) < 1, `Segment ${i} dist=${dist.toFixed(2)}, expected ~20`);
    }
  });

  it('should pin first body when pinFirst=true', () => {
    const bodies = makeBodies(4, 20);
    const startPos = new Vec2(bodies[0].position.x, bodies[0].position.y);
    const rope = new RopeConstraint(bodies, { pinFirst: true, iterations: 10 });

    // Displace middle body
    bodies[2].position = new Vec2(80, 100);
    rope.solve();

    // First body should not have moved
    assert.ok(Math.abs(bodies[0].position.x - startPos.x) < 0.01);
    assert.ok(Math.abs(bodies[0].position.y - startPos.y) < 0.01);
  });

  it('should pin last body when pinLast=true', () => {
    const bodies = makeBodies(3, 30);
    const lastPos = new Vec2(bodies[2].position.x, bodies[2].position.y);
    const rope = new RopeConstraint(bodies, { pinLast: true, iterations: 10 });

    bodies[0].position = new Vec2(-50, 50);
    rope.solve();

    assert.ok(Math.abs(bodies[2].position.x - lastPos.x) < 0.01);
    assert.ok(Math.abs(bodies[2].position.y - lastPos.y) < 0.01);
  });

  it('should pin both ends', () => {
    const bodies = makeBodies(5, 20);
    const firstPos = new Vec2(bodies[0].position.x, bodies[0].position.y);
    const lastPos = new Vec2(bodies[4].position.x, bodies[4].position.y);

    const rope = new RopeConstraint(bodies, { pinFirst: true, pinLast: true, iterations: 20 });

    // Move middle bodies down
    bodies[1].position = new Vec2(20, 100);
    bodies[2].position = new Vec2(40, 150);
    bodies[3].position = new Vec2(60, 100);
    rope.solve();

    // Endpoints should be unchanged
    assert.ok(Math.abs(bodies[0].position.x - firstPos.x) < 0.01);
    assert.ok(Math.abs(bodies[4].position.x - lastPos.x) < 0.01);
  });

  it('should support custom segment length', () => {
    const bodies = makeBodies(3, 50);
    const rope = new RopeConstraint(bodies, { segmentLength: 10, iterations: 30 });

    rope.solve();

    for (let i = 0; i < 2; i++) {
      const dist = bodies[i].position.distance(bodies[i + 1].position);
      assert.ok(Math.abs(dist - 10) < 0.5, `Segment ${i} dist=${dist.toFixed(2)}, expected ~10`);
    }
  });

  it('should return points for rendering', () => {
    const bodies = makeBodies(3);
    const rope = new RopeConstraint(bodies);
    const points = rope.getPoints();

    assert.equal(points.length, 3);
    assert.ok('x' in points[0] && 'y' in points[0]);
  });

  it('should apply gravity to non-pinned bodies', () => {
    const bodies = makeBodies(4, 20);
    const rope = new RopeConstraint(bodies, { pinFirst: true });
    const gravity = new Vec2(0, 9.8);

    rope.applyGravity(gravity);

    // First body (pinned) should have no force applied
    assert.ok(Math.abs(bodies[0].forces.y) < 0.01, 'Pinned body should not receive gravity');
    // Others should
    assert.ok(bodies[1].forces.y > 0);
    assert.ok(bodies[2].forces.y > 0);
    assert.ok(bodies[3].forces.y > 0);
  });

  it('more iterations = stiffer rope', () => {
    // Two ropes with same displacement, different iterations
    const bodies1 = makeBodies(5, 20);
    const bodies2 = makeBodies(5, 20);
    
    const rope1 = new RopeConstraint(bodies1, { iterations: 1 });
    const rope2 = new RopeConstraint(bodies2, { iterations: 20 });

    // Displace after rope creation
    bodies1[2].position = new Vec2(40, 80);
    bodies2[2].position = new Vec2(40, 80);

    rope1.solve();
    rope2.solve();

    // Higher iterations → distances closer to segment length
    let maxErr1 = 0, maxErr2 = 0;
    for (let i = 0; i < 4; i++) {
      maxErr1 = Math.max(maxErr1, Math.abs(bodies1[i].position.distance(bodies1[i+1].position) - 20));
      maxErr2 = Math.max(maxErr2, Math.abs(bodies2[i].position.distance(bodies2[i+1].position) - 20));
    }
    assert.ok(maxErr2 <= maxErr1, `High iterations (err=${maxErr2.toFixed(2)}) should be <= low (err=${maxErr1.toFixed(2)})`);
  });

  it('should handle long rope (20 segments)', () => {
    const bodies = makeBodies(21, 10);
    const rope = new RopeConstraint(bodies, { pinFirst: true, iterations: 15 });

    // Swing the end
    bodies[20].position = new Vec2(200, 150);
    rope.solve();

    // First body still pinned
    assert.ok(Math.abs(bodies[0].position.x) < 0.01);
    // Total rope should still be reasonable (not exploded)
    for (const b of bodies) {
      assert.ok(Math.abs(b.position.x) < 500 && Math.abs(b.position.y) < 500, 'No body should explode');
    }
  });
});
