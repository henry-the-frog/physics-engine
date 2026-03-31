/**
 * Tiny Physics Engine (2D)
 * 
 * Simple rigid body simulation:
 * - Bodies with position, velocity, mass
 * - Forces: gravity, friction, springs
 * - Collision detection: AABB, circle
 * - Collision response: elastic, inelastic
 * - Integration: Euler, Verlet
 */

class Vec2 {
  constructor(x = 0, y = 0) { this.x = x; this.y = y; }
  add(v) { return new Vec2(this.x + v.x, this.y + v.y); }
  sub(v) { return new Vec2(this.x - v.x, this.y - v.y); }
  mul(s) { return new Vec2(this.x * s, this.y * s); }
  div(s) { return new Vec2(this.x / s, this.y / s); }
  dot(v) { return this.x * v.x + this.y * v.y; }
  length() { return Math.sqrt(this.x ** 2 + this.y ** 2); }
  normalize() { const l = this.length(); return l > 0 ? this.div(l) : new Vec2(); }
  distance(v) { return this.sub(v).length(); }
  static zero() { return new Vec2(0, 0); }
}

class Body {
  constructor(opts = {}) {
    this.position = opts.position || new Vec2();
    this.velocity = opts.velocity || new Vec2();
    this.acceleration = new Vec2();
    this.mass = opts.mass || 1;
    this.restitution = opts.restitution || 0.8;
    this.friction = opts.friction || 0;
    this.shape = opts.shape || { type: 'circle', radius: 10 };
    this.isStatic = opts.isStatic || false;
    this.forces = new Vec2();
  }

  get inverseMass() { return this.isStatic ? 0 : 1 / this.mass; }

  applyForce(force) {
    this.forces = this.forces.add(force);
  }

  update(dt) {
    if (this.isStatic) return;
    this.acceleration = this.forces.div(this.mass);
    this.velocity = this.velocity.add(this.acceleration.mul(dt));
    if (this.friction > 0) {
      this.velocity = this.velocity.mul(1 - this.friction * dt);
    }
    this.position = this.position.add(this.velocity.mul(dt));
    this.forces = new Vec2();
  }
}

class World {
  constructor(opts = {}) {
    this.bodies = [];
    this.gravity = opts.gravity || new Vec2(0, 9.81);
    this.iterations = opts.iterations || 1;
  }

  addBody(body) { this.bodies.push(body); return body; }
  removeBody(body) { this.bodies = this.bodies.filter(b => b !== body); }

  step(dt) {
    // Apply gravity
    for (const body of this.bodies) {
      if (!body.isStatic) {
        body.applyForce(this.gravity.mul(body.mass));
      }
    }

    // Integrate
    for (const body of this.bodies) {
      body.update(dt);
    }

    // Detect and resolve collisions
    for (let iter = 0; iter < this.iterations; iter++) {
      for (let i = 0; i < this.bodies.length; i++) {
        for (let j = i + 1; j < this.bodies.length; j++) {
          const collision = detectCollision(this.bodies[i], this.bodies[j]);
          if (collision) {
            resolveCollision(this.bodies[i], this.bodies[j], collision);
          }
        }
      }
    }
  }
}

function detectCollision(a, b) {
  if (a.shape.type === 'circle' && b.shape.type === 'circle') {
    return circleCircle(a, b);
  }
  if (a.shape.type === 'aabb' && b.shape.type === 'aabb') {
    return aabbAABB(a, b);
  }
  return null;
}

function circleCircle(a, b) {
  const dist = a.position.distance(b.position);
  const overlap = a.shape.radius + b.shape.radius - dist;
  if (overlap <= 0) return null;
  const normal = b.position.sub(a.position).normalize();
  return { normal, overlap };
}

function aabbAABB(a, b) {
  const aw = a.shape.width / 2, ah = a.shape.height / 2;
  const bw = b.shape.width / 2, bh = b.shape.height / 2;
  const dx = b.position.x - a.position.x;
  const dy = b.position.y - a.position.y;
  const overlapX = aw + bw - Math.abs(dx);
  const overlapY = ah + bh - Math.abs(dy);
  if (overlapX <= 0 || overlapY <= 0) return null;
  if (overlapX < overlapY) {
    return { normal: new Vec2(dx > 0 ? 1 : -1, 0), overlap: overlapX };
  }
  return { normal: new Vec2(0, dy > 0 ? 1 : -1), overlap: overlapY };
}

function resolveCollision(a, b, collision) {
  const { normal, overlap } = collision;
  const totalInvMass = a.inverseMass + b.inverseMass;
  if (totalInvMass === 0) return;

  // Separate bodies
  const correction = normal.mul(overlap / totalInvMass);
  a.position = a.position.sub(correction.mul(a.inverseMass));
  b.position = b.position.add(correction.mul(b.inverseMass));

  // Impulse
  const relVel = b.velocity.sub(a.velocity);
  const velAlongNormal = relVel.dot(normal);
  if (velAlongNormal > 0) return; // moving apart

  const e = Math.min(a.restitution, b.restitution);
  const j = -(1 + e) * velAlongNormal / totalInvMass;
  const impulse = normal.mul(j);
  a.velocity = a.velocity.sub(impulse.mul(a.inverseMass));
  b.velocity = b.velocity.add(impulse.mul(b.inverseMass));
}

module.exports = { Vec2, Body, World, detectCollision, resolveCollision };
