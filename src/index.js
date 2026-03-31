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

let _nextBodyId = 1;

class Body {
  constructor(opts = {}) {
    this.id = _nextBodyId++;
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

class SpatialHashGrid {
  constructor(cellSize = 50) {
    this.cellSize = cellSize;
    this.grid = new Map(); // "x,y" → Set<Body>
  }

  _hash(x, y) {
    const cx = Math.floor(x / this.cellSize);
    const cy = Math.floor(y / this.cellSize);
    return `${cx},${cy}`;
  }

  _getCells(body) {
    const cells = [];
    if (body.shape.type === 'circle') {
      const r = body.shape.radius;
      const minX = body.position.x - r;
      const maxX = body.position.x + r;
      const minY = body.position.y - r;
      const maxY = body.position.y + r;
      for (let x = Math.floor(minX / this.cellSize); x <= Math.floor(maxX / this.cellSize); x++) {
        for (let y = Math.floor(minY / this.cellSize); y <= Math.floor(maxY / this.cellSize); y++) {
          cells.push(`${x},${y}`);
        }
      }
    } else if (body.shape.type === 'aabb') {
      const hw = body.shape.width / 2, hh = body.shape.height / 2;
      const minX = body.position.x - hw;
      const maxX = body.position.x + hw;
      const minY = body.position.y - hh;
      const maxY = body.position.y + hh;
      for (let x = Math.floor(minX / this.cellSize); x <= Math.floor(maxX / this.cellSize); x++) {
        for (let y = Math.floor(minY / this.cellSize); y <= Math.floor(maxY / this.cellSize); y++) {
          cells.push(`${x},${y}`);
        }
      }
    }
    return cells;
  }

  clear() {
    this.grid.clear();
  }

  insert(body) {
    for (const key of this._getCells(body)) {
      if (!this.grid.has(key)) this.grid.set(key, new Set());
      this.grid.get(key).add(body);
    }
  }

  // Returns candidate pairs (may contain duplicates — caller deduplicates)
  getCandidatePairs() {
    const pairs = new Set();
    for (const [, bodies] of this.grid) {
      const arr = [...bodies];
      for (let i = 0; i < arr.length; i++) {
        for (let j = i + 1; j < arr.length; j++) {
          // Order by reference to avoid duplicates
          const a = arr[i], b = arr[j];
          const key = a.id < b.id ? `${a.id}_${b.id}` : `${b.id}_${a.id}`;
          pairs.add(key);
        }
      }
    }
    return pairs;
  }

  // Get all candidate bodies that might collide with the given body
  query(body) {
    const result = new Set();
    for (const key of this._getCells(body)) {
      const cell = this.grid.get(key);
      if (cell) {
        for (const b of cell) {
          if (b !== body) result.add(b);
        }
      }
    }
    return result;
  }
}

class World {
  constructor(opts = {}) {
    this.bodies = [];
    this.gravity = opts.gravity || new Vec2(0, 9.81);
    this.iterations = opts.iterations || 1;
    this.broadphase = opts.broadphase !== false; // Enable by default
    this.cellSize = opts.cellSize || 50;
    this._grid = new SpatialHashGrid(this.cellSize);
    this._broadphaseChecks = 0;
    this._narrowphaseChecks = 0;
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
    this._broadphaseChecks = 0;
    this._narrowphaseChecks = 0;
    
    for (let iter = 0; iter < this.iterations; iter++) {
      if (this.broadphase && this.bodies.length > 8) {
        this._collideWithBroadphase();
      } else {
        this._collideNaive();
      }
    }
  }

  _collideNaive() {
    for (let i = 0; i < this.bodies.length; i++) {
      for (let j = i + 1; j < this.bodies.length; j++) {
        this._narrowphaseChecks++;
        const collision = detectCollision(this.bodies[i], this.bodies[j]);
        if (collision) {
          resolveCollision(this.bodies[i], this.bodies[j], collision);
        }
      }
    }
  }

  _collideWithBroadphase() {
    // Build spatial hash grid
    this._grid.clear();
    for (const body of this.bodies) {
      this._grid.insert(body);
    }

    // Check only candidate pairs from the grid
    const checked = new Set();
    for (const body of this.bodies) {
      const candidates = this._grid.query(body);
      for (const other of candidates) {
        const key = body.id < other.id ? `${body.id}_${other.id}` : `${other.id}_${body.id}`;
        if (checked.has(key)) continue;
        checked.add(key);
        this._broadphaseChecks++;
        this._narrowphaseChecks++;
        const collision = detectCollision(body, other);
        if (collision) {
          resolveCollision(body, other, collision);
        }
      }
    }
  }

  get stats() {
    return {
      bodies: this.bodies.length,
      broadphaseChecks: this._broadphaseChecks,
      narrowphaseChecks: this._narrowphaseChecks,
    };
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

module.exports = { Vec2, Body, World, SpatialHashGrid, detectCollision, resolveCollision };
