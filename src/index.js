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
    
    // Sleeping state
    this.isSleeping = false;
    this.sleepThreshold = opts.sleepThreshold || 0.5; // velocity below this → can sleep
    this.sleepTimer = 0;    // how long velocity has been below threshold
    this.sleepDelay = opts.sleepDelay || 0.5; // seconds below threshold before sleeping
    this.canSleep = opts.canSleep !== false; // enable sleeping by default
    
    // Rotation
    this.angle = opts.angle || 0;           // radians
    this.angularVelocity = opts.angularVelocity || 0;
    this.torque = 0;
    this.inertia = opts.inertia || this._computeInertia();
  }

  get inverseMass() { return this.isStatic ? 0 : 1 / this.mass; }
  get inverseInertia() { return this.isStatic ? 0 : 1 / this.inertia; }

  _computeInertia() {
    if (this.shape.type === 'circle') {
      return 0.5 * this.mass * this.shape.radius * this.shape.radius;
    }
    if (this.shape.type === 'aabb') {
      const w = this.shape.width, h = this.shape.height;
      return (1 / 12) * this.mass * (w * w + h * h);
    }
    return this.mass * 100;
  }

  applyForce(force) {
    if (force.x === 0 && force.y === 0) return;
    if (this.isSleeping) this.wake();
    this.forces = this.forces.add(force);
  }

  applyTorque(t) {
    if (t === 0) return;
    if (this.isSleeping) this.wake();
    this.torque += t;
  }

  applyForceAtPoint(force, point) {
    this.applyForce(force);
    const r = point.sub(this.position);
    this.applyTorque(r.x * force.y - r.y * force.x);
  }

  wake() {
    this.isSleeping = false;
    this.sleepTimer = 0;
  }

  sleep() {
    this.isSleeping = true;
    this.velocity = new Vec2();
    this.acceleration = new Vec2();
    this.angularVelocity = 0;
  }

  update(dt) {
    if (this.isStatic || this.isSleeping) return;
    this.acceleration = this.forces.div(this.mass);
    this.velocity = this.velocity.add(this.acceleration.mul(dt));
    if (this.friction > 0) {
      this.velocity = this.velocity.mul(1 - this.friction * dt);
    }
    this.position = this.position.add(this.velocity.mul(dt));
    this.forces = new Vec2();
    
    // Angular integration
    const angularAccel = this.torque / this.inertia;
    this.angularVelocity += angularAccel * dt;
    this.angularVelocity *= 0.999; // tiny angular damping
    this.angle += this.angularVelocity * dt;
    this.torque = 0;
    
    // Sleep check (include angular velocity)
    const totalVelocity = this.velocity.length() + Math.abs(this.angularVelocity) * 10;
    if (this.canSleep && totalVelocity < this.sleepThreshold) {
      this.sleepTimer += dt;
      if (this.sleepTimer >= this.sleepDelay) {
        this.sleep();
      }
    } else {
      this.sleepTimer = 0;
    }
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
    this.constraints = [];
    this.springs = [];
    this.gravity = opts.gravity || new Vec2(0, 9.81);
    this.iterations = opts.iterations || 1;
    this.constraintIterations = opts.constraintIterations || 4;
    this.broadphase = opts.broadphase !== false;
    this.cellSize = opts.cellSize || 50;
    this._grid = new SpatialHashGrid(this.cellSize);
    this._broadphaseChecks = 0;
    this._narrowphaseChecks = 0;
  }

  addBody(body) { this.bodies.push(body); return body; }
  removeBody(body) { this.bodies = this.bodies.filter(b => b !== body); }
  
  addConstraint(constraint) { this.constraints.push(constraint); return constraint; }
  removeConstraint(constraint) { this.constraints = this.constraints.filter(c => c !== constraint); }
  
  addSpring(spring) { this.springs.push(spring); return spring; }
  removeSpring(spring) { this.springs = this.springs.filter(s => s !== spring); }

  step(dt) {
    // Apply gravity
    for (const body of this.bodies) {
      if (!body.isStatic && !body.isSleeping) {
        body.applyForce(this.gravity.mul(body.mass));
      }
    }
    
    // Apply spring forces
    for (const spring of this.springs) {
      spring.apply();
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
    
    // Solve distance constraints (multiple iterations for stability)
    for (let iter = 0; iter < this.constraintIterations; iter++) {
      for (const constraint of this.constraints) {
        constraint.solve();
      }
    }
  }

  _collideNaive() {
    for (let i = 0; i < this.bodies.length; i++) {
      for (let j = i + 1; j < this.bodies.length; j++) {
        const a = this.bodies[i], b = this.bodies[j];
        // Skip pairs where both are sleeping or static
        if ((a.isSleeping || a.isStatic) && (b.isSleeping || b.isStatic)) continue;
        this._narrowphaseChecks++;
        const collision = detectCollision(a, b);
        if (collision) {
          resolveCollision(a, b, collision);
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
      constraints: this.constraints.length,
      springs: this.springs.length,
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
  if (a.shape.type === 'polygon' && b.shape.type === 'polygon') {
    return polygonPolygon(a, b);
  }
  if (a.shape.type === 'circle' && b.shape.type === 'polygon') {
    return circlePolygon(a, b);
  }
  if (a.shape.type === 'polygon' && b.shape.type === 'circle') {
    const result = circlePolygon(b, a);
    if (result) result.normal = result.normal.mul(-1);
    return result;
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

// Get world-space vertices of a polygon body
function getWorldVertices(body) {
  const cos = Math.cos(body.angle);
  const sin = Math.sin(body.angle);
  return body.shape.vertices.map(v => {
    const rx = v.x * cos - v.y * sin;
    const ry = v.x * sin + v.y * cos;
    return new Vec2(body.position.x + rx, body.position.y + ry);
  });
}

// Get edge normals for a polygon
function getEdgeNormals(vertices) {
  const normals = [];
  for (let i = 0; i < vertices.length; i++) {
    const j = (i + 1) % vertices.length;
    const edge = vertices[j].sub(vertices[i]);
    // Normal is perpendicular to edge
    normals.push(new Vec2(-edge.y, edge.x).normalize());
  }
  return normals;
}

// Project polygon vertices onto an axis, return [min, max]
function projectPolygon(vertices, axis) {
  let min = Infinity, max = -Infinity;
  for (const v of vertices) {
    const proj = v.dot(axis);
    if (proj < min) min = proj;
    if (proj > max) max = proj;
  }
  return { min, max };
}

// SAT polygon-polygon collision
function polygonPolygon(a, b) {
  const vertsA = getWorldVertices(a);
  const vertsB = getWorldVertices(b);
  const normalsA = getEdgeNormals(vertsA);
  const normalsB = getEdgeNormals(vertsB);
  
  let minOverlap = Infinity;
  let minNormal = null;
  
  // Test all axes from both polygons
  const allNormals = [...normalsA, ...normalsB];
  for (const axis of allNormals) {
    const projA = projectPolygon(vertsA, axis);
    const projB = projectPolygon(vertsB, axis);
    
    const overlap = Math.min(projA.max - projB.min, projB.max - projA.min);
    if (overlap <= 0) return null; // Separating axis found
    
    if (overlap < minOverlap) {
      minOverlap = overlap;
      // Normal should point from A to B
      const d = b.position.sub(a.position);
      minNormal = d.dot(axis) >= 0 ? axis : axis.mul(-1);
    }
  }
  
  return { normal: minNormal, overlap: minOverlap };
}

// Circle-polygon collision
function circlePolygon(circle, polygon) {
  const verts = getWorldVertices(polygon);
  const center = circle.position;
  const radius = circle.shape.radius;
  
  let minOverlap = Infinity;
  let minNormal = null;
  
  // Test polygon edge normals
  const normals = getEdgeNormals(verts);
  for (const axis of normals) {
    const proj = projectPolygon(verts, axis);
    const circleProj = center.dot(axis);
    const circleMin = circleProj - radius;
    const circleMax = circleProj + radius;
    
    const overlap = Math.min(proj.max - circleMin, circleMax - proj.min);
    if (overlap <= 0) return null;
    
    if (overlap < minOverlap) {
      minOverlap = overlap;
      const d = polygon.position.sub(center);
      minNormal = d.dot(axis) >= 0 ? axis : axis.mul(-1);
    }
  }
  
  // Test axis from circle center to nearest vertex
  let nearestVert = verts[0];
  let nearestDist = Infinity;
  for (const v of verts) {
    const d = center.distance(v);
    if (d < nearestDist) { nearestDist = d; nearestVert = v; }
  }
  
  const vertAxis = nearestVert.sub(center).normalize();
  if (vertAxis.length() > 0) {
    const proj = projectPolygon(verts, vertAxis);
    const circleProj = center.dot(vertAxis);
    const circleMin = circleProj - radius;
    const circleMax = circleProj + radius;
    
    const overlap = Math.min(proj.max - circleMin, circleMax - proj.min);
    if (overlap <= 0) return null;
    
    if (overlap < minOverlap) {
      minOverlap = overlap;
      const d = polygon.position.sub(center);
      minNormal = d.dot(vertAxis) >= 0 ? vertAxis : vertAxis.mul(-1);
    }
  }
  
  return { normal: minNormal, overlap: minOverlap };
}

function resolveCollision(a, b, collision) {
  const { normal, overlap } = collision;
  const totalInvMass = a.inverseMass + b.inverseMass;
  if (totalInvMass === 0) return;

  // Wake sleeping bodies on collision
  if (a.isSleeping && !b.isStatic) a.wake();
  if (b.isSleeping && !a.isStatic) b.wake();

  // Separate bodies (positional correction)
  const correction = normal.mul(overlap / totalInvMass);
  a.position = a.position.sub(correction.mul(a.inverseMass));
  b.position = b.position.add(correction.mul(b.inverseMass));

  // Relative velocity
  const relVel = b.velocity.sub(a.velocity);
  const velAlongNormal = relVel.dot(normal);
  if (velAlongNormal > 0) return; // moving apart

  // Normal impulse (restitution)
  const e = Math.min(a.restitution, b.restitution);
  
  // Contact point (midpoint of overlap for circles)
  const contactPoint = a.position.add(normal.mul(a.shape?.radius || 0));
  const rA = contactPoint.sub(a.position);
  const rB = contactPoint.sub(b.position);
  
  // Cross product helpers (2D)
  const crossRAN = rA.x * normal.y - rA.y * normal.x;
  const crossRBN = rB.x * normal.y - rB.y * normal.x;
  
  // Effective mass including rotational inertia
  const effectiveMass = totalInvMass + 
    crossRAN * crossRAN * a.inverseInertia + 
    crossRBN * crossRBN * b.inverseInertia;
  
  const jN = -(1 + e) * velAlongNormal / effectiveMass;
  const normalImpulse = normal.mul(jN);
  a.velocity = a.velocity.sub(normalImpulse.mul(a.inverseMass));
  b.velocity = b.velocity.add(normalImpulse.mul(b.inverseMass));
  
  // Angular impulse from normal force
  a.angularVelocity -= (rA.x * normalImpulse.y - rA.y * normalImpulse.x) * a.inverseInertia;
  b.angularVelocity += (rB.x * normalImpulse.y - rB.y * normalImpulse.x) * b.inverseInertia;

  // Coulomb friction (tangential impulse)
  const mu = Math.sqrt(a.friction * a.friction + b.friction * b.friction); // combined friction
  if (mu > 0) {
    // Recalculate relative velocity after normal impulse
    const relVel2 = b.velocity.sub(a.velocity);
    // Tangent direction: component of relative velocity perpendicular to normal
    const tangent = relVel2.sub(normal.mul(relVel2.dot(normal)));
    const tangentLen = tangent.length();
    if (tangentLen > 0.0001) {
      const tangentDir = tangent.normalize();
      const velAlongTangent = relVel2.dot(tangentDir);
      
      // Friction impulse magnitude: clamped by Coulomb's law (|Ft| <= mu * |Fn|)
      let jT = -velAlongTangent / totalInvMass;
      if (Math.abs(jT) > mu * Math.abs(jN)) {
        // Dynamic friction: apply full friction force
        jT = jT > 0 ? mu * Math.abs(jN) : -mu * Math.abs(jN);
      }
      
      const frictionImpulse = tangentDir.mul(jT);
      a.velocity = a.velocity.sub(frictionImpulse.mul(a.inverseMass));
      b.velocity = b.velocity.add(frictionImpulse.mul(b.inverseMass));
    }
  }
}

// === Constraints ===

class DistanceConstraint {
  // Maintains a fixed distance between two bodies
  constructor(bodyA, bodyB, opts = {}) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.distance = opts.distance || bodyA.position.distance(bodyB.position);
    this.stiffness = opts.stiffness || 1.0; // 0..1, how rigidly to enforce
  }

  solve() {
    const delta = this.bodyB.position.sub(this.bodyA.position);
    const currentDist = delta.length();
    if (currentDist < 0.0001) return;

    const diff = (currentDist - this.distance) / currentDist;
    const totalInvMass = this.bodyA.inverseMass + this.bodyB.inverseMass;
    if (totalInvMass === 0) return;

    const correction = delta.mul(diff * this.stiffness / totalInvMass);
    
    if (!this.bodyA.isStatic && !this.bodyA.isSleeping) {
      this.bodyA.position = this.bodyA.position.add(correction.mul(this.bodyA.inverseMass));
    }
    if (!this.bodyB.isStatic && !this.bodyB.isSleeping) {
      this.bodyB.position = this.bodyB.position.sub(correction.mul(this.bodyB.inverseMass));
    }
  }
}

class SpringConstraint {
  // Hooke's law spring between two bodies
  constructor(bodyA, bodyB, opts = {}) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.restLength = opts.restLength || bodyA.position.distance(bodyB.position);
    this.stiffness = opts.stiffness || 50;  // spring constant k
    this.damping = opts.damping || 1;       // damping coefficient
  }

  apply() {
    const delta = this.bodyB.position.sub(this.bodyA.position);
    const currentLen = delta.length();
    if (currentLen < 0.0001) return;

    const direction = delta.normalize();
    const displacement = currentLen - this.restLength;
    
    // Hooke's law: F = -k * x
    const springForce = direction.mul(this.stiffness * displacement);
    
    // Damping: F = -c * v_relative_along_spring
    const relVel = this.bodyB.velocity.sub(this.bodyA.velocity);
    const dampingForce = direction.mul(this.damping * relVel.dot(direction));
    
    const totalForce = springForce.add(dampingForce);
    
    if (!this.bodyA.isStatic) this.bodyA.applyForce(totalForce);
    if (!this.bodyB.isStatic) this.bodyB.applyForce(totalForce.mul(-1));
  }
}

module.exports = { Vec2, Body, World, SpatialHashGrid, DistanceConstraint, SpringConstraint, detectCollision, resolveCollision };
