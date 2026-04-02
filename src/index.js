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
    this.isTrigger = opts.isTrigger || false; // Triggers detect overlap but don't resolve
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
    } else if (body.shape.type === 'polygon') {
      // Compute AABB of rotated polygon for broadphase cells
      const cos = Math.cos(body.angle || 0);
      const sin = Math.sin(body.angle || 0);
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
      for (const v of body.shape.vertices) {
        const wx = body.position.x + v.x * cos - v.y * sin;
        const wy = body.position.y + v.x * sin + v.y * cos;
        if (wx < minX) minX = wx;
        if (wx > maxX) maxX = wx;
        if (wy < minY) minY = wy;
        if (wy > maxY) maxY = wy;
      }
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
    
    // Event system
    this._listeners = { collision: [], trigger: [] };
    this._activeCollisions = new Set(); // "id1_id2" keys for tracking start/end
  }

  addBody(body) { this.bodies.push(body); return body; }
  removeBody(body) { this.bodies = this.bodies.filter(b => b !== body); }
  
  addConstraint(constraint) { this.constraints.push(constraint); return constraint; }
  removeConstraint(constraint) { this.constraints = this.constraints.filter(c => c !== constraint); }
  
  addSpring(spring) { this.springs.push(spring); return spring; }
  removeSpring(spring) { this.springs = this.springs.filter(s => s !== spring); }

  on(event, callback) {
    if (this._listeners[event]) this._listeners[event].push(callback);
    return this;
  }

  off(event, callback) {
    if (this._listeners[event]) {
      this._listeners[event] = this._listeners[event].filter(cb => cb !== callback);
    }
    return this;
  }

  _emit(event, data) {
    for (const cb of this._listeners[event] || []) {
      cb(data);
    }
  }

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
    const currentCollisions = new Set();
    for (let i = 0; i < this.bodies.length; i++) {
      for (let j = i + 1; j < this.bodies.length; j++) {
        const a = this.bodies[i], b = this.bodies[j];
        if ((a.isSleeping || a.isStatic) && (b.isSleeping || b.isStatic)) continue;
        this._narrowphaseChecks++;
        const collision = detectCollision(a, b);
        if (collision) {
          const key = a.id < b.id ? `${a.id}_${b.id}` : `${b.id}_${a.id}`;
          currentCollisions.add(key);
          
          // Fire collision start if new
          if (!this._activeCollisions.has(key)) {
            this._emit('collision', { bodyA: a, bodyB: b, type: 'start', ...collision });
          }
          this._emit('collision', { bodyA: a, bodyB: b, type: 'active', ...collision });
          
          // Check if either body is a trigger (no physics response)
          if (a.isTrigger || b.isTrigger) {
            this._emit('trigger', { bodyA: a, bodyB: b, ...collision });
          } else {
            resolveCollision(a, b, collision);
          }
        }
      }
    }
    
    // Fire collision end for pairs that stopped colliding
    for (const key of this._activeCollisions) {
      if (!currentCollisions.has(key)) {
        const [idA, idB] = key.split('_').map(Number);
        const bodyA = this.bodies.find(b => b.id === idA);
        const bodyB = this.bodies.find(b => b.id === idB);
        if (bodyA && bodyB) {
          this._emit('collision', { bodyA, bodyB, type: 'end' });
        }
      }
    }
    this._activeCollisions = currentCollisions;
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
  // AABB-polygon: convert AABB to polygon and use SAT
  if (a.shape.type === 'aabb' && b.shape.type === 'polygon') {
    const polyA = aabbToPolyBody(a);
    return polygonPolygon(polyA, b);
  }
  if (a.shape.type === 'polygon' && b.shape.type === 'aabb') {
    const polyB = aabbToPolyBody(b);
    return polygonPolygon(a, polyB);
  }
  // circle-aabb: existing fallback (already handled above if both aabb)
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

// Convert AABB body to a polygon body (for mixed collision detection)
function aabbToPolyBody(body) {
  const hw = body.shape.width / 2, hh = body.shape.height / 2;
  return {
    position: body.position,
    angle: body.angle || 0,
    shape: {
      type: 'polygon',
      vertices: [
        new Vec2(-hw, -hh), new Vec2(hw, -hh),
        new Vec2(hw, hh), new Vec2(-hw, hh)
      ]
    }
  };
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
      
      // Angular friction: friction creates torque at contact point
      a.angularVelocity -= (rA.x * frictionImpulse.y - rA.y * frictionImpulse.x) * a.inverseInertia;
      b.angularVelocity += (rB.x * frictionImpulse.y - rB.y * frictionImpulse.x) * b.inverseInertia;
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

/**
 * Revolute Joint (Hinge/Pin) — constrains two bodies to share a common point
 * Bodies can rotate freely around the anchor point
 */
class RevoluteJoint {
  constructor(bodyA, bodyB, anchor, options = {}) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    // Local anchors: transform world anchor to body-local coordinates
    this.localAnchorA = this._toLocal(bodyA, anchor);
    this.localAnchorB = this._toLocal(bodyB, anchor);
    this.stiffness = options.stiffness || 1.0;
    this.enableLimit = options.enableLimit || false;
    this.lowerAngle = options.lowerAngle || -Math.PI;
    this.upperAngle = options.upperAngle || Math.PI;
    this.enableMotor = options.enableMotor || false;
    this.motorSpeed = options.motorSpeed || 0;
    this.maxMotorTorque = options.maxMotorTorque || 0;
  }

  _toLocal(body, worldPoint) {
    const d = worldPoint.sub(body.position);
    const cos = Math.cos(-body.angle), sin = Math.sin(-body.angle);
    return new Vec2(d.x * cos - d.y * sin, d.x * sin + d.y * cos);
  }

  _toWorld(body, localPoint) {
    const cos = Math.cos(body.angle), sin = Math.sin(body.angle);
    return body.position.add(new Vec2(
      localPoint.x * cos - localPoint.y * sin,
      localPoint.x * sin + localPoint.y * cos
    ));
  }

  solve() {
    // Position constraint: anchor points must coincide
    const worldA = this._toWorld(this.bodyA, this.localAnchorA);
    const worldB = this._toWorld(this.bodyB, this.localAnchorB);
    
    const error = worldB.sub(worldA);
    const correction = error.mul(this.stiffness * 0.5);
    
    const totalInvMass = this.bodyA.inverseMass + this.bodyB.inverseMass;
    if (totalInvMass === 0) return;
    
    if (!this.bodyA.isStatic) {
      this.bodyA.position = this.bodyA.position.add(correction.mul(this.bodyA.inverseMass / totalInvMass));
    }
    if (!this.bodyB.isStatic) {
      this.bodyB.position = this.bodyB.position.sub(correction.mul(this.bodyB.inverseMass / totalInvMass));
    }
    
    // Angle limits
    if (this.enableLimit) {
      const relAngle = this.bodyB.angle - this.bodyA.angle;
      if (relAngle < this.lowerAngle) {
        const diff = this.lowerAngle - relAngle;
        if (!this.bodyA.isStatic) this.bodyA.angle -= diff * 0.5;
        if (!this.bodyB.isStatic) this.bodyB.angle += diff * 0.5;
      } else if (relAngle > this.upperAngle) {
        const diff = relAngle - this.upperAngle;
        if (!this.bodyA.isStatic) this.bodyA.angle += diff * 0.5;
        if (!this.bodyB.isStatic) this.bodyB.angle -= diff * 0.5;
      }
    }
    
    // Motor
    if (this.enableMotor) {
      const relAngVel = this.bodyB.angularVelocity - this.bodyA.angularVelocity;
      const torque = Math.max(-this.maxMotorTorque, Math.min(this.maxMotorTorque, 
        (this.motorSpeed - relAngVel) * 0.5));
      if (!this.bodyA.isStatic) this.bodyA.angularVelocity -= torque * this.bodyA.inverseInertia;
      if (!this.bodyB.isStatic) this.bodyB.angularVelocity += torque * this.bodyB.inverseInertia;
    }
  }
}

/**
 * Prismatic Joint (Slider) — constrains two bodies to move along a fixed axis
 * Bodies can only translate along the axis, rotation is constrained
 */
class PrismaticJoint {
  constructor(bodyA, bodyB, axis, options = {}) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.axis = axis.normalize(); // Movement axis (world space)
    this.stiffness = options.stiffness || 1.0;
    this.enableLimit = options.enableLimit || false;
    this.lowerLimit = options.lowerLimit || -Infinity;
    this.upperLimit = options.upperLimit || Infinity;
    // Store initial relative position projected onto axis
    this.initialOffset = bodyB.position.sub(bodyA.position).dot(this.axis);
  }

  solve() {
    const d = this.bodyB.position.sub(this.bodyA.position);
    const axisProjection = d.dot(this.axis);
    const perpendicular = d.sub(this.axis.mul(axisProjection));
    
    // Constrain perpendicular movement (keep on axis)
    const correction = perpendicular.mul(this.stiffness * 0.5);
    const totalInvMass = this.bodyA.inverseMass + this.bodyB.inverseMass;
    if (totalInvMass > 0) {
      if (!this.bodyA.isStatic) {
        this.bodyA.position = this.bodyA.position.add(correction.mul(this.bodyA.inverseMass / totalInvMass));
      }
      if (!this.bodyB.isStatic) {
        this.bodyB.position = this.bodyB.position.sub(correction.mul(this.bodyB.inverseMass / totalInvMass));
      }
    }
    
    // Translation limits along axis
    if (this.enableLimit) {
      const translation = axisProjection - this.initialOffset;
      if (translation < this.lowerLimit) {
        const fix = this.axis.mul((this.lowerLimit - translation) * 0.5);
        if (!this.bodyA.isStatic) this.bodyA.position = this.bodyA.position.sub(fix);
        if (!this.bodyB.isStatic) this.bodyB.position = this.bodyB.position.add(fix);
      } else if (translation > this.upperLimit) {
        const fix = this.axis.mul((translation - this.upperLimit) * 0.5);
        if (!this.bodyA.isStatic) this.bodyA.position = this.bodyA.position.add(fix);
        if (!this.bodyB.isStatic) this.bodyB.position = this.bodyB.position.sub(fix);
      }
    }
    
    // Constrain relative rotation (keep angles aligned)
    const angleDiff = this.bodyB.angle - this.bodyA.angle;
    if (Math.abs(angleDiff) > 0.001) {
      if (!this.bodyA.isStatic) this.bodyA.angle += angleDiff * 0.25;
      if (!this.bodyB.isStatic) this.bodyB.angle -= angleDiff * 0.25;
    }
  }
}

/**
 * Rope Constraint — a chain of distance constraints between multiple bodies
 * Creates a flexible rope/chain by linking N bodies with distance constraints.
 * Supports variable stiffness, iteration count, and optional anchor pinning.
 */
class RopeConstraint {
  /**
   * @param {Body[]} bodies - Ordered list of bodies forming the rope
   * @param {object} opts
   * @param {number} opts.segmentLength - Distance between adjacent bodies (default: auto from positions)
   * @param {number} opts.stiffness - How rigidly constraints are enforced (0..1, default: 1)
   * @param {number} opts.iterations - Solver iterations per step (default: 10, higher = stiffer)
   * @param {boolean} opts.pinFirst - Pin the first body (make static-like for constraint solving)
   * @param {boolean} opts.pinLast - Pin the last body
   */
  constructor(bodies, opts = {}) {
    if (!bodies || bodies.length < 2) {
      throw new Error('RopeConstraint requires at least 2 bodies');
    }
    this.bodies = bodies;
    this.stiffness = opts.stiffness !== undefined ? opts.stiffness : 1.0;
    this.iterations = opts.iterations || 10;
    this.pinFirst = opts.pinFirst || false;
    this.pinLast = opts.pinLast || false;

    // Build internal distance constraints between adjacent bodies
    this.constraints = [];
    for (let i = 0; i < bodies.length - 1; i++) {
      const segLen = opts.segmentLength || bodies[i].position.distance(bodies[i + 1].position);
      this.constraints.push(new DistanceConstraint(bodies[i], bodies[i + 1], {
        distance: segLen,
        stiffness: this.stiffness
      }));
    }
  }

  get segmentCount() { return this.constraints.length; }
  get length() {
    return this.constraints.reduce((sum, c) => sum + c.distance, 0);
  }

  solve() {
    // Save pinned positions
    const firstPos = this.pinFirst ? this.bodies[0].position : null;
    const lastPos = this.pinLast ? this.bodies[this.bodies.length - 1].position : null;

    for (let iter = 0; iter < this.iterations; iter++) {
      // Alternate solve direction for stability
      if (iter % 2 === 0) {
        for (let i = 0; i < this.constraints.length; i++) {
          this.constraints[i].solve();
        }
      } else {
        for (let i = this.constraints.length - 1; i >= 0; i--) {
          this.constraints[i].solve();
        }
      }

      // Re-pin endpoints after each iteration
      if (firstPos) this.bodies[0].position = firstPos;
      if (lastPos) this.bodies[this.bodies.length - 1].position = lastPos;
    }
  }

  /**
   * Get positions of all rope bodies (for rendering)
   */
  getPoints() {
    return this.bodies.map(b => ({ x: b.position.x, y: b.position.y }));
  }

  /**
   * Apply gravity to all non-pinned rope bodies
   */
  applyGravity(gravity) {
    for (let i = 0; i < this.bodies.length; i++) {
      if (i === 0 && this.pinFirst) continue;
      if (i === this.bodies.length - 1 && this.pinLast) continue;
      this.bodies[i].applyForce(gravity.mul(this.bodies[i].mass));
    }
  }
}

module.exports = { Vec2, Body, World, SpatialHashGrid, DistanceConstraint, SpringConstraint, RevoluteJoint, PrismaticJoint, RopeConstraint, raycast, sweepTest, detectCollision, resolveCollision };

// === Continuous Collision Detection (CCD) ===

/**
 * Sweep test: check if a moving circle will collide with a static circle
 * during the timestep [0, dt].
 * Uses swept sphere (Minkowski sum) approach.
 * 
 * Returns: { time, point, normal } or null
 *   time: fraction [0,1] of dt when collision first occurs
 */
function sweepTest(movingBody, staticBody, dt) {
  if (movingBody.shape.type !== 'circle' || staticBody.shape.type !== 'circle') {
    return null; // Only circle-circle CCD for now
  }
  
  const pos = movingBody.position;
  const vel = movingBody.velocity;
  const r = movingBody.shape.radius + staticBody.shape.radius; // Minkowski sum radius
  const center = staticBody.position;
  
  // Ray from pos in direction vel*dt, checking intersection with sphere of radius r at center
  const motion = vel.mul(dt);
  const oc = pos.sub(center);
  
  const a = motion.dot(motion);
  if (a < 0.0001) return null; // Not moving
  
  const b = 2 * oc.dot(motion);
  const c = oc.dot(oc) - r * r;
  
  // Check if already overlapping
  if (c < 0) {
    return { time: 0, point: pos, normal: oc.length() > 0 ? oc.mul(1/oc.length()) : new Vec2(0, 1) };
  }
  
  const discriminant = b * b - 4 * a * c;
  if (discriminant < 0) return null;
  
  const sqrtD = Math.sqrt(discriminant);
  const t = (-b - sqrtD) / (2 * a);
  
  if (t < 0 || t > 1) return null; // Collision outside [0, dt]
  
  const hitPos = pos.add(motion.mul(t));
  const normal = hitPos.sub(center);
  const normalLen = normal.length();
  
  return {
    time: t,
    point: hitPos,
    normal: normalLen > 0 ? normal.mul(1 / normalLen) : new Vec2(0, 1),
  };
}

// === Ray Casting ===

/**
 * Cast a ray from origin in direction, testing against a list of bodies.
 * Returns the first hit: { body, point, normal, distance } or null.
 */
function raycast(origin, direction, bodies, maxDist = Infinity) {
  const dir = direction.length() > 0 ? direction.mul(1 / direction.length()) : direction;
  let closest = null;
  let closestDist = maxDist;

  for (const body of bodies) {
    const hit = raycastBody(origin, dir, body, closestDist);
    if (hit && hit.distance < closestDist) {
      closest = { ...hit, body };
      closestDist = hit.distance;
    }
  }

  return closest;
}

/**
 * Cast ray against a single body. Returns { point, normal, distance } or null.
 */
function raycastBody(origin, dir, body, maxDist) {
  if (body.shape.type === 'circle') {
    return raycastCircle(origin, dir, body.position, body.shape.radius, maxDist);
  }
  if (body.shape.type === 'aabb') {
    const hw = body.shape.width / 2, hh = body.shape.height / 2;
    return raycastAABB(origin, dir, 
      new Vec2(body.position.x - hw, body.position.y - hh),
      new Vec2(body.position.x + hw, body.position.y + hh), maxDist);
  }
  if (body.shape.type === 'polygon') {
    return raycastPolygon(origin, dir, body, maxDist);
  }
  return null;
}

function raycastCircle(origin, dir, center, radius, maxDist) {
  const oc = origin.sub(center);
  const a = dir.dot(dir);
  const b = 2 * oc.dot(dir);
  const c = oc.dot(oc) - radius * radius;
  const discriminant = b * b - 4 * a * c;
  
  if (discriminant < 0) return null;
  
  const sqrtD = Math.sqrt(discriminant);
  let t = (-b - sqrtD) / (2 * a);
  if (t < 0) t = (-b + sqrtD) / (2 * a);
  if (t < 0 || t > maxDist) return null;
  
  const point = origin.add(dir.mul(t));
  const normal = point.sub(center).mul(1 / radius);
  
  return { point, normal, distance: t };
}

function raycastAABB(origin, dir, min, max, maxDist) {
  let tmin = -Infinity, tmax = Infinity;
  let normalAxis = 0; // 0=x, 1=y
  let normalSign = 1;
  
  // X axis
  if (Math.abs(dir.x) > 0.0001) {
    let t1 = (min.x - origin.x) / dir.x;
    let t2 = (max.x - origin.x) / dir.x;
    if (t1 > t2) [t1, t2] = [t2, t1];
    if (t1 > tmin) { tmin = t1; normalAxis = 0; normalSign = dir.x > 0 ? -1 : 1; }
    if (t2 < tmax) tmax = t2;
  } else if (origin.x < min.x || origin.x > max.x) return null;
  
  // Y axis
  if (Math.abs(dir.y) > 0.0001) {
    let t1 = (min.y - origin.y) / dir.y;
    let t2 = (max.y - origin.y) / dir.y;
    if (t1 > t2) [t1, t2] = [t2, t1];
    if (t1 > tmin) { tmin = t1; normalAxis = 1; normalSign = dir.y > 0 ? -1 : 1; }
    if (t2 < tmax) tmax = t2;
  } else if (origin.y < min.y || origin.y > max.y) return null;
  
  if (tmin > tmax || tmax < 0 || tmin > maxDist) return null;
  
  const t = tmin >= 0 ? tmin : tmax;
  if (t < 0 || t > maxDist) return null;
  
  const point = origin.add(dir.mul(t));
  const normal = normalAxis === 0 ? new Vec2(normalSign, 0) : new Vec2(0, normalSign);
  
  return { point, normal, distance: t };
}

function raycastPolygon(origin, dir, body, maxDist) {
  const verts = body.shape.vertices.map(v => {
    const cos = Math.cos(body.angle || 0);
    const sin = Math.sin(body.angle || 0);
    return new Vec2(
      body.position.x + v.x * cos - v.y * sin,
      body.position.y + v.x * sin + v.y * cos
    );
  });
  
  let closestT = maxDist;
  let hitNormal = null;
  
  for (let i = 0; i < verts.length; i++) {
    const j = (i + 1) % verts.length;
    const edgeStart = verts[i];
    const edgeDir = verts[j].sub(verts[i]);
    
    // Ray-segment intersection
    const denom = dir.x * edgeDir.y - dir.y * edgeDir.x;
    if (Math.abs(denom) < 0.0001) continue;
    
    const t = ((edgeStart.x - origin.x) * edgeDir.y - (edgeStart.y - origin.y) * edgeDir.x) / denom;
    const u = ((edgeStart.x - origin.x) * dir.y - (edgeStart.y - origin.y) * dir.x) / denom;
    
    if (t >= 0 && t < closestT && u >= 0 && u <= 1) {
      closestT = t;
      // Normal perpendicular to edge, facing outward
      hitNormal = new Vec2(-edgeDir.y, edgeDir.x);
      const len = hitNormal.length();
      if (len > 0) hitNormal = hitNormal.mul(1 / len);
      // Ensure normal faces the ray origin
      if (hitNormal.dot(dir) > 0) hitNormal = hitNormal.mul(-1);
    }
  }
  
  if (closestT >= maxDist) return null;
  
  return { point: origin.add(dir.mul(closestT)), normal: hitNormal, distance: closestT };
}
