/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A Arrow is a THREE object that can be used to display an arrow model.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * origin (optional) - the origin of the arrow
 *   * direction (optional) - the direction vector of the arrow
 *   * length (optional) - the length of the arrow
 *   * headLength (optional) - the head length of the arrow
 *   * shaftDiameter (optional) - the shaft diameter of the arrow
 *   * headDiameter (optional) - the head diameter of the arrow
 *   * material (optional) - the material to use for this arrow
 */
ROS3D.Arrow = function(options) {
  options = options || {};
  var origin = options.origin || new THREE.Vector3(0, 0, 0);
  var direction = options.direction || new THREE.Vector3(1, 0, 0);
  var length = options.length || 1;
  var headLength = options.headLength || 0.2;
  var shaftDiameter = options.shaftDiameter || 0.05;
  var headDiameter = options.headDiameter || 0.1;
  var material = options.material || new THREE.MeshBasicMaterial();

  var shaftLength = length - headLength;

  // create and merge geometry
  var geometry = new THREE.CylinderGeometry(shaftDiameter * 0.5, shaftDiameter * 0.5, shaftLength,
      12, 1);
  var m = new THREE.Matrix4();
  m.setPosition(new THREE.Vector3(0, shaftLength * 0.5, 0));
  geometry.applyMatrix(m);

  // create the head
  var coneGeometry = new THREE.CylinderGeometry(0, headDiameter * 0.5, headLength, 12, 1);
  m.setPosition(new THREE.Vector3(0, shaftLength + (headLength * 0.5), 0));
  coneGeometry.applyMatrix(m);

  // put the arrow together
  geometry.merge(coneGeometry);

  THREE.Mesh.call(this, geometry, material);

  this.position.copy(origin);
  this.setDirection(direction);
};
ROS3D.Arrow.prototype.__proto__ = THREE.Mesh.prototype;

/**
 * Set the direction of this arrow to that of the given vector.
 *
 * @param direction - the direction to set this arrow
 */
ROS3D.Arrow.prototype.setDirection = function(direction) {
  var axis = new THREE.Vector3();
  if(direction.x === 0 && direction.z === 0){
    axis.set(1, 0, 0);
  } else {
    axis.set(0, 1, 0).cross(direction);
  }
  var radians = Math.acos(new THREE.Vector3(0, 1, 0).dot(direction.clone().normalize()));
  this.matrix = new THREE.Matrix4().makeRotationAxis(axis.normalize(), radians);
  this.rotation.setFromRotationMatrix(this.matrix, this.rotation.order);
};

/**
 * Set this arrow to be the given length.
 *
 * @param length - the new length of the arrow
 */
ROS3D.Arrow.prototype.setLength = function(length) {
  this.scale.set(length, length, length);
};

/**
 * Set the color of this arrow to the given hex value.
 *
 * @param hex - the hex value of the color to use
 */
ROS3D.Arrow.prototype.setColor = function(hex) {
  this.material.color.setHex(hex);
};

/*
 * Free memory of elements in this marker.
 */
ROS3D.Arrow.prototype.dispose = function() {
  if (this.geometry !== undefined) {
      this.geometry.dispose();
  }
  if (this.material !== undefined) {
      this.material.dispose();
  }
};
