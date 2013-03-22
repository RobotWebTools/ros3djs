ROS3D.ArrowMarker = function(options) {
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

  var m = new THREE.Matrix4;
  m.setPosition(new THREE.Vector3(0, shaftLength * 0.5, 0));
  geometry.applyMatrix(m);

  var coneGeometry = new THREE.CylinderGeometry(0, headDiameter * 0.5, headLength, 12, 1);

  m.setPosition(new THREE.Vector3(0, shaftLength + (headLength * 0.5), 0));
  coneGeometry.applyMatrix(m);

  THREE.GeometryUtils.merge(geometry, coneGeometry);

  THREE.Mesh.call(this, geometry, material);

  this.position = origin;
  this.setDirection(direction);
};
ROS3D.ArrowMarker.prototype = Object.create(THREE.Mesh.prototype);

ROS3D.ArrowMarker.prototype.setDirection = function(direction) {

  var axis = new THREE.Vector3(0, 1, 0).cross(direction);

  var radians = Math.acos(new THREE.Vector3(0, 1, 0).dot(direction.clone().normalize()));

  this.matrix = new THREE.Matrix4().makeRotationAxis(axis.normalize(), radians);

  this.rotation.setEulerFromRotationMatrix(this.matrix, this.eulerOrder);

};

ROS3D.ArrowMarker.prototype.setLength = function(length) {
  this.scale.set(length, length, length);
};

ROS3D.ArrowMarker.prototype.setColor = function(hex) {
  this.line.material.color.setHex(hex);
  this.cone.material.color.setHex(hex);
};
