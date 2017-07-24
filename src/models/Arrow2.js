/**
 * @author Jihoon Lee - lee@magazino.eu
 */

/**
 * A Arrow is a THREE object that can be used to display an arrow model using ArrowHelper
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
ROS3D.Arrow2 = function(options) {
  options = options || {};
  var origin = options.origin || new THREE.Vector3(0, 0, 0);
  var direction = options.direction || new THREE.Vector3(1, 0, 0);
  var length = options.length || 1;
  var headLength = options.headLength || 0.2;
  var shaftDiameter = options.shaftDiameter || 0.05;
  var headDiameter = options.headDiameter || 0.1;
  var material = options.material || new THREE.MeshBasicMaterial();

  THREE.ArrowHelper.call(this, direction, origin, length, 0xff0000);

};

ROS3D.Arrow2.prototype.__proto__ = THREE.ArrowHelper.prototype;

/*
 * Free memory of elements in this object.
 */
ROS3D.Arrow2.prototype.dispose = function() {
  if (this.line !== undefined) {
      this.line.material.dispose();
      this.line.geometry.dispose();
  }
  if (this.cone!== undefined) {
      this.cone.material.dispose();
      this.cone.geometry.dispose();
  }
};

/*
ROS3D.Arrow2.prototype.setLength = function ( length, headLength, headWidth ) {
	if ( headLength === undefined ) {
    headLength = 0.2 * length;
  }
	if ( headWidth === undefined ) {
    headWidth = 0.2 * headLength;
  }

	this.line.scale.set( 1, Math.max( 0, length), 1 );
	this.line.updateMatrix();

	this.cone.scale.set( headWidth, headLength, headWidth );
	this.cone.position.y = length;
	this.cone.updateMatrix();

};
*/
