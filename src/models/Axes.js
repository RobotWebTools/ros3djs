/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * An Axes object can be used to display the axis of a particular coordinate frame.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * shaftRadius (optional) - the radius of the shaft to render
 *   * headRadius (optional) - the radius of the head to render
 *   * headLength (optional) - the length of the head to render
 *   * scale (optional) - the scale of the frame (defaults to 1.0)
 *   * lineType (optional) - the line type for the axes. Supported line types:
 *                           'dashed' and 'full'.
 *   * lineDashLength (optional) - the length of the dashes, relative to the length of the axis.
 *                                 Maximum value is 1, which means the dash length is
 *                                 equal to the length of the axis. Parameter only applies when
 *                                 lineType is set to dashed.
 */
ROS3D.Axes = function(options) {
  THREE.Object3D.call(this);
  var that = this;
  options = options || {};
  var shaftRadius = options.shaftRadius || 0.008;
  var headRadius = options.headRadius || 0.023;
  var headLength = options.headLength || 0.1;
  var scaleArg = options.scale || 1.0;
  var lineType = options.lineType || 'full';
  var lineDashLength = options.lineDashLength || 0.1;


  this.scale.set(scaleArg, scaleArg, scaleArg);

  // create the cylinders for the objects
  this.lineGeom = new THREE.CylinderGeometry(shaftRadius, shaftRadius, 1.0 - headLength);
  this.headGeom = new THREE.CylinderGeometry(0, headRadius, headLength);

  /**
   * Adds an axis marker to this axes object.
   *
   * @param axis - the 3D vector representing the axis to add
   */
  function addAxis(axis) {
    // set the color of the axis
    var color = new THREE.Color();
    color.setRGB(axis.x, axis.y, axis.z);
    var material = new THREE.MeshBasicMaterial({
      color : color.getHex()
    });

    // setup the rotation information
    var rotAxis = new THREE.Vector3();
    rotAxis.crossVectors(axis, new THREE.Vector3(0, -1, 0));
    var rot = new THREE.Quaternion();
    rot.setFromAxisAngle(rotAxis, 0.5 * Math.PI);

    // create the arrow
    var arrow = new THREE.Mesh(that.headGeom, material);
    arrow.position.copy(axis);
    arrow.position.multiplyScalar(0.95);
    arrow.quaternion.copy(rot);
    arrow.updateMatrix();
    that.add(arrow);

    // create the line
    var line;
    if (lineType === 'dashed') {
      var l = lineDashLength;
      for (var i = 0; (l / 2 + 3 * l * i + l / 2) <= 1; ++i) {
        var geom = new THREE.CylinderGeometry(shaftRadius, shaftRadius, l);
        line = new THREE.Mesh(geom, material);
        line.position.copy(axis);
        // Make spacing between dashes equal to 1.5 times the dash length.
        line.position.multiplyScalar(l / 2 + 3 * l * i);
        line.quaternion.copy(rot);
        line.updateMatrix();
        that.add(line);
      }
    } else if (lineType === 'full') {
      line = new THREE.Mesh(that.lineGeom, material);
      line.position.copy(axis);
      line.position.multiplyScalar(0.45);
      line.quaternion.copy(rot);
      line.updateMatrix();
      that.add(line);
    } else {
      console.warn('[ROS3D.Axes]: Unsupported line type. Not drawing any axes.');
    }
  }

  // add the three markers to the axes
  addAxis(new THREE.Vector3(1, 0, 0));
  addAxis(new THREE.Vector3(0, 1, 0));
  addAxis(new THREE.Vector3(0, 0, 1));
};
ROS3D.Axes.prototype.__proto__ = THREE.Object3D.prototype;
