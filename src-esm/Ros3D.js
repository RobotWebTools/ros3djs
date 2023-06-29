import THREE from '../shims/three/core.js';

/**
 * @fileOverview
 * @author Russell Toris - rctoris@wpi.edu
 * @author David Gossow - dgossow@willowgarage.com
 */

var ROS3D = ROS3D || {
  /**
   * @default
   * @description Library version
   */
  REVISION : '1.1.0'
};

// Marker types
export var MARKER_ARROW = 0;
export var MARKER_CUBE = 1;
export var MARKER_SPHERE = 2;
export var MARKER_CYLINDER = 3;
export var MARKER_LINE_STRIP = 4;
export var MARKER_LINE_LIST = 5;
export var MARKER_CUBE_LIST = 6;
export var MARKER_SPHERE_LIST = 7;
export var MARKER_POINTS = 8;
export var MARKER_TEXT_VIEW_FACING = 9;
export var MARKER_MESH_RESOURCE = 10;
export var MARKER_TRIANGLE_LIST = 11;

// Interactive marker feedback types
export var INTERACTIVE_MARKER_KEEP_ALIVE = 0;
export var INTERACTIVE_MARKER_POSE_UPDATE = 1;
export var INTERACTIVE_MARKER_MENU_SELECT = 2;
export var INTERACTIVE_MARKER_BUTTON_CLICK = 3;
export var INTERACTIVE_MARKER_MOUSE_DOWN = 4;
export var INTERACTIVE_MARKER_MOUSE_UP = 5;

// Interactive marker control types
export var INTERACTIVE_MARKER_NONE = 0;
export var INTERACTIVE_MARKER_MENU = 1;
export var INTERACTIVE_MARKER_BUTTON = 2;
export var INTERACTIVE_MARKER_MOVE_AXIS = 3;
export var INTERACTIVE_MARKER_MOVE_PLANE = 4;
export var INTERACTIVE_MARKER_ROTATE_AXIS = 5;
export var INTERACTIVE_MARKER_MOVE_ROTATE = 6;
export var INTERACTIVE_MARKER_MOVE_3D = 7;
export var INTERACTIVE_MARKER_ROTATE_3D = 8;
export var INTERACTIVE_MARKER_MOVE_ROTATE_3D = 9;

// Interactive marker rotation behavior
export var INTERACTIVE_MARKER_INHERIT = 0;
export var INTERACTIVE_MARKER_FIXED = 1;
export var INTERACTIVE_MARKER_VIEW_FACING = 2;

/**
 * @function makeColorMaterial
 * @description Create a THREE material based on the given RGBA values.
 *
 * @param r - the red value
 * @param g - the green value
 * @param b - the blue value
 * @param a - the alpha value
 * @returns the THREE material
 */
export var makeColorMaterial = function(r, g, b, a) {
  var color = new THREE.Color();
  color.setRGB(r, g, b);
  if (a <= 0.99) {
    return new THREE.MeshBasicMaterial({
      color : color.getHex(),
      opacity : a + 0.1,
      transparent : true,
      depthWrite : true,
      blendSrc : THREE.SrcAlphaFactor,
      blendDst : THREE.OneMinusSrcAlphaFactor,
      blendEquation : THREE.ReverseSubtractEquation,
      blending : THREE.NormalBlending
    });
  } else {
    return new THREE.MeshPhongMaterial({
      color : color.getHex(),
      opacity : a,
      blending : THREE.NormalBlending
    });
  }
};

/**
 * @function intersectPlane
 * @description Return the intersection between the mouseray and the plane.
 *
 * @param mouseRay - the mouse ray
 * @param planeOrigin - the origin of the plane
 * @param planeNormal - the normal of the plane
 * @returns the intersection point
 */
export var intersectPlane = function(mouseRay, planeOrigin, planeNormal) {
  var vector = new THREE.Vector3();
  var intersectPoint = new THREE.Vector3();
  vector.subVectors(planeOrigin, mouseRay.origin);
  var dot = mouseRay.direction.dot(planeNormal);

  // bail if ray and plane are parallel
  if (Math.abs(dot) < mouseRay.precision) {
    return undefined;
  }

  // calc distance to plane
  var scalar = planeNormal.dot(vector) / dot;

  intersectPoint.addVectors(mouseRay.origin, mouseRay.direction.clone().multiplyScalar(scalar));
  return intersectPoint;
};

/**
 * @function findClosestPoint
 * @description Find the closest point on targetRay to any point on mouseRay. Math taken from
 * http://paulbourke.net/geometry/lineline3d/
 *
 * @param targetRay - the target ray to use
 * @param mouseRay - the mouse ray
 * @param the closest point between the two rays
 */
export var findClosestPoint = function(targetRay, mouseRay) {
  var v13 = new THREE.Vector3();
  v13.subVectors(targetRay.origin, mouseRay.origin);
  var v43 = mouseRay.direction.clone();
  var v21 = targetRay.direction.clone();
  var d1343 = v13.dot(v43);
  var d4321 = v43.dot(v21);
  var d1321 = v13.dot(v21);
  var d4343 = v43.dot(v43);
  var d2121 = v21.dot(v21);

  var denom = d2121 * d4343 - d4321 * d4321;
  // check within a delta
  if (Math.abs(denom) <= 0.0001) {
    return undefined;
  }
  var numer = d1343 * d4321 - d1321 * d4343;

  var mua = numer / denom;
  return mua;
};

/**
 * @function closestAxisPoint
 * @description Find the closest point between the axis and the mouse.
 *
 * @param axisRay - the ray from the axis
 * @param camera - the camera to project from
 * @param mousePos - the mouse position
 * @returns the closest axis point
 */
export var closestAxisPoint = function(axisRay, camera, mousePos) {
  // project axis onto screen
  var o = axisRay.origin.clone();
  o.project(camera);
  var o2 = axisRay.direction.clone().add(axisRay.origin);
  o2.project(camera);

  // d is the axis vector in screen space (d = o2-o)
  var d = o2.clone().sub(o);

  // t is the 2d ray param of perpendicular projection of mousePos onto o
  var tmp = new THREE.Vector2();
  // (t = (mousePos - o) * d / (d*d))
  var t = tmp.subVectors(mousePos, o).dot(d) / d.dot(d);

  // mp is the final 2d-projected mouse pos (mp = o + d*t)
  var mp = new THREE.Vector2();
  mp.addVectors(o, d.clone().multiplyScalar(t));

  // go back to 3d by shooting a ray
  var vector = new THREE.Vector3(mp.x, mp.y, 0.5);
  vector.unproject(camera);
  var mpRay = new THREE.Ray(camera.position, vector.sub(camera.position).normalize());

  return findClosestPoint(axisRay, mpRay);
};
