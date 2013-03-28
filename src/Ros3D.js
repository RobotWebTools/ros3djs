/**
* @author Russell Toris - rctoris@wpi.edu
*/

var ROS3D = ROS3D || {
  REVISION : '1'
};

// Marker types
ROS3D.MARKER_ARROW = 0;
ROS3D.MARKER_CUBE = 1;
ROS3D.MARKER_SPHERE = 2;
ROS3D.MARKER_CYLINDER = 3;
ROS3D.MARKER_LINE_STRIP = 4;
ROS3D.MARKER_LINE_LIST = 5;
ROS3D.MARKER_CUBE_LIST = 6;
ROS3D.MARKER_SPHERE_LIST = 7;
ROS3D.MARKER_POINTS = 8;
ROS3D.MARKER_TEXT_VIEW_FACING = 9;
ROS3D.MARKER_MESH_RESOURCE = 10;
ROS3D.MARKER_TRIANGLE_LIST = 11;

// Interactive marker feedback types
ROS3D.INTERACTIVE_MARKER_KEEP_ALIVE = 0;
ROS3D.INTERACTIVE_MARKER_POSE_UPDATE = 1;
ROS3D.INTERACTIVE_MARKER_MENU_SELECT = 2;
ROS3D.INTERACTIVE_MARKER_BUTTON_CLICK = 3;
ROS3D.INTERACTIVE_MARKER_MOUSE_DOWN = 4;
ROS3D.INTERACTIVE_MARKER_MOUSE_UP = 5;

/**
 * Create a THREE material based on the given RGBA values.
 * 
 * @param r - the red value
 * @param g - the green value
 * @param b - the blue value
 * @param a - the alpha value
 * @returns the THREE material
 */
ROS3D.makeColorMaterial = function(r, g, b, a) {
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
    return new THREE.MeshLambertMaterial({
      color : color.getHex(),
      opacity : a,
      blending : THREE.NormalBlending
    });
  }
};
