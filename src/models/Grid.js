/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * Create a grid object.
 *
 * @constructor
 * @param options - object with following keys:
 *  * size (optional) - the size of the grid
 *  * color (optional) - the line color of the grid, like '#cccccc'
 *  * lineWidth (optional) - the width of the lines in the grid
 */
ROS3D.Grid = function(options) {
  options = options || {};
  var size = options.size || 50;
  var color = options.color || '#cccccc';
  var lineWidth = options.lineWidth || 1;

  // create the mesh
  THREE.Mesh.call(this, new THREE.PlaneGeometry(size, size, size, size),
      new THREE.MeshBasicMaterial({
        color : color,
        wireframe : true,
        wireframeLinewidth : lineWidth,
        transparent : true
      }));
};
ROS3D.Grid.prototype.__proto__ = THREE.Mesh.prototype;
