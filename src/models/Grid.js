/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * Create a grid object.
 *
 * @constructor
 * @param options - object with following keys:
 *  * size (optional) - The number of cells of the grid
 *  * color (optional) - the line color of the grid, like '#cccccc'
 *  * lineWidth (optional) - the width of the lines in the grid
 *  * cellSize (optional) - The length, in meters, of the side of each cell
 */
ROS3D.Grid = function(options) {
  options = options || {};
  var size = options.size || 10;
  var color = options.color || '#cccccc';
  var lineWidth = options.lineWidth || 1;
  var cellSize = options.cellSize || 1;

  // create the mesh
  THREE.Mesh.call(this, new THREE.PlaneGeometry(size*cellSize, size*cellSize, size, size),
      new THREE.MeshBasicMaterial({
        color : color,
        wireframe : true,
        wireframeLinewidth : lineWidth,
        transparent : true
      }));
};
ROS3D.Grid.prototype.__proto__ = THREE.Mesh.prototype;
