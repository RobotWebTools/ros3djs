/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * Create a grid object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * num_cells (optional) - The number of cells of the grid
 *  * color (optional) - the line color of the grid, like '#cccccc'
 *  * lineWidth (optional) - the width of the lines in the grid
 *  * cellSize (optional) - The length, in meters, of the side of each cell
 */
ROS3D.Grid = function(options) {
  options = options || {};
  var num_cells = options.num_cells || 10;
  var color = options.color || '#cccccc';
  var lineWidth = options.lineWidth || 1;
  var cellSize = options.cellSize || 1;

  THREE.Object3D.call(this);

  var material = new THREE.LineBasicMaterial({
    color: color,
    linewidth: lineWidth
  });

  for (var i = 0; i <= num_cells; ++i) {
    var edge = cellSize * num_cells / 2;
    var position = edge - (i * cellSize);

    const pointsH = [];
    pointsH.push(
      new THREE.Vector3( -edge, position, 0 ),
      new THREE.Vector3( edge, position, 0 )
    );

    const pointsV = [];
    pointsV.push(
      new THREE.Vector3( position, -edge, 0 ),
      new THREE.Vector3( position, edge, 0 )
    );

    const geometryH = new THREE.BufferGeometry().setFromPoints(pointsH);
    const geometryV = new THREE.BufferGeometry().setFromPoints(pointsV);

    this.add(new THREE.Line(geometryH, material));
    this.add(new THREE.Line(geometryV, material));
  }
};

ROS3D.Grid.prototype.__proto__ = THREE.Object3D.prototype;
