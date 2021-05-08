/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An OccupancyGrid can convert a ROS occupancy grid message into a THREE object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * message - the occupancy grid message
 *   * color (optional) - color of the visualized grid
 *   * opacity (optional) - opacity of the visualized grid (0.0 == fully transparent, 1.0 == opaque)
 */
ROS3D.OccupancyGrid = function(options) {
  options = options || {};
  var message = options.message;
  var opacity = options.opacity || 1.0;
  var color = options.color || {r:255,g:255,b:255,a:255};

  // create the geometry
  var info = message.info;
  var origin = info.origin;
  var width = info.width;
  var height = info.height;
  var geom = new THREE.PlaneBufferGeometry(width, height);

  // create the color material
  var imageData = new Uint8Array(width * height * 4);
  var texture = new THREE.DataTexture(imageData, width, height, THREE.RGBAFormat);
  texture.flipY = true;
  texture.minFilter = THREE.NearestFilter;
  texture.magFilter = THREE.NearestFilter;
  texture.needsUpdate = true;

  var material = new THREE.MeshBasicMaterial({
    map : texture,
    transparent : opacity < 1.0,
    opacity : opacity
  });
  material.side = THREE.DoubleSide;

  // create the mesh
  THREE.Mesh.call(this, geom, material);
  // move the map so the corner is at X, Y and correct orientation (informations from message.info)

  // assign options to this for subclasses
  Object.assign(this, options);

  this.quaternion.copy(new THREE.Quaternion(
      origin.orientation.x,
      origin.orientation.y,
      origin.orientation.z,
      origin.orientation.w
  ));
  this.position.x = (width * info.resolution) / 2 + origin.position.x;
  this.position.y = (height * info.resolution) / 2 + origin.position.y;
  this.position.z = origin.position.z;
  this.scale.x = info.resolution;
  this.scale.y = info.resolution;

  var data = message.data;
  // update the texture (after the the super call and this are accessible)
  this.color = color;
  this.material = material;
  this.texture = texture;

  for ( var row = 0; row < height; row++) {
    for ( var col = 0; col < width; col++) {

      // determine the index into the map data
      var invRow = (height - row - 1);
      var mapI = col + (invRow * width);
      // determine the value
      var val = this.getValue(mapI, invRow, col, data);

      // determine the color
      var color = this.getColor(mapI, invRow, col, val);

      // determine the index into the image data array
      var i = (col + (row * width)) * 4;

      // copy the color
      imageData.set(color, i);
    }
  }

  texture.needsUpdate = true;

};

ROS3D.OccupancyGrid.prototype.dispose = function() {
  this.material.dispose();
  this.texture.dispose();
};

/**
 * Returns the value for a given grid cell
 * @param {int} index the current index of the cell
 * @param {int} row the row of the cell
 * @param {int} col the column of the cell
 * @param {object} data the data buffer
 */
ROS3D.OccupancyGrid.prototype.getValue = function(index, row, col, data) {
  return data[index];
};

/**
 * Returns a color value given parameters of the position in the grid; the default implementation
 * scales the default color value by the grid value. Subclasses can extend this functionality
 * (e.g. lookup a color in a color map).
 * @param {int} index the current index of the cell
 * @param {int} row the row of the cell
 * @param {int} col the column of the cell
 * @param {float} value the value of the cell
 * @returns r,g,b,a array of values from 0 to 255 representing the color values for each channel
 */
ROS3D.OccupancyGrid.prototype.getColor = function(index, row, col, value) {
  return [
    (value * this.color.r) / 255,
    (value * this.color.g) / 255,
    (value * this.color.b) / 255,
    255
  ];
};

ROS3D.OccupancyGrid.prototype.__proto__ = THREE.Mesh.prototype;
