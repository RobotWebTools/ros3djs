/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An OccupancyGrid can convert a ROS occupancy grid message into a THREE object.
 *
 * @constructor
 * @param options - object with following keys:
 *   * message - the occupancy grid message
 */
ROS3D.OccupancyGrid = function(options) {
  var options = options || {};
  var message = options.message;

  // create the geometry
  var width = message.info.width;
  var height = message.info.height;
  var geom = new THREE.PlaneGeometry(width, height);

  // create the color material
  var dataColor = new Uint8Array(width * height * 3);
  for ( var row = 0; row < height; row++) {
    for ( var col = 0; col < width; col++) {
      // determine the index into the map data
      var mapI = col + ((height - row - 1) * width);
      // determine the value
      var data = message.data[mapI];
      if (data === 100) {
        var val = 0;
      } else if (data === 0) {
        var val = 255;
      } else {
        var val = 127;
      }

      // determine the index into the image data array
      var i = (col + (row * width)) * 3;
      // r
      dataColor[i] = val;
      // g
      dataColor[++i] = val;
      // b
      dataColor[++i] = val;
    }
  }
  var texture = new THREE.DataTexture(dataColor, width, height, THREE.RGBFormat);
  texture.needsUpdate = true;
  var material = new THREE.MeshBasicMaterial({
    map : texture
  });
  material.side = THREE.DoubleSide;

  // create the mesh
  THREE.Mesh.call(this, geom, material);
  this.scale.x = message.info.resolution;
  this.scale.y = message.info.resolution;
};
ROS3D.OccupancyGrid.prototype.__proto__ = THREE.Mesh.prototype;
