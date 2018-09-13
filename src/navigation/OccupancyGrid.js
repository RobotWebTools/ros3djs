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
  var color = options.color || {r:255,g:255,b:255};
  var opacity = options.opacity || 1.0;

  // create the geometry
  var width = message.info.width;
  var height = message.info.height;
  var geom = new THREE.PlaneBufferGeometry(width, height);

  // create the color material
  var imageData = new Uint8Array(width * height * 3);
  for ( var row = 0; row < height; row++) {
    for ( var col = 0; col < width; col++) {
      // determine the index into the map data
      var mapI = col + ((height - row - 1) * width);
      // determine the value
      var data = message.data[mapI];
      var val;
      if (data === 100) {
        val = 0;
      } else if (data === 0) {
        val = 255;
      } else {
        val = 127;
      }

      // determine the index into the image data array
      var i = (col + (row * width)) * 3;
      // r
      imageData[i] = (val * color.r) / 255;
      // g
      imageData[++i] = (val * color.g) / 255;
      // b
      imageData[++i] = (val * color.b) / 255;
    }
  }

  var texture = new THREE.DataTexture(imageData, width, height, THREE.RGBFormat);
  texture.flipY = true;
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;
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
  this.quaternion.copy(new THREE.Quaternion(
      message.info.origin.orientation.x,
      message.info.origin.orientation.y,
      message.info.origin.orientation.z,
      message.info.origin.orientation.w
  ));
  this.position.x = (width * message.info.resolution) / 2 + message.info.origin.position.x;
  this.position.y = (height * message.info.resolution) / 2 + message.info.origin.position.y;
  this.position.z = message.info.origin.position.z;
  this.scale.x = message.info.resolution;
  this.scale.y = message.info.resolution;
};
ROS3D.OccupancyGrid.prototype.__proto__ = THREE.Mesh.prototype;
