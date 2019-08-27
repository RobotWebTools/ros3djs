/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A MeshResource is an THREE object that will load from a external mesh file. Currently loads
 * Collada files.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * path (optional) - the base path to the associated models that will be loaded
 *  * resource - the resource file name to load
 *  * material (optional) - the material to use for the object
 *  * warnings (optional) - if warnings should be printed
 */
ROS3D.MeshResource = function(options) {
  THREE.Object3D.call(this);
  var that = this;
  options = options || {};
  var path = options.path || '/';
  var resource = options.resource;
  var material = options.material || null;
  this.warnings = options.warnings;


  // check for a trailing '/'
  if (path.substr(path.length - 1) !== '/') {
    path += '/';
  }

  var uri = path + resource;
  var fileType = uri.substr(-3).toLowerCase();

  // check the type
  var loaderFunc = ROS3D.MeshLoader.loaders[fileType];
  if (loaderFunc) {
    loaderFunc(this, uri, options);
  } else {
    console.warn('Unsupported loader for file type: \'' + fileType + '\'');
  }
};
ROS3D.MeshResource.prototype.__proto__ = THREE.Object3D.prototype;
