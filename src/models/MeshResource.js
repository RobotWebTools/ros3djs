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
 *  * path (optional) - the base path to the associated models that will be loaded
 *  * resource - the resource file name to load
 */
ROS3D.MeshResource = function(options) {
  var that = this;
  var options = options || {};
  var path = options.path || '/';
  var resource = options.resource;

  THREE.Object3D.call(this);

  // check for a trailing '/'
  if (path.substr(path.length - 1) !== '/') {
    this.path += '/';
  }

  var uri = path + resource;
  var fileType = uri.substr(-4).toLowerCase();

  // check the type
  if (uri.substr(-4).toLowerCase() === '.dae') {
    var loader = new THREE.ColladaLoader();
    loader.load(uri, function colladaReady(collada) {
      that.add(collada.scene);
    });
  }
};
ROS3D.MeshResource.prototype.__proto__ = THREE.Object3D.prototype;
