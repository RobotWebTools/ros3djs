/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 */

/**
 * A mesh load can be used to load a given 3D mesh into a THREE object.
 * 
 * @constructor
 * @param options - object with following keys:
 *  * path - the base path to the associated Collada models that will be loaded
 */
ROS3D.MeshLoader = function(options) {
  var options = options || {};
  this.path = options.path || '';

  // check for a trailing '/'
  if (this.path.substr(this.path.length - 1) !== '/') {
    this.path += '/';
  }
};

/**
 * Load the given mesh resource into a THREE Object3D.
 * 
 * @param resource - the mesh resource to load.
 * @returns the THREE Object 3D containing the loaded mesh
 */
ROS3D.MeshLoader.prototype.load = function(resource) {
  var object = new THREE.Object3D();
  var uri = this.path + resource;
  var fileType = uri.substr(-4).toLowerCase();

  // check the type
  if (uri.substr(-4).toLowerCase() === '.dae') {
    var loader = new ROS3D.ColladaLoader();
    loader.load(uri, function colladaReady(collada) {
      object.add(collada.scene);
    });
  }
  return object;
};
