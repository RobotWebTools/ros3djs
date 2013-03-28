/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A URDF can be used to load a ROSLIB.UrdfModel and its associated models into a 3D object.
 *  
 * @constructor
 * @param options - object with following keys:
 *   * urdfModel - the ROSLIB.UrdfModel to load
 *   * tfClient - the TF client handle to use
 *   * path (optional) - the base path to the associated Collada models that will be loaded
 */
ROS3D.Urdf = function(options) {
  var options = options || {};
  var urdfModel = options.urdfModel;
  var path = options.path || '/';
  var tfClient = options.tfClient;

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // load all models
  var links = urdfModel.links;
  var meshLoader = new ROS3D.MeshLoader({
    path : path
  });
  for ( var l in links) {
    var link = links[l];
    if (link.visual && link.visual.geometry) {
      if (link.visual.geometry.type === ROSLIB.URDF_MESH) {
        var frameID = '/' + link.name;
        var uri = link.visual.geometry.filename;
        var fileType = uri.substr(-4).toLowerCase();

        // ignore mesh files which are not in Collada format
        if (fileType === '.dae') {
          var colladaModel = meshLoader.load(uri.substring(10));
          // create a scene node with the model
          var sceneNode = new ROS3D.SceneNode({
            frameID : frameID,
            tfClient : tfClient,
            object : colladaModel
          });
          this.add(sceneNode);
        }
      }
    }
  }
};
ROS3D.Urdf.prototype.__proto__ = THREE.Object3D.prototype;
