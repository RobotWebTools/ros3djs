/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A URDF loader can be used to load a URDF and its associated models into a 3D object. By default,
 * the URDF is loaded from the ROS parameter server.
 *
 * Emits the following events:
 * * 'ready' - emited after the URDF and its meshes have been loaded into the root object
 * 
 * @constructor
 * @param options - object with following keys:
 *  * tfClient - a handle to the TF client to be used to update the model's pose
 *  * path - the base path to the associated Collada models that will be loaded
 */
ROS3D.UrdfLoader = function(options) {
  var that = this;
  var options = options || {};
  this.tfClient = options.tfClient;
  this.path = options.path || 'resources/';
};

/**
 * Load the given URDF into a THREE Object3D.
 * 
 * @param string - the URDF as a string
 * @returns the THREE Object3D that contains the entire URDF
 */
ROS3D.UrdfLoader.prototype.load = function(string) {
  // hand off the XML string to the URDF model
  var urdfModel = new ROSLIB.UrdfModel({
    string : string
  });

  var objectRoot = new THREE.Object3D();

  // load all models
  var links = urdfModel.links;
  var meshLoader = new ROS3D.MeshLoader({
    path : this.path
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
            tfClient : this.tfClient,
            model : colladaModel
          });
          objectRoot.add(sceneNode);
        }
      }
    }
  }
  return objectRoot;
};
