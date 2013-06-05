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
  options = options || {};
  var urdfModel = options.urdfModel;
  var path = options.path || '/';
  var tfClient = options.tfClient;

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // load all models
  var links = urdfModel.links;
  for ( var l in links) {
    var link = links[l];
    if (link.visual && link.visual.geometry) {
      if (link.visual.geometry.type === ROSLIB.URDF_MESH) {
        var frameID = '/' + link.name;
        var uri = link.visual.geometry.filename;
        var fileType = uri.substr(-4).toLowerCase();

        // ignore mesh files which are not in Collada format
        if (fileType === '.dae') {
          // create the model
          var mesh = new ROS3D.MeshResource({
            path : path,
            resource : uri.substring(10)
          });
          
          // check for a scale
          if(link.visual.geometry.scale) {
            mesh.scale = new THREE.Vector3(
              link.visual.geometry.scale.x,
              link.visual.geometry.scale.y,
              link.visual.geometry.scale.z
            );
          }

          // create a scene node with the model
          var sceneNode = new ROS3D.SceneNode({
            frameID : frameID,
            pose : link.visual.origin,
            tfClient : tfClient,
            object : mesh
          });
          this.add(sceneNode);
        }
      }
    }
  }
};
ROS3D.Urdf.prototype.__proto__ = THREE.Object3D.prototype;
