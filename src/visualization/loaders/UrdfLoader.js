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
 *  * ros - a handle to the ROS connection
 *  * tfClient - a handle to the TF client to be used to update the model's pose
 *  * xmlParam - the parameter name to load the URDF from, like 'robot_description'
 *  * objectRoot - the THREE 3D object to be used as the root of the model
 *  * path - the base path to the associated Collada models that will be loaded
 */
ROS3D.UrdfLoader = function(options) {
  var that = this;
  var options = options || {};
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.xmlParam = options.xmlParam || 'robot_description';
  this.objectRoot = options.objectRoot || new THREE.Object3D();
  this.path = options.path || 'resources/';
  this.urdfModel = null;

  // get the value from ROS
  var param = new ROSLIB.Param({
    ros : this.ros,
    name : this.xmlParam
  });
  param.get(function(xml) {
    // hand off the XML string to the URDF model
    that.urdfModel = new ROS3D.UrdfModel({
      string : xml
    });
    // load all models
    var links = that.urdfModel.links;
    for ( var l in links) {
      var link = links[l];
      if (link.visual && link.visual.geometry) {
        if (link.visual.geometry.type === ROS3D.URDF_MESH) {
          var frameID = '/' + link.name;
          var uri = link.visual.geometry.filename;
          var fileType = uri.substr(-4).toLowerCase();

          // ignore mesh files which are not in Collada format
          var meshLoader = new ROS3D.MeshLoader(that.path);
          if (fileType === '.dae') {
            var material = that.urdfModel.materials[link.visual.materialName];
            var colladaModel = meshLoader.load(uri, material);
            // create a scene node with the model
            var sceneNode = new ROS3D.SceneNode({
              frameID : frameID,
              tfClient : that.tfClient,
              pose : link.visual.origin,
              model : colladaModel
            });
            that.objectRoot.add(sceneNode);
          }
        }
      }
      that.emit('ready');
    }
  });
};
ROS3D.UrdfLoader.prototype.__proto__ = EventEmitter2.prototype;
