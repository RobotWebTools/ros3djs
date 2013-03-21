ROS3D.UrdfLoader = function(options) {
  var urdfLoader = this;
  var options = options || {};
  this.ros = options.ros;
  this.xmlParam = options.xmlParam || 'robot_description';
  this.objectRoot = options.objectRoot || new THREE.Object3D();
  this.urdfModel = new ROS3D.UrdfModel();
  this.meshLoader = new ROS3D.MeshLoader('resources/');
  this.tfClient = options.tfClient;

  // get the value from ROS
  var param = new ROSLIB.Param({
    ros : this.ros,
    name : this.xmlParam
  });
  param.get(function(xml) {
    // hand off the XML string to the URDF model
    if (urdfLoader.urdfModel.initString(xml)) {
      // load all models
      var links = urdfLoader.urdfModel.links;
      for ( var l in links) {
        var link = links[l];
        if (link.visual && link.visual.geometry) {
          if (link.visual.geometry.type === ROS3D.URDF_MESH) {
            var frameID = new String('/' + link.name);
            var uri = link.visual.geometry.filename;
            var fileType = uri.substr(-4).toLowerCase();

            // ignore mesh files which are not in collada format
            if (fileType === '.dae') {
              var material = urdfLoader.urdfModel.materials[link.visual.materialName];
              var colladaModel = urdfLoader.meshLoader.load(uri, material);

              var sceneNode = new ROS3D.SceneNode({
                frameID : frameID,
                tfClient : urdfLoader.tfClient,
                pose : link.visual.origin,
                model : colladaModel
              });
              urdfLoader.objectRoot.add(sceneNode);
            }
          }
        }
      }
      urdfLoader.emit('ready');
    }
  });

  this.getObjectRoot = function() {
    return urdfLoader.objectRoot;
  };
};
ROS3D.UrdfLoader.prototype.__proto__ = EventEmitter2.prototype;
