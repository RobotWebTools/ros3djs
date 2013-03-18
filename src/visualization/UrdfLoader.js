ROS3D.UrdfLoader = function(options) {
  var urdfLoader = this;
  var options = options || {};
  this.ros = options.ros;
  this.xmlParam = options.xmlParam || 'robot_description';
  this.urdf = new ROS3D.UrdfModel();

  // wait for the model to load
  this.urdf.on('ready', function() {
    console.log("OKAY");
  });
  
  // get the value from ROS
  var param = new ROSLIB.Param({
    ros : this.ros,
    name : this.xmlParam
  });
  param.get(function(xml) {
    // hand off the XML to the URDF model
    urdfLoader.urdf.initXml(xml);
  });
};

var UrdfLoader = {};

UrdfLoader.load = function(objroot, meshLoader, tfClient, urdf_src) {

  var urdf_model = new UrdfModel();
  var that = this;
  var scene_handlers = {};

  urdf_model.initFile(urdf_src, urdfReady);

  function urdfReady() {
    // load all models
    var links = urdf_model.getLinks();
    for ( var l in links) {
      var link = links[l];
      if (!link.visual)
        continue;
      if (!link.visual.geometry)
        continue;
      if (link.visual.geometry.type == link.visual.geometry.GeometryTypes.MESH) {
        var frame_id = new String("/" + link.name);
        var uri = link.visual.geometry.filename;
        var uriends = uri.substr(-4).toLowerCase();

        // ignore mesh files which are not in collada format
        if (uriends == ".dae") {
          var material_name = link.visual.material_name;
          var material = urdf_model.getMaterial(material_name);

          var collada_model = meshLoader.load(uri, material);

          var scene_node = new SceneNode({
            frame_id : frame_id,
            tfclient : tfClient,
            pose : link.visual.origin,
            model : collada_model
          });
          objroot.add(scene_node);
        } else {
          console.log("Not Supported format : ", uri);
        }
      }
    }
  }
};
