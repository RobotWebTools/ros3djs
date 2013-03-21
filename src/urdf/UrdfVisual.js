ROS3D.UrdfVisual = function() {
  var that = this;
  this.origin = null;
  this.geometry = null;
  this.material = null;

  this.initXml = function(xml) {
    // origin
    var origins = xml.getElementsByTagName('origin');
    if (origins.length === 0) {
      // use the identity as the default
      that.origin = new ROSLIB.Pose();
    } else {
      // check the XYZ
      var xyz = xml.getAttribute('xyz');
      if (!xyz) {
        // use the default values
        var position = new ROSLIB.Vector3();
      } else {
        var xyz = xyz.split(' ');
        var position = new ROSLIB.Vector3({
          x : parseFloat(xyz[0]),
          y : parseFloat(xyz[1]),
          z : parseFloat(xyz[2])
        });
      }

      // check the RPY
      rpy = xml.getAttribute('rpy');
      if (!rpy) {
        // use the default values
        var orientation = new ROSLIB.Quaternion();
      } else {
        var rpy = rpy.split(' ');
        // convert from RPY
        var roll = parseFloat(rpy[0]);
        var pitch = parseFloat(rpy[1]);
        var yaw = parseFloat(rpy[2]);
        var phi = roll / 2.0;
        var the = pitch / 2.0;
        var psi = yaw / 2.0;
        var x = Math.sin(phi) * Math.cos(the) * Math.cos(psi) - Math.cos(phi) * Math.sin(the)
            * Math.sin(psi);
        var y = Math.cos(phi) * Math.sin(the) * Math.cos(psi) + Math.sin(phi) * Math.cos(the)
            * Math.sin(psi);
        var z = Math.cos(phi) * Math.cos(the) * Math.sin(psi) - Math.sin(phi) * Math.sin(the)
            * Math.cos(psi);
        var w = Math.cos(phi) * Math.cos(the) * Math.cos(psi) + Math.sin(phi) * Math.sin(the)
            * Math.sin(psi);

        var orientation = new ROSLIB.Quaternion({
          x : x,
          y : y,
          z : z,
          w : w
        });
        orientation.normalize();
      }
      that.origin = new ROSLIB.Pose({
        position : position,
        orientation : orientation
      });
    }

    // geometry
    var geoms = xml.getElementsByTagName('geometry');
    if (geoms.length > 0) {
      var shape = null;
      // check for the shape
      for (n in geoms[0].childNodes) {
        var node = geoms[0].childNodes[n];
        if (node.nodeType === 1) {
          shape = node;
          break;
        }
      }
      // check the type
      var type = shape.nodeName;
      if (type === 'sphere') {
        that.geometry = new ROS3D.UrdfSphere();
      } else if (type === 'box') {
        that.geometry = new ROS3D.UrdfBox();
      } else if (type === 'cylinder') {
        that.geometry = new ROS3D.UrdfCylinder();
      } else if (type === 'mesh') {
        that.geometry = new ROS3D.UrdfMesh();
      } else {
        console.warn('Unknown geometry type ' + type);
      }
      that.geometry.initXml(shape);
    }

    // material
    var materials = xml.getElementsByTagName('material');
    if (materials.length > 0) {
      // get material name
      that.material = new ROS3D.UrdfMaterial();
      // try to parse material element in place
      that.material.initXml(materials[0]);
    }
  };
};
