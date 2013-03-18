ROS3D.UrdfVisual = function() {
  var urdfVisual = this;
  this.origin;
  this.geometry;
  this.material;
  this.materialName;

  this.initXml = function(xml) {
    // origin
    var origins = xml.getElementsByTagName('origin');
    if (origins.length === 0) {
      // use the identity as the default
      urdfVisual.origin = new ROSLIB.Pose();
    } else {
      // check the XYZ
      var xyz = xml.getAttribute('xyz');
      if (!xyz) {
        // use the default values
        var position = new ROSLIB.Vector3();
      } else {
        var xyz = xyz.split(' ');
        if (xyz.length !== 3) {
          console.error('Invalid XYZ string in origin.');
          return false;
        } else {
          var position = new ROSLIB.Vector3(parseFloat(xyz[0]), parseFloat(xyz[1]),
              parseFloat(xyz[2]));
        }
      }

      // check the RPY
      rpy = xml.getAttribute('rpy');
      if (!rpy) {
        // use the default values
        var orientation = new ROSLIB.Quaternion();
      } else {
        var rpy = rpy.split(' ');
        if (rpy.length !== 3) {
          console.error('Invalid RPY string in origin.');
          return false;
        } else {
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

          var orientation = new ROSLIB.Quaternion(x, y, z, w);
          orientation.normalize();
        }
      }
      urdfVisual.origin = new ROSLIB.Pose(position, orientation);
    }

    var geoms = xml.getElementsByTagName('geometry');
    if (geoms.length > 0) {
      var shape;
      // check for the shape
      for (n in geoms[0].childNodes) {
        var node = geoms[0].childNodes[n];
        if (node.nodeType === 1) {
          shape = node;
          break;
        }
      }
      if (!shape) {
        console.error('Geometry tag contains no child element.');
        return false;
      }
      var type = shape.nodeName;
      if (type === 'sphere') {
        geometry = new ROS3D.UrdfSphere();
      } else if (type === 'box') {
        geometry = new ROS3D.UrdfBox();
      } else if (type === 'cylinder') {
        geometry = new ROS3D.UrdfCylinder();
      } else if (type === 'mesh') {
        geometry = new ROS3D.UrdfMesh();
      } else {
        console.error('Unknown geometry type ' + type);
        return false;
      }
      if (!geometry.initXml(shape)) {
        console.error('Could not parse visual.');
        return false;
      }

      urdfVisual.geometry = geometry;
    }

    // material (optional)
    var materials = xml.getElementsByTagName('material');
    if (materials.length > 0) {
      // get material name
      var materialXml = materials[0];
      var material = new ROS3D.UrdfMaterial();
      // could just be a name
      if (!(urdfVisual.materialName = materialXml.getAttribute('name'))) {
        console.error('URDF material must contain a name attribute');
        return false;
      }

      // try to parse material element in place
      if (material.initXml(materialXml)) {
        urdfVisual.material = material;
      }
    }
    return true;
  };
};
