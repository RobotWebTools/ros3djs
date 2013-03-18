/**
 * Mesh information for the urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfMesh = function() {
  var urdfMesh = this;
  this.filename = '';
  this.scale;
  this.type;

  this.initXml = function(xml) {
    this.type = ROS3D.URDF_MESH;
    if (!(urdfMesh.filename = xml.getAttribute('filename'))) {
      console.error('Mesh must contain a filename attribute.');
      return false;
    }

    // check for a scale
    if (xml.getAttribute('scale')) {
      // check the XYZ
      var xyz = xml.getAttribute('scale').split(' ');
      if (xyz.length !== 3) {
        console.error('Invalid scale string.');
        return false;
      } else {
        urdfMesh.scale = new ROSLIB.Vector3(parseFloat(xyz[0]), parseFloat(xyz[1]),
            parseFloat(xyz[2]));
      }
    }

    return true;
  };
};
