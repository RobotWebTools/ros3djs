/**
 * Mesh information for the urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfMesh = function() {
  var that = this;
  this.filename = null;
  this.scale = null;
  this.type = null;

  this.initXml = function(xml) {
    that.type = ROS3D.URDF_MESH;
    that.filename = xml.getAttribute('filename');

    // check for a scale
    var scale = xml.getAttribute('scale');
    if (scale) {
      // get the XYZ
      var xyz = scale.split(' ');
      that.scale = new ROSLIB.Vector3({
        x : parseFloat(xyz[0]),
        y : parseFloat(xyz[1]),
        z : parseFloat(xyz[2])
      });
    }
  };
};
