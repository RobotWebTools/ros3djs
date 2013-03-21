/**
 * Class to handle the Cylinder geometry type of a URDF.
 * 
 * @class
 */
ROS3D.UrdfCylinder = function() {
  var that = this;
  this.type = null;
  this.length = null;
  this.radius = null;

  this.initXml = function(xml) {
    that.type = ROS3D.URDF_CYLINDER;
    that.length = parseFloat(xml.getAttribute('length'));
    that.radius = parseFloat(xml.getAttribute('radius'));
  };
};
