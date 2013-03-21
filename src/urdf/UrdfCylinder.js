/**
 * Class to handle the Cylinder geometry type of a URDF.
 * 
 * @class
 */
ROS3D.UrdfCylinder = function() {
  this.type = 0;
  this.length = 0;
  this.radius = 0;

  this.initXml = function(xml) {
    this.type = ROS3D.URDF_CYLINDER;
    this.length = parseFloat(xml.getAttribute('length'));
    this.radius = parseFloat(xml.getAttribute('radius'));
  };
};
