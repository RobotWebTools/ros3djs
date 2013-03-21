/**
 * Class to handle the Cylinder geometry type of a URDF.
 * 
 * @class
 */
ROS3D.UrdfCylinder = function(options) {
  var that = this;
  var options = options || {};
  var xml = options.xml;
  this.type = null;
  this.length = null;
  this.radius = null;

  var initXml = function(xml) {
    that.type = ROS3D.URDF_CYLINDER;
    that.length = parseFloat(xml.getAttribute('length'));
    that.radius = parseFloat(xml.getAttribute('radius'));
  };
  
  // pass it to the XML parser
  initXml(xml);
};
