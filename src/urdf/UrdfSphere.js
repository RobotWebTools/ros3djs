/**
 * Class to handle visualizing spheres in the urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfSphere = function(options) {
  var that = this;
  var options = options || {};
  var xml = options.xml;
  this.radius = null;
  this.type = null;

  var initXml = function(xml) {
    that.type = ROS3D.URDF_SPHERE;
    that.radius = parseFloat(xml.getAttribute('radius'));
  };
  
  // pass it to the XML parser
  initXml(xml);
};
