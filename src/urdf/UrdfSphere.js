/**
 * Class to handle visualizing spheres in the urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfSphere = function() {
  var that = this;
  this.radius = null;
  this.type = null;

  this.initXml = function(xml) {
    that.type = ROS3D.URDF_SPHERE;
    that.radius = parseFloat(xml.getAttribute('radius'));
  };
};
