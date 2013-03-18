/**
 * Class to handle visualizing spheres in the urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfSphere = function() {
  var urdfSphere = this;
  this.radius;
  this.type;

  this.initXml = function(xml) {
    this.type = ROS3D.URDF_SPHERE;
    if (!xml.getAttribute('radius')) {
      console.error('Sphere shape must have a radius attribute');
      return false;
    }
    urdfSphere.radius = parseFloat(xml.getAttribute("radius"));
    return true;
  };
};
