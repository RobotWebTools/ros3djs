/**
 * Class to handle color for urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfColor = function() {
  var urdfColor = this;
  this.r;
  this.g;
  this.b;
  this.a;

  this.initString = function(str) {
    // try and parse the string
    var rgba = str.split(' ');
    if (rgba.length !== 4) {
      console.error('Invalid RBGA string.');
      return false;
    } else {
      urdfColor.r = parseFloat(rgba[0]);
      urdfColor.g = parseFloat(rgba[1]);
      urdfColor.b = parseFloat(rgba[2]);
      urdfColor.a = parseFloat(rgba[3]);
      return true;
    }
  };
};
