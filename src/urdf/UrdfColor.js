/**
 * Class to handle color for urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfColor = function() {
  var that = this;
  this.r = null;
  this.g = null;
  this.b = null;
  this.a = null;

  this.initString = function(str) {
    // parse the string
    var rgba = str.split(' ');
    that.r = parseFloat(rgba[0]);
    that.g = parseFloat(rgba[1]);
    that.b = parseFloat(rgba[2]);
    that.a = parseFloat(rgba[3]);
    return true;
  };
};
