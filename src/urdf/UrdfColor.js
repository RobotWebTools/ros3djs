/**
 * Class to handle color for urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfColor = function(options) {
  var that = this;
  var options = options || {};
  var xml = options.xml;
  this.r = null;
  this.g = null;
  this.b = null;
  this.a = null;

  var initXml = function(xml) {
    // parse the string
    var rgba = xml.getAttribute('rgba').split(' ');
    that.r = parseFloat(rgba[0]);
    that.g = parseFloat(rgba[1]);
    that.b = parseFloat(rgba[2]);
    that.a = parseFloat(rgba[3]);
    return true;
  };

  // pass it to the XML parser
  initXml(xml);
};
