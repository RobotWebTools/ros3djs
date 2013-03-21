/**
 * Class to handle the Box geometry type
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfBox = function() {
  var that = this;
  this.dimension = null;
  this.type = null;

  this.initXml = function(xml) {
    this.type = ROS3D.URDF_BOX;

    // parse the string
    var xyz = xml.getAttribute('size').split(' ');
    that.dimension = new ROSLIB.Vector3({
      x : parseFloat(xyz[0]),
      y : parseFloat(xyz[1]),
      z : parseFloat(xyz[2])
    });
  };
};
