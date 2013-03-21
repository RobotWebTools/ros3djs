/**
 * Class to handle the Box geometry type
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfBox = function() {
  var urdfBox = this;
  this.dim;
  this.type;

  this.initXml = function(xml) {
    this.type = ROS3D.URDF_BOX;
    // check for a size
    if (!xml.getAttribute('size')) {
      console.error("Box shape must have a size attribute'");
      return false;
    }
    // parse the string
    var xyz = xml.getAttribute('size').split(' ');
    if (xyz.length !== 3) {
      console.error('Invalid size string.');
      return false;
    } else {
      urdfBox.dim = new ROSLIB.Vector3({
        x : parseFloat(xyz[0]),
        y : parseFloat(xyz[1]),
        z : parseFloat(xyz[2])
      });
    }

    return true;
  };
};
