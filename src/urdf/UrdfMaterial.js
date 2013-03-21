/**
 * Handles material information for the URDF
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfMaterial = function() {
  var that = this;
  this.name = null;
  this.textureFilename = null;
  this.color = null;

  this.initXml = function(xml) {
    that.name = xml.getAttribute('name');

    // texture
    var textures = xml.getElementsByTagName('texture');
    if (textures.length > 0) {
      that.textureFilename = textures[0].getAttribute('filename');
    }

    // color
    var colors = xml.getElementsByTagName('color');
    if (colors.length > 0) {
      // parse the RBGA string
      that.color = new ROS3D.UrdfColor();
      that.color.initString(colors[0].getAttribute('rgba'));
    }
  };
};
