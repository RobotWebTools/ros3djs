/**
 * Handles material information for the URDF
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfMaterial = function(options) {
  var that = this;
  var options = options || {};
  var xml = options.xml;
  this.name = null;
  this.textureFilename = null;
  this.color = null;

  var initXml = function(xml) {
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
      that.color = new ROS3D.UrdfColor({
        xml : colors[0]
      });
    }
  };

  // pass it to the XML parser
  initXml(xml);
};
