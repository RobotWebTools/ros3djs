/**
 * Handles material information for the URDF
 * @class
 * @augments Class
 */
ROS3D.UrdfMaterial = function() {
  var urdfMaterial = this;
  this.name;
  this.textureFilename;
  this.color;

  this.init = function(xmlNode) {
    var hasRgba = false;
    var hasFilename = false;

    // check for the name
    if (!(urdfMaterial.name = xmlNode.getAttribute('name'))) {
      console.error('URDF Material must contain a name attribute.');
      return false;
    }

    // texture
    var textures = xmlNode.getElementsByTagName('texture');
    if (textures.length > 0) {
      var texture = textures[0];
      if ((urdfMaterial.textureFilename = texture.getAttribute('filename'))) {
        hasFilename = true;
      } else {
        console.error('URDF texture has no filename for material ' + urdfMaterial.name + '.');
      }
    }

    // color
    var colors = xmlNode.getElementsByTagName('color');
    if (colors.length > 0) {
      var c = colors[0];
      if (c.getAttribute('rgba')) {
        // parse the RBGA string
        var rgba = c.getAttribute('rgba').split(' ');
        urdfMaterial.color = new ROS3D.UrdfColor(parseInt(rgba[0], rgba[1], rgba[2], rgba[3]));
        hasRgba = true;
      } else {
        console.error('Material ' + this.name + ' color has no rgba.');
      }
    }

    // check if we have a texture or color
    if (hasRgba || hasFilename) {
      return true;
    } else {
      console.error('Material ' + this.name + ' has no color or texture.');
      return false;
    }
  };
};
