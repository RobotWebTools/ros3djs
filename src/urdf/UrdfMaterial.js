/**
 * Handles material information for the URDF
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfMaterial = function() {
  var urdfMaterial = this;
  this.name;
  this.textureFilename;
  this.color;

  this.initXml = function(xml) {
    var hasRgba = false;
    var hasFilename = false;

    // check for the name
    if (!(urdfMaterial.name = xml.getAttribute('name'))) {
      console.error('URDF Material must contain a name attribute.');
      return false;
    }

    // texture
    var textures = xml.getElementsByTagName('texture');
    if (textures.length > 0) {
      var texture = textures[0];
      if ((urdfMaterial.textureFilename = texture.getAttribute('filename'))) {
        hasFilename = true;
      } else {
        console.error('URDF texture has no filename for material ' + urdfMaterial.name + '.');
      }
    }

    // color
    var colors = xml.getElementsByTagName('color');
    if (colors.length > 0) {
      var c = colors[0];
      if (c.getAttribute('rgba')) {
        // parse the RBGA string
        urdfMaterial.color = new ROS3D.UrdfColor();
        hasRgba = urdfMaterial.color.initString(c.getAttribute('rgba'));
      } else {
        console.error('Material ' + this.name + ' color has no rgba.');
      }
    }

    // check if we have a texture or color
    return (hasRgba || hasFilename);
  };
};
