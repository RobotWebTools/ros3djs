/**
 * Handles material information for the URDF
 * @class
 * @augments Class
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfutils','./urdfcolor'],factory);
  }
  else {
    root.UrdfVector3 = factory(root.UrdfUtils,root.UrdfColor);
  }
}(this, function(UrdfUtils,UrdfColor) {

  var UrdfMaterial = function() {
    // members
    this.name = "";
    this.texture_filename = "";
    this.color = new UrdfColor();

    // methods
    this.clear = function () 
    {
      this.name = "";
      this.texture_filename = "";
      this.color.clear();
    };

    this.initXml = function (xml) {
      var has_rgb = false;
      var has_filename = false;

      this.clear();

      if (!xml.getAttribute("name"))
      {
        console.error("Material must contain a name attribute");
        return false;
      }

      this.name = xml.getAttribute("name");

      // texture
      var textures = xml.getElementsByTagName("texture");
      if(textures.length>0)
      {
        var texture = textures[0];
        if (texture.getAttribute("filename"))
        {
          this.texture_filename = texture.getAttribute("filename");
          has_filename = true;
        }
        else
        {
          console.error("texture has no filename for Material " + this.name);
        }
      }

      // color
      var colors = xml.getElementsByTagName("color");
      if(colors.length>0)
      {
        var c = colors[0];
        if (c.getAttribute("rgba"))
        {
          if (!this.color.initString(c.getAttribute("rgba")))
          {
            console.error("Material " + this.name + " has malformed color rgba values.");
            this.color.clear();
            return false;
          }
          else
            has_rgb = true;
        }
        else
        {
          console.error("Material " + this.name + " color has no rgba");
        }
      }


  //		if(has_rgb == false && has_filename ==false)
  //		{
  //		console.error("material xml is not initialized correctly");
  //		this.color.clear();
  //		has_rgb = true;
  //		}

      return (has_rgb || has_filename);
    };
  };

  return UrdfMaterial;
}));
