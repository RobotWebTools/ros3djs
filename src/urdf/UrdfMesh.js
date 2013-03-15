/**
 * Mesh information for the urdf
 * @class
 * @augments Class
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfvector3'],factory);
  }
  else {
    root.UrdfMesh = factory(root.UrdfVector3);
  }
}(this, function(UrdfVector3) {
  var UrdfMesh = function() { 
		// members
		this.type = 0;
		this.GeometryTypes = {"SPHERE" : 0, "BOX" : 1, "CYLINDER" : 3, "MESH" : 4};
		this.filename = "";
		this.scale = new UrdfVector3();

    // methods
    this.clear = function () {
      this.filename = "";
      // default scale
      this.scale.x = 1;
      this.scale.y = 1;
      this.scale.z = 1;
    };

    this.initXml = function (xml) {
      this.clear();
      this.type = this.GeometryTypes.MESH;
      if (!xml.getAttribute("filename"))
      {
        console.error("Mesh must contain a filename attribute");
        return false;
      }

      this.filename = xml.getAttribute("filename");

      if (xml.getAttribute("scale"))
      {
        if (!this.scale.initString(xml.getAttribute("scale")))
        {
          console.error("Mesh scale was specified, but could not be parsed");
          this.scale.clear();
          return false;
        }
      }
  //		else
  //		{
  //		console.error("Mesh must contain a scale attribute");
  //		return false;
  //		}

      return true;
    };
  };

  return UrdfMesh;
}));
