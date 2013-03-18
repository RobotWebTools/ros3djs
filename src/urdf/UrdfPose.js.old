/**
 *  Stores urdf pose information
 *  @class
 * @augments Class
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfvector3','./urdfrotation'],factory);
  }
  else {
    root.UrdfPose = factory(root.UrdfVector3,root.UrdfRotation);
  }
}(this, function(UrdfVector3,UrdfRotation) {                          		
	
  var UrdfPose = function() {
  
		// members
		this.position = new UrdfVector3();
		this.orientation = new UrdfRotation();

    // methods
    this.clear = function ()     {
      this.position.clear();
      this.orientation.clear();
    };

    this.initXml = function (xml) {
      this.clear();

      var xyz_str = xml.getAttribute("xyz");
      if (!xyz_str)
      {
        //console.log("parsing pose: no xyz, using default values.");
        return true;
      }
      else
      {
        if (!this.position.initString(xyz_str))
        {
          console.error("malformed xyz");
          this.position.clear();
          return false;
        }
      }

      rpy_str = xml.getAttribute("rpy");
      if (!rpy_str)
      {
        //console.log("parsing pose: no rpy, using default values.");
        return true;
      }
      else
      {
        if (!this.orientation.initString(rpy_str))
        {
          console.error("malformed rpy");
          return false;
          this.orientation.clear();
        }
      }

      return true;
    };
  };

  return UrdfPose;
}));
