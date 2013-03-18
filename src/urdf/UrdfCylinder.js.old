/**
 * Class to handle Cylinder geometry type
 * @class
 * @augments Class
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define([],factory);
  }
  else {
    root.UrdfCylinder = factory();
  }
}(this, function() {                          		
	
  var UrdfCylinder = function() {
		// members
		this.type = 0;
		this.GeometryTypes = {"SPHERE" : 0, "BOX" : 1, "CYLINDER" : 3, "MESH" : 4};
		this.length = 0;
		this.radius = 0;

    // methods
    this.clear = function () {
      this.length = 0;
      this.radius = 0;
    };

    this.initXml = function (xml) { 
      this.clear();
      this.type = this.GeometryTypes.CYLINDER;
      if (!xml.getAttribute("length") ||
          !xml.getAttribute("radius"))
      {
        ros_error("Cylinder shape must have both length and radius attributes");
        return false;
      }

      this.length = parseFloat(xml.getAttribute("length"));
      this.radius = parseFloat(xml.getAttribute("radius"));
     
      return true;
    };
  };

  return UrdfCylinder;

}));
