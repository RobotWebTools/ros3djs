/**
 * Class to handle visualizing spheres in the urdf
 * @class
 * @augments Class
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define([],factory);
  }
  else {
    root.UrdfSphere = factory();
  }
}(this, function() {
  var UrdfSphere = function() {
		// members
		this.type = 0;
		this.GeometryTypes = {"SPHERE" : 0, "BOX" : 1, "CYLINDER" : 3, "MESH" : 4};
		this.radius = 0;

    // methods
    this.clear = function() {
  		this.radius = 0;
	  };

    this.initXml = function(xml) {
		  this.clear();
  		this.type = this.GeometryTypes.SPHERE;
	  	if (!xml.getAttribute("radius"))
		  {
			  console.error("Sphere shape must have a radius attribute");
  	  	return false;
	  	}

  		this.radius = parseFloat(xml.getAttribute("radius"));
   
	  	return true;
    };
	};

  return UrdfSphere;
}));


