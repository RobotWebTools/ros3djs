/**
 * Class to handle 3 dimensional vectors
 * @class
 * @augments Class
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfutils'],factory);
  }
  else {
    root.UrdfVector3 = factory(root.UrdfUtils);
  }
}(this, function(UrdfUtils) {

  var UrdfVector3 = function(x,y,z) { 
		// members
		this.x = x || 0;
		this.y = y || 0;
		this.z = z || 0;


    // methods
    this.clear = function ()     {
      this.x = 0;
      this.y = 0;
      this.z = 0;
    };

    this.initString = function (str) {
      this.clear();
    
      var xyz = UrdfUtils.parseFloatListString(str);

      if (xyz.length != 3) {
        console.error("Vector contains " + xyz.length + " elements instead of 3 elements"); 
        return false;
      }

      this.x = xyz[0];
      this.y = xyz[1];
      this.z = xyz[2];

      return true;
    };
  };

  return UrdfVector3;
}));
