/**
 * Class to handle color for urdf
 * @class
 * @augments Class
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfutils'],factory);
  }
  else {
    root.UrdfColor = factory(root.UrdfUtils);
  }
}(this, function(UrdfUtils) {
  var UrdfColor = function() {
		// members
		this.r = 0;
		this.g = 0;
		this.b = 0;
		this.a = 0;

    // methods
    this.clear = function () {
      this.r = 0;
      this.g = 0;
      this.b = 0;
      this.a = 1.0;
    };

    this.initString = function (str) {
      this.clear();
      
      var rgba = UrdfUtils.parseFloatListString(str);

      if (rgba.length != 4)
      {
        console.error("Color contains " + rgba.length + " elements instead of 4 elements");
        return false;
      }

      this.r = rgba[0];
      this.g = rgba[1];
      this.b = rgba[2];
      this.a = rgba[3];
      
      return true;
    };
  };

  return UrdfColor;
}));

