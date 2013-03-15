/**
 * Class to handle rotation information in the urdf
 * @class
 * @augments Class
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfutils','./urdfvector3'],factory);
  }
  else {
    root.UrdfRotation = factory(root.UrdfUtils,root.UrdfVector3);
  }
}(this, function(UrdfUtils,UrdfVector3) {

  var UrdfRotation = function() {
		// members
		this.x = 0;
		this.y = 0;
		this.z = 0;
		this.w = 1.0;

    // methods
    this.clear = function () {
      this.x = 0;
      this.y = 0;
      this.z = 0;
      this.w = 1.0;
    };

    this.initString = function (str) {
    
      this.clear();
    
      var rpy = new UrdfVector3();
    
      if (!rpy.initString(str)) {
        return false;
      }
      else
      {
        this.setFromRPY(rpy.x,rpy.y,rpy.z);
        return true;
      }
      delete rpy;
    
      return true;
    };

    this.setFromRPY = function(roll, pitch, yaw) {
      var phi = roll / 2.0;
      var the = pitch / 2.0;
      var psi = yaw / 2.0;

      this.x = Math.sin(phi) * Math.cos(the) * Math.cos(psi) - Math.cos(phi) * Math.sin(the) * Math.sin(psi);
      this.y = Math.cos(phi) * Math.sin(the) * Math.cos(psi) + Math.sin(phi) * Math.cos(the) * Math.sin(psi);
      this.z = Math.cos(phi) * Math.cos(the) * Math.sin(psi) - Math.sin(phi) * Math.sin(the) * Math.cos(psi);
      this.w = Math.cos(phi) * Math.cos(the) * Math.cos(psi) + Math.sin(phi) * Math.sin(the) * Math.sin(psi);

      this.normalize();
    };

    this.normalize = function() {
      var s = Math.sqrt(this.x * this.x +
                      this.y * this.y +
                      this.z * this.z +
                      this.w * this.w);
      if (s == 0.0)
      {
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
        this.w = 1.0;
      }
      else
      {
        this.x /= s;
        this.y /= s;
        this.z /= s;
        this.w /= s;
      }
    };
  };
  return UrdfRotation;
}));
