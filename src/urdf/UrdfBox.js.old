/**
 * Class to handle the Box geometry type
 * @class
 * @augments Class
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfvector3'],factory);
  }
  else {
    root.UrdfBox = factory(root.UrdfVector3);
  }
}(this, function(UrdfVector3) {

  var UrdfBox = function() {
    // members
    this.type = 0;
    this.GeometryTypes = {"SPHERE" : 0, "BOX" : 1, "CYLINDER" : 3, "MESH" : 4};
      
    this.dim = new UrdfVector3(-1,1,0);

    // methods
    this.clear = function () {
      this.dim.clear();
    };

    this.initXml = function (xml) {
      this.clear();
      this.type = this.GeometryTypes.BOX;
      if (!xml.getAttribute("size"))
      {
        console.error("Box shape must have a size attribute");
        return false;
      }
      if (!this.dim.initString(xml.getAttribute("size")))
      {
        console.error("Box shape has malformed size attribute");
        dim.clear();
        return false;
      }
      return true;
    }; 
  };

  return UrdfBox;

}));

