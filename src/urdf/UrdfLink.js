ROS3D.UrdfLink = function() {
  var that = this;
  this.name = null;
  this.visual = null;

  this.initXml = function(xml) {
    that.name = xml.getAttribute('name');
    var visuals = xml.getElementsByTagName('visual');
    if (visuals.length > 0) {
      that.visual = new ROS3D.UrdfVisual();
      that.visual.initXml(visuals[0]);
    }
  };
};
