ROS3D.UrdfLink = function(options) {
  var that = this;
  var options = options || {};
  var xml = options.xml;
  this.name = null;
  this.visual = null;

  var initXml = function(xml) {
    that.name = xml.getAttribute('name');
    var visuals = xml.getElementsByTagName('visual');
    if (visuals.length > 0) {
      that.visual = new ROS3D.UrdfVisual({
        xml : visuals[0]
      });
    }
  };

  // pass it to the XML parser
  initXml(xml);
};
