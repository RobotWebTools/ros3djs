ROS3D.UrdfLink = function() {
  var urdfLink = this;
  this.name;
  this.visual;

  this.initXml = function(xml) {
    if (!(urdfLink.name = xml.getAttribute('name'))) {
      console.error('No name given for link.');
      return false;
    }

    // visual (optional)
    var visuals = xml.getElementsByTagName('visual');
    if (visuals.length > 0) {
      var visualXml = visuals[0];
      var visual = new ROS3D.UrdfVisual();
      if (!visual.initXml(visualXml)) {
        console.error('Could not parse visual element for Link ' + this.name);
        return false;
      }
      urdfLink.visual = visual;
    }

    return true;
  };
};
