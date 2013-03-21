ROS3D.UrdfModel = function(options) {
  var that = this;
  var options = options || {};
  var xml = options.xml;
  var string = options.string;

  this.name;
  this.materials = [];
  this.links = [];

  var initXml = function(xml) {
    // get the robot tag
    var robotXml = xml.evaluate('//robot', xml, null, XPathResult.FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;

    // get the robot name
    that.name = robotXml.getAttribute('name');

    // parse all the visual elements we need
    for (n in robotXml.childNodes) {
      var node = robotXml.childNodes[n];
      if (node.tagName === 'material') {
        var material = new ROS3D.UrdfMaterial();
        material.initXml(node);
        // make sure this is unique
        if (that.materials[material.name]) {
          console.warn('Material ' + material.name + 'is not unique.');
        } else {
          that.materials[material.name] = material;
        }
      } else if (node.tagName === 'link') {
        var link = new ROS3D.UrdfLink();
        link.initXml(node);
        // make sure this is unique
        if (that.links[link.name]) {
          console.warn('Link ' + link.name + ' is not unique.');
        } else {
          // check for a material
          if (link.visual && link.visual.material) {
            if (that.materials[link.visual.material.name]) {
              link.visual.material = that.materials[link.visual.material.name];
            } else if (link.visual.material) {
              that.materials[link.visual.material.name] = link.visual.material;
            }
          }

          // add the link
          that.links[link.name] = link;
        }
      }
    }
  };

  // check if we are using a string or an XML element
  if (string) {
    // parse the string
    var parser = new DOMParser();
    xml = parser.parseFromString(string, 'text/xml');
  }
  // pass it to the XML parser
  initXml(xml);
};
