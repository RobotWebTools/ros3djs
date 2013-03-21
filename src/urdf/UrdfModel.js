ROS3D.UrdfModel = function(options) {
  var that = this;
  var options = options || {};
  var xml = options.xml;
  var string = options.string;

  this.name;
  this.materials = [];
  this.links = [];

  var initXml = function(xml) {
    // check for the robot tag
    var robotXml = xml.evaluate('//robot', xml, null, XPathResult.FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
    if (!robotXml) {
      console.error('Could not find the "robot" element in the URDF XML file.');
      return false;
    }

    // get the robot name
    if (!(that.name = robotXml.getAttribute('name'))) {
      console.error("No name given for the robot.");
      return false;
    }

    // parse all the visual elements we need
    for (n in robotXml.childNodes) {
      var node = robotXml.childNodes[n];
      if (node.tagName === 'material') {
        var material = new ROS3D.UrdfMaterial();
        if (material.initXml(node)) {
          // make sure this is unique
          if (that.materials[material.name]) {
            console.error('Material ' + material.name + 'is not unique.');
            return false;
          } else {
            that.materials[material.name] = material;
          }
        } else {
          return false;
        }
      } else if (node.tagName === 'link') {
        var link = new ROS3D.UrdfLink();

        if (link.initXml(node)) {
          if (that.links[link.name]) {
            console.error('Link ' + link.name + ' is not unique.');
            return false;
          } else {
            // check for a material
            if (link.visual && link.visual.materialName) {
              if (that.materials[link.visual.materialName]) {
                link.visual.material = that.materials[link.visual.materialName];
              } else {
                if (link.visual.material) {
                  that.materials[link.visual.material.name] = link.visual.material;
                } else {
                  console.error('Link ' + link.name + ' material ' + link.visual.material_name
                      + ' is undefined.');
                  return false;
                }
              }
            }

            // add the link
            that.links[link.name] = link;
          }
        } else {
          console.error('Could not parse link.');
          return false;
        }
      }
    }

    return true;
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
