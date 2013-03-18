ROS3D.UrdfModel = function() {
  var urdfModel = this;
  this.name;
  this.materials = [];
  this.links = [];

  this.initXml = function(xml) {
    // check for the robot tag
    var robotXml = xml.evaluate('//robot', xml, null, XPathResult.FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
    if (!robotXml) {
      console.error('Could not find the "robot" element in the URDF XML file.');
      return false;
    }

    // get the robot name
    if (!(urdfModel.name = robotXml.getAttribute('name'))) {
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
          if (urdfModel.materials[material.name]) {
            console.error('Material ' + material.name + 'is not unique.');
            return false;
          } else {
            urdfModel.materials[material.name] = material;
          }
        } else {
          return false;
        }
      } else if (node.tagName === 'link') {
        var link = new ROS3D.UrdfLink();

        if (link.initXml(node)) {
          if (urdfModel.links[link.name]) {
            console.error('Link ' + link.name + ' is not unique.');
            return false;
          } else {
            // check for a material
            if (link.visual && link.visual.materialName) {
              if (urdfModel.materials[link.visual.materialName]) {
                link.visual.material = urdfModel.materials[link.visual.materialName];
              } else {
                if (link.visual.material) {
                  urdfModel.materials[link.visual.material.name] = link.visual.material;
                } else {
                  console.error('Link ' + link.name + ' material ' + link.visual.material_name
                      + ' is undefined.');
                  return false;
                }
              }
            }

            // add the link
            urdfModel.links[link.name] = link;
          }
        } else {
          console.error('Could not parse link.');
          return false;
        }
      }
    }

    return true;
  };

  this.initString = function(str) {
    // parse the string
    var parser = new DOMParser();
    var xml = parser.parseFromString(str, 'text/xml');

    // pass it to the XML parser
    return urdfModel.initXml(xml);
  };
};
