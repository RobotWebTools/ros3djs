ROS3D.UrdfModel = function() {
  var urdfModel = this;
  this.name;
  this.materials = [];

  this.initXml = function(xml) {
    // parse the string
    var parser = new DOMParser();
    var xmlDoc = parser.parseFromString(xml, 'text/xml');

    // check for the robot tag
    var robotXml = xmlDoc.evaluate('//robot', xmlDoc, null, XPathResult.FIRST_ORDERED_NODE_TYPE,
        null).singleNodeValue;
    if (!robotXml) {
      console.error('Could not find the "robot" element in the URDF XML file.');
      return false;
    }

    // get the robot name
    if (!(urdfModel.name = robotXml.getAttribute('name'))) {
      console.error("No name given for the robot.");
      return false;
    }

    // parse all the elements we need
    for (n in robotXml.childNodes) {
      var node = robotXml.childNodes[n];
      if (node.tagName === 'material') {
        var material = new ROS3D.UrdfMaterial();
        if (material.init(node)) {
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
        continue;
        var link_xml = node;
        var link = new UrdfLink();

        if (link.initXml(link_xml)) {
          if (urdfModel.getLink(link.name)) {
            console.error("link " + link.name + " is not unique. ");
            return false;
          } else {
            // console.log("setting link " + link.name + " material");
            if (link.visual) {
              if (link.visual.material_name.length > 0) {
                if (urdfModel.getMaterial(link.visual.material_name)) {
                  // console.log("Setting link " + link.name + " material to " +
                  // link.visual.material_name);
                  link.visual.material = urdfModel.getMaterial(link.visual.material_name);
                } else {
                  if (link.visual.material) {
                    // console.log("link " + link.name + " material " + link.visual.material_name +
                    // "
                    // define in Visual.");
                    urdfModel.links_[link.visual.material.name] = link.visual.material;
                  } else {
                    console.error("link " + link.name + " material " + link.visual.material_name
                        + " undefined.");
                    return false;
                  }
                }
              }
            }

            urdfModel.links_[link.name] = link;
            // console.log('successfully added a new link ' + link.name);
          }
        } else {
          return false;
        }

        if (urdfModel.links_.length == 0) {
          console.error('No link elements found in urdf file');
          return false;
        }
      } else if (node.tagName === 'joint') {
        // parse the joint
        var joint = new ROS3D.UrdfJoint();
        if (joint.init(node)) {
          if (urdfModel.joints[joint.name]) {
            console.error('Joint ' + joint.name + ' is not unique.');
            return false;
          } else {
            urdfModel.joints[joint.name] = joint;
          }
        } else {
          return false;
        }
      }
    }

    this.emit('ready');

    return true;
  };
};
ROS3D.UrdfModel.prototype.__proto__ = EventEmitter2.prototype;
