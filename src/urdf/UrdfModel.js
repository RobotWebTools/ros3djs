(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfmaterial','./urdflink','./urdfjoint'], factory);
  }
  else {
    root.UrdfModel = factory(root.UrdfMaterial,root.UrdfLink,root.UrdfJoint);
  }
}(this, function(UrdfMaterial,UrdfLink,UrdfJoint) {
  var UrdfModel = function() {
    var urdf = this;

    // members
    this.name_ = '';
    this.links_ = {};
    this.joints_ = {};
    this.materials_ = {};

    this.clear = function() {
      this.name_ = '';
      this.links_ = {};
      this.joints_ = {};
      this.materials_ = {};
    }

    this.initFile = function(src, callback) {
      var xhr = new XMLHttpRequest();
      var that = this;
      xhr.onreadystatechange = function() {
        if (xhr.readyState == 4 && (xhr.status == 200 || xhr.status == 0 )) {
          window.setTimeout(function() {
            var xml = xhr.responseXML;
            xml.getElementById = function(id) {
              return xpathGetElementById(xml, id);
            };
            that.initXml(xml);

            if (callback) {
              callback(that);
            }
            
          },0);
        }
      };

      xhr.open("GET",src, true);
      xhr.overrideMimeType("text/xml");
      xhr.setRequestHeader("Content-type", "text/xml");
      xhr.send(null);
    };

    this.initXml = function(xml_doc) {
      function nsResolver(prefix) {
        var ns = {
          'c' : 'http://www.collada.org/2005/11/COLLADASchema'
        };
        return ns[prefix] || null;
      };

      function getNode(xpathexpr, ctxNode) {
        if (ctxNode == null)
          ctxNode = xml_doc;
        return xml_doc.evaluate(xpathexpr, ctxNode, null,
            XPathResult.FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
      }

      var robot_xml = getNode('//robot');
      if(!robot_xml) {
        console.error("Could not find the 'robot' element in the xml file");
        return false;
      }

      this.clear();
      //console.log('Parsing robot xml');

      // Get robot name
      var name = robot_xml.getAttribute('name');
      if(!name) {
        console.error("No name given for the robot.");
        return false;
      }
      this.name_ = name;

      // Get all Material elements
      for (n in robot_xml.childNodes) {
        var node = robot_xml.childNodes[n];
        if(node.tagName != "material") continue;
        var material_xml = node;
        var material = new UrdfMaterial();

        if(material.initXml(material_xml)) {
          if (this.getMaterial(material.name)) {
            console.error("material " + material.name + "is not unique.");
            return false;
          }
          else {
            this.materials_[material.name] = material;
            //console.log('Succesfully added a new material ' + material.name);
          }
        }  
        else {
          console.error('material xml is not initialized correctly');
          return false;
        }
      }

      for (n in robot_xml.childNodes) {
        var node = robot_xml.childNodes[n];
        if(node.tagName != "link") continue;
        var link_xml = node;
        var link = new UrdfLink(); 

        if(link.initXml(link_xml)) {
          if(this.getLink(link.name)) {
            console.error("link " + link.name + " is not unique. ");
            return false;
          } else {
            //console.log("setting link " + link.name + " material");
            if(link.visual) {
              if(link.visual.material_name.length > 0) {
                if(this.getMaterial(link.visual.material_name)) {
                  //console.log("Setting link " + link.name + " material to " + link.visual.material_name);
                  link.visual.material = this.getMaterial(link.visual.material_name);
                } else {
                  if (link.visual.material) {
                    //console.log("link " + link.name + " material " + link.visual.material_name + " define in Visual.");
                    this.links_[link.visual.material.name] = link.visual.material;
                  } else {
                    console.error("link " + link.name + " material " + link.visual.material_name + " undefined.");
                    return false;
                  }
                }
              }
            }

            this.links_[link.name]= link;
            //console.log('successfully added a new link ' + link.name);
          }
        } else {
          console.error('link xml is not initialied correctly');
          return false;
        }

        if (this.links_.length == 0) {
          console.error('No link elements found in urdf file');
          return false;
        }
      }

        // Get all Joint elements
        for (n in robot_xml.childNodes) {
          var node = robot_xml.childNodes[n];
          if(node.tagName != 'joint') continue;
          var joint_xml = node;
          var joint = new UrdfJoint();

          if(joint.initXml(joint_xml)) {
            if(this.getJoint(joint.name)) {
              console.error('joint ' + joint.name + ' is not unique.');
              return false;
            } else {
              this.joints_[joint.name] = joint;
              //console.log('successfully added a new joint ' + joint.name);
            }
          } else {
            console.error('joint xml is not initialized correctly');
            return false;
          }
        }

        return true;
      };

      this.getMaterial = function(name) {
        return this.materials_[name];
      };

      this.getLink = function(name) {
        return this.links_[name];
      };

      this.getLinks = function(name) {
        return this.links_;
      };

      this.getJoint = function(name) {
        return this.joints_[name];
      };



  };

  return UrdfModel;
}));
