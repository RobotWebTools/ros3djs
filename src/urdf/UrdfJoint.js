/**
 * Class to handle urdf joints
 * @class
 * @augments Class
 */
ROS3D.UrdfJoint = function() {
  // members
  this.name = "";
  this.JointTypes = {
    "unknown" : 0,
    "revolute" : 1,
    "continuous" : 3,
    "prismatic" : 4,
    "floating" : 5,
    "planar" : 6,
    "fixed" : 7
  };
  this.type = this.JointTypes.unknown;
  this.axis = new UrdfVector3();
  this.child_link_name = "";
  this.parent_link_name = "";
  this.parent_to_joint_origin_transform = new UrdfPose();
  this.dynamics = null;
  this.limits = null;
  this.safety = null;
  this.calibration = null;
  this.mimic = null;

  // methods
  this.clear = function() {
    this.name = "";
    this.axis.clear();
    this.child_link_name = "";
    this.parent_link_name = "";
    this.parent_to_joint_origin_transform.clear();
    this.dynamics = null;
    this.limits = null;
    this.safety = null;
    this.calibration = null;
    this.mimic = null;
    this.type = this.JointTypes.unknown;
  };

  this.init = function(xml) {
    this.clear();

    // Get Joint Name
    var name = xml.getAttribute("name");
    if (!name) {
      console.error("unnamed joint found");
      return false;
    }
    this.name = name;

    // Get transform from Parent Link to Joint Frame
    var origins = xml.getElementsByTagName("origin");
    if (origins.length == 0) {
      // console.log("Joint " + this.name + " missing origin tag under parent describing transform
      // from Parent Link to Joint Frame, (using Identity transform).");
      this.parent_to_joint_origin_transform.clear();
    } else {
      origin_xml = origins[0];
      if (!this.parent_to_joint_origin_transform.initXml(origin_xml)) {
        console.error("Malformed parent origin element for joint " + this.name);
        this.parent_to_joint_origin_transform.clear();
        return false;
      }
    }

    // Get Parent Link
    var parents = xml.getElementsByTagName("parent");
    if (parents.length > 0) {
      var parent_xml = parents[0];
      var pname = parent_xml.getAttribute("link");
      if (!pname)
        ros_info("no parent link name specified for Joint link " + this.name
            + ". this might be the root?");
      else {
        this.parent_link_name = pname;
      }
    }

    // Get Child Link
    var children = xml.getElementsByTagName("child");
    if (children.length > 0) {
      var child_xml = children[0];
      var pname = child_xml.getAttribute("link");
      if (!pname)
        ros_info("no child link name specified for Joint link " + this.name);
      else {
        this.child_link_name = pname;
      }
    }

    // Get Joint type
    var type_char = xml.getAttribute("type");
    if (!type_char) {
      console.error("joint " + this.name + " has no type, check to see if it's a reference.");
      return false;
    }
    var type_str = type_char;
    if (type_str == "planar")
      this.type = this.JointTypes.planar;
    else if (type_str == "floating")
      this.type = this.JointTypes.floating;
    else if (type_str == "revolute")
      this.type = this.JointTypes.revolute;
    else if (type_str == "continuous")
      this.type = this.JointTypes.continuous;
    else if (type_str == "prismatic")
      this.type = this.JointTypes.prismatic;
    else if (type_str == "fixed")
      this.type = this.JointTypes.fixed;
    else {
      console.error("Joint " + this.name + " has no known type " + type_str);
      return false;
    }

    // Get Joint Axis
    if (this.type != this.JointTypes.floating && this.type != this.JointTypes.fixed) {
      // axis
      var axis = xml.getElementsByTagName("axis");
      if (axis.length == 0) {
        // console.log("no axis elemement for Joint link " + this.name + ", defaulting to (1,0,0)
        // axis");
        this.axis = new Vector3(1.0, 0.0, 0.0);
      } else {
        var axis_xml = axis[0];
        if (!axis_xml.getAttribute("xyz")) {
          console.error("no xyz attribute for axis element for Joint link " + this.name);
        } else {
          if (!this.axis.initString(axis_xml.getAttribute("xyz"))) {
            console.error("Malformed axis element for joint " + this.name);
            this.axis.clear();
            return false;
          }
        }
      }
    }

    return true;
  };
};
