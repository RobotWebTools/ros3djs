(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfvisual'],factory);
  }
  else {
    root.UrdfLink = factory(root.UrdfVisual);
  }
}(this, function(UrdfVisual) {
  var UrdfLink = function() { 

		// members
		this.name = "";
		this.inertial = null;
		this.visual = null;
		this.collision = null;
		this.parent_link = null;
		this.parent_joint = null;
		this.child_joints = [];
		this.child_links = [];

    // methods
    this.clear = function () {
  		this.name = "";	
  		this.inertial = null;
	  	this.visual = null;
  		this.collision = null;
	  	this.parent_joint = null;
  		this.child_joints = [];
  		this.child_links = [];
    };

    this.getParent = function() {
		  return this.parent_link;
  	};

    this.setParent = function(parent) {
  		this.parent_link = parent;
    };

    this.getParentJoint = function() {
  		return this.parent_joint;
    };

	  this.setParentJoint = function(parent) {
  		this.parent_joint = parent;
	  };

	  this.addChild = function(child) {
		  this.child_links.push(child);
  	};

	  this.addChildJoint = function(child) {
		  this.child_joints.push(child);
  	};

  	this.initXml = function(xml) {

  		this.clear();

	  	var name = xml.getAttribute("name");
  		if (!name)
	  	{
		  	console.error("No name given for the link.");
			  return false;
  		}
	  	this.name = name;

  		// Inertial (optional)
//		TiXmlElement *i = config->FirstChildElement("inertial");
//		if (i)
//		{
//		inertial.reset(new Inertial);
//		if (!inertial->initXml(i))
//		{
//		ROS_ERROR("Could not parse inertial element for Link '%s'", this->name.c_str());
//		return false;
//		}
//		}

  		// Visual (optional)
	  	var visuals = xml.getElementsByTagName("visual");
		  if(visuals.length>0)
  		{
	  		var visual_xml = visuals[0];
  			var visual = new UrdfVisual();
	  		if (!visual.initXml(visual_xml))
		  	{
			  	console.error("Could not parse visual element for Link " + this.name);
				  return false;
  			}
	  		this.visual = visual;
		  }

  		// Collision (optional)
//		TiXmlElement *col = xml.FirstChildElement("collision");
//		if (col)
//		{
//		collision.reset(new Collision);
//		if (!collision.initXml(col))
//		{
//		console.log("Could not parse collision element for Link '%s'", this.name.c_str());
//		return false;
//		}
//		}

  		return true;
	  };
  };
  
  return UrdfLink;
}));
