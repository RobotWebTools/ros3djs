(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['./urdfsphere','./urdfbox','./urdfcylinder','./urdfmesh','./urdfpose','./urdfmaterial'], factory);
  }
  else {
    root.UrdfVisual = factory(root.UrdfSphere,root.UrdfBox,root.UrdfCylinder,root.UrdfMesh,root.UrdfPose,root.UrdfMaterial);
  }
}(this, function(UrdfSphere,UrdfBox,UrdfCylinder,UrdfMesh,UrdfPose,UrdfMaterial) {

  var UrdfVisual = function() {
  
	  // members
  	this.origin = null;
	  this.geometry = null;
  	this.material_name = "";
	  this.material = null;

    // methods
	  this.clear = function() {
		  this.origin = null;
  		this.geometry = null;
	  	this.material_name = "";
		  this.material = null;
  	};

	  this.initXml = function (visual_xml) {
      function parseGeometry(geometry_xml)
      {
        var geometry = null;
        var shape = null;

        for (n in geometry_xml.childNodes) {
          var node = geometry_xml.childNodes[n];
          if(node.nodeType == 1) {
            shape = node;
            break;
          }
        }

        if (!shape)
        {
          console.error("Geometry tag contains no child element.");
        }

        var type_name = shape.nodeName;
        if (type_name == "sphere")
          geometry = new UrdfSphere();
        else if (type_name == "box")
          geometry = new UrdfBox();
        else if (type_name == "cylinder")
          geometry = new UrdfCylinder();
        else if (type_name == "mesh")
          geometry = new UrdfMesh();
        else
        {
          console.error("Unknown geometry type " + type_name);
          return geometry;
        }

        // clear geom object when fails to initialize
        if (!geometry.initXml(shape)){
          console.error("Geometry failed to parse");
          geometry = null;
        }

        return geometry;
      }


      this.clear();

      // Origin
      var origins = visual_xml.getElementsByTagName("origin");
      if(origins.length==0)
      {
        ////console.log("Origin tag not present for visual element, using default (Identity)");
        this.origin = new UrdfPose();
      }
      else 
      {
        var origin = new UrdfPose();
        if (!origin.initXml(origins[0])) {
          console.error("Visual has a malformed origin tag");
          return false;
        }
        this.origin = origin;
      }

      // Geometry
      var geoms = visual_xml.getElementsByTagName("geometry");
      //console.log(geoms);
      if(geoms.length==0)
      {
        ////console.log("Geometry tag not present for visual element.");
      }
      else 
      {
        var geometry = parseGeometry(geoms[0]);    
        if (!geometry) {
          console.error("Malformed geometry for Visual element.");
          return false;
        }
        this.geometry = geometry;
      }

      // Material
      var materials = visual_xml.getElementsByTagName("material");
      if(materials.length==0)
      {
        //console.log("visual element has no material tag.");
      }
      else
      {
        // get material name
        var material_xml = materials[0];
        var material = new UrdfMaterial();

        if (!material_xml.getAttribute("name"))
        {
          console.error("Visual material must contain a name attribute");
          return false;
        }
        this.material_name = material_xml.getAttribute("name");

        // try to parse material element in place
        if (!material.initXml(material_xml))
        {
          //console.log("Could not parse material element in Visual block, maybe defined outside.");
        }
        else
        {
          //console.log("Parsed material element in Visual block.");
          this.material = material;
        }
      }
      return true;
    };
  };

  return UrdfVisual;

}));




