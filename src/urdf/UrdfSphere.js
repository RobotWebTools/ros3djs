/**
 * @author Benjamin Pitzer (ben.pitzer@gmail.com)
 * @author Russell Toris - (rctoris@wpi.edu)
 */

/**
 * A Sphere element in a URDF.
 * 
 * @constructor
 * @param options - object with following keys:
 *  * xml - the XML element to parse
 */
ROS3D.UrdfSphere = function(options) {
  var that = this;
  var options = options || {};
  var xml = options.xml;
  this.radius = null;
  this.type = null;

  /**
   * Initialize the element with the given XML node.
   * 
   * @param xml - the XML element to parse
   */
  var initXml = function(xml) {
    that.type = ROS3D.URDF_SPHERE;
    that.radius = parseFloat(xml.getAttribute('radius'));
  };

  // pass it to the XML parser
  initXml(xml);
};
