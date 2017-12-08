/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A mouseover highlighter for 3D objects in the scene.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * mouseHandler - the handler for the mouseover and mouseout events
 */
ROS3D.Highlighter = function(options) {
  options = options || {};
  this.mouseHandler = options.mouseHandler;
  this.hoverObjs = [];

  // bind the mouse events
  this.mouseHandler.addEventListener('mouseover', this.onMouseOver.bind(this));
  this.mouseHandler.addEventListener('mouseout', this.onMouseOut.bind(this));
};

/**
 * Add the current target of the mouseover to the hover list.
 *
 * @param event - the event that contains the target of the mouseover
 */
ROS3D.Highlighter.prototype.onMouseOver = function(event) {
  // this.hoverObjs.push(event.currentTarget);
  this.highlightObject(event.currentTarget, true);
};

/**
 * Remove the current target of the mouseover from the hover list.
 *
 * @param event - the event that contains the target of the mouseout
 */
ROS3D.Highlighter.prototype.onMouseOut = function(event) {
  // this.hoverObjs.splice(this.hoverObjs.indexOf(event.currentTarget), 1);
  this.highlightObject(event.currentTarget, false);
};


/**
 * Highlight and unhighlight the given object
 *
 * @param object - the target object to (un)highlight
 * @param flag - whether to highlight or unhighlight
 */
ROS3D.Highlighter.prototype.highlightObject = function (object, flag) {
  if(object.material === undefined) {
    if(object.children === undefined) {
      return;
    }
    else {
      for(var c = 0 ; c < object.children.length; c++) {
        this.highlightObject(object.children[c], flag);
      }
    }
  }
  else {
    if(flag) {
      object.currentMaterial = object.material;
      object.material = new THREE.MeshBasicMaterial({
        fog : false,
        opacity : 0.5,
        depthTest : true,
        depthWrite : false,
        polygonOffset : true,
        polygonOffsetUnits : -1,
        side : THREE.DoubleSide
      });
    }
    else {
      object.material = object.currentMaterial;
    }
  }
};
