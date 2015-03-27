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
  var mouseHandler = options.mouseHandler;
  this.hoverObjs = [];

  // bind the mouse events
  mouseHandler.addEventListener('mouseover', this.onMouseOver.bind(this));
  mouseHandler.addEventListener('mouseout', this.onMouseOut.bind(this));
};

/**
 * Add the current target of the mouseover to the hover list.
 *
 * @param event - the event that contains the target of the mouseover
 */
ROS3D.Highlighter.prototype.onMouseOver = function(event) {
  this.hoverObjs.push(event.currentTarget);
};

/**
 * Remove the current target of the mouseover from the hover list.
 *
 * @param event - the event that contains the target of the mouseout
 */
ROS3D.Highlighter.prototype.onMouseOut = function(event) {
  this.hoverObjs.splice(this.hoverObjs.indexOf(event.currentTarget), 1);
};

/**
 * Add all corresponding webgl objects in the given scene and add them to the given render list.
 *
 * @param scene - the scene to check for webgl objects
 * @param objects - the objects list to check
 * @param renderList - the list to add to
 */
ROS3D.Highlighter.prototype.getWebglObjects = function(scene, objects, renderList) {
  var objlist = scene.__webglObjects;
  // get corresponding webgl objects
  for ( var c = 0; c < objects.length; c++) {
    if (objects[c]) {
      for ( var o = objlist.length - 1; o >= 0; o--) {
        if (objlist[o].object === objects[c]) {
          renderList.push(objlist[o]);
          break;
        }
      }
      // recurse into children
      this.getWebglObjects(scene, objects[c].children, renderList);
    }
  }
};

/**
 * Render highlighted objects in the scene.
 *
 * @param renderer - the renderer to use
 * @param scene - the scene to use
 * @param camera - the camera to use
 */
ROS3D.Highlighter.prototype.renderHighlight = function(renderer, scene, camera) {
  // get webgl objects
  var renderList = [];
  this.getWebglObjects(scene, this.hoverObjs, renderList);

  if ( renderList.length === 0 ) {
    return;
  }

  // define highlight material
  scene.overrideMaterial = new THREE.MeshBasicMaterial({
    fog : false,
    opacity : 0.5,
    depthTest : true,
    depthWrite : false,
    polygonOffset : true,
    polygonOffsetUnits : -1,
    side : THREE.DoubleSide
  });

  // swap render lists, render, undo
  var oldWebglObjects = scene.__webglObjects;
  scene.__webglObjects = renderList;

  renderer.render(scene, camera);

  scene.__webglObjects = oldWebglObjects;
  scene.overrideMaterial = null;
};
