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
  this.hoverObjs = {};

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
  this.hoverObjs[event.currentTarget.uuid] = event.currentTarget;
};

/**
 * Remove the current target of the mouseover from the hover list.
 *
 * @param event - the event that contains the target of the mouseout
 */
ROS3D.Highlighter.prototype.onMouseOut = function(event) {
  var uuid = event.currentTarget.uuid;
  if (uuid in this.hoverObjs)
  {
    delete this.hoverObjs[uuid];
  }
};


/**
 * Render the highlights for all objects that are currently highlighted.
 *
 * This method should be executed after clearing the renderer and
 * rendering the regular scene.
 *
 * @param scene - the current scene, which should contain the highlighted objects (among others)
 * @param renderer - the renderer used to render the scene.
 * @param camera - the scene's camera
 */
ROS3D.Highlighter.prototype.renderHighlights = function(scene, renderer, camera) {

  // Render highlights by making everything but the highlighted
  // objects invisible...
  this.makeEverythingInvisible(scene);
  this.makeHighlightedVisible(scene);

  // Providing a transparent overrideMaterial...
  var originalOverrideMaterial = scene.overrideMaterial;
  scene.overrideMaterial = new THREE.MeshBasicMaterial({
      fog : false,
      opacity : 0.5,
      transparent : true,
      depthTest : true,
      depthWrite : false,
      polygonOffset : true,
      polygonOffsetUnits : -1,
      side : THREE.DoubleSide
  });

  // And then rendering over the regular scene
  renderer.render(scene, camera);

  // Finally, restore the original overrideMaterial (if any) and
  // object visibility.
  scene.overrideMaterial = originalOverrideMaterial;
  this.restoreVisibility(scene);
};


/**
 * Traverses the given object and makes every object that's a Mesh,
 * Line or Sprite invisible. Also saves the previous visibility state
 * so we can restore it later.
 *
 * @param scene - the object to traverse
 */
ROS3D.Highlighter.prototype.makeEverythingInvisible = function (scene) {
  scene.traverse(function(currentObject) {
    if ( currentObject instanceof THREE.Mesh || currentObject instanceof THREE.Line
         || currentObject instanceof THREE.Sprite ) {
      currentObject.previousVisibility = currentObject.visible;
      currentObject.visible = false;
    }
  });
};


/**
 * Make the objects in the scene that are currently highlighted (and
 * all of their children!) visible.
 *
 * @param scene - the object to traverse
 */
ROS3D.Highlighter.prototype.makeHighlightedVisible = function (scene) {
  var makeVisible = function(currentObject) {
      if ( currentObject instanceof THREE.Mesh || currentObject instanceof THREE.Line
           || currentObject instanceof THREE.Sprite ) {
        currentObject.visible = true;
      }
  };

  for (var uuid in this.hoverObjs) {
    var selectedObject = this.hoverObjs[uuid];
    // Make each selected object and all of its children visible
    selectedObject.visible = true;
    selectedObject.traverse(makeVisible);
  }
};

/**
 * Restore the old visibility state that was saved by
 * makeEverythinginvisible.
 *
 * @param scene - the object to traverse
 */
ROS3D.Highlighter.prototype.restoreVisibility = function (scene) {
  scene.traverse(function(currentObject) {
    if (currentObject.hasOwnProperty('previousVisibility')) {
      currentObject.visible = currentObject.previousVisibility;
    }
  }.bind(this));
};
