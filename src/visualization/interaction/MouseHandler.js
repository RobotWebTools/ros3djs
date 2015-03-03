/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A handler for mouse events within a 3D viewer.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * renderer - the main renderer
 *   * camera - the main camera in the scene
 *   * rootObject - the root object to check for mouse events
 *   * fallbackTarget - the fallback target, e.g., the camera controls
 */
ROS3D.MouseHandler = function(options) {
  THREE.EventDispatcher.call(this);
  this.renderer = options.renderer;
  this.camera = options.camera;
  this.rootObject = options.rootObject;
  this.fallbackTarget = options.fallbackTarget;
  this.lastTarget = this.fallbackTarget;
  this.dragging = false;

  // listen to DOM events
  var eventNames = [ 'contextmenu', 'click', 'dblclick', 'mouseout', 'mousedown', 'mouseup',
      'mousemove', 'mousewheel', 'DOMMouseScroll', 'touchstart', 'touchend', 'touchcancel',
      'touchleave', 'touchmove' ];
  this.listeners = {};

  // add event listeners for the associated mouse events
  eventNames.forEach(function(eventName) {
    this.listeners[eventName] = this.processDomEvent.bind(this);
    this.renderer.domElement.addEventListener(eventName, this.listeners[eventName], false);
  }, this);
};

/**
 * Process the particular DOM even that has occurred based on the mouse's position in the scene.
 *
 * @param domEvent - the DOM event to process
 */
ROS3D.MouseHandler.prototype.processDomEvent = function(domEvent) {
  // don't deal with the default handler
  domEvent.preventDefault();

  // compute normalized device coords and 3D mouse ray
  var target = domEvent.target;
  var rect = target.getBoundingClientRect();
  var pos_x, pos_y;

  if(domEvent.type.indexOf('touch') !== -1) {
	pos_x = domEvent.changedTouches[0].clientX;
	pos_y = domEvent.changedTouches[0].clientY;
  }
  else {
	pos_x = domEvent.clientX;
	pos_y = domEvent.clientY;
  }
  var left = pos_x - rect.left - target.clientLeft + target.scrollLeft;
  var top = pos_y - rect.top - target.clientTop + target.scrollTop;
  var deviceX = left / target.clientWidth * 2 - 1;
  var deviceY = -top / target.clientHeight * 2 + 1;
  var vector = new THREE.Vector3(deviceX, deviceY, 0.5);
  vector.unproject(this.camera); // THREE r69
  // use the THREE raycaster
  var mouseRaycaster = new THREE.Raycaster(this.camera.position.clone(), vector.sub(
      this.camera.position).normalize());
  mouseRaycaster.linePrecision = 0.001;
  var mouseRay = mouseRaycaster.ray;

  // make our 3d mouse event
  var event3D = {
    mousePos : new THREE.Vector2(deviceX, deviceY),
    mouseRay : mouseRay,
    domEvent : domEvent,
    camera : this.camera,
    intersection : this.lastIntersection
  };

  // if the mouse leaves the dom element, stop everything
  if (domEvent.type === 'mouseout') {
    if (this.dragging) {
      this.notify(this.lastTarget, 'mouseup', event3D);
      this.dragging = false;
    }
    this.notify(this.lastTarget, 'mouseout', event3D);
    this.lastTarget = null;
    return;
  }

  // if the touch leaves the dom element, stop everything
  if (domEvent.type === 'touchleave' || domEvent.type === 'touchend') {
    if (this.dragging) {
      this.notify(this.lastTarget, 'mouseup', event3D);
      this.dragging = false;
    }
    this.notify(this.lastTarget, 'touchend', event3D);
    this.lastTarget = null;
    return;
  }

  // while the user is holding the mouse down, stay on the same target
  if (this.dragging) {
    this.notify(this.lastTarget, domEvent.type, event3D);
    // for check for right or left mouse button
    if ((domEvent.type === 'mouseup' && domEvent.button === 2) || domEvent.type === 'click' || domEvent.type === 'touchend') {
      this.dragging = false;
    }
    return;
  }

  // in the normal case, we need to check what is under the mouse
  target = this.lastTarget;
  var intersections = [];
  intersections = mouseRaycaster.intersectObject(this.rootObject, true);
  if (intersections.length > 0) {
    target = intersections[0].object;
    event3D.intersection = this.lastIntersection = intersections[0];
  } else {
    target = this.fallbackTarget;
  }

  // if the mouse moves from one object to another (or from/to the 'null' object), notify both
  if (target !== this.lastTarget && domEvent.type.match(/mouse/)) {
    var eventAccepted = this.notify(target, 'mouseover', event3D);
    if (eventAccepted) {
      this.notify(this.lastTarget, 'mouseout', event3D);
    } else {
      // if target was null or no target has caught our event, fall back
      target = this.fallbackTarget;
      if (target !== this.lastTarget) {
        this.notify(target, 'mouseover', event3D);
        this.notify(this.lastTarget, 'mouseout', event3D);
      }
    }
  }

  // if the finger moves from one object to another (or from/to the 'null' object), notify both
  if (target !== this.lastTarget && domEvent.type.match(/touch/)) {
    var toucheventAccepted = this.notify(target, domEvent.type, event3D);
    if (toucheventAccepted) {
      this.notify(this.lastTarget, 'touchleave', event3D);
      this.notify(this.lastTarget, 'touchend', event3D);
    } else {
      // if target was null or no target has caught our event, fall back
      target = this.fallbackTarget;
      if (target !== this.lastTarget) {
        this.notify(this.lastTarget, 'touchmove', event3D);
        this.notify(this.lastTarget, 'touchend', event3D);
      }
    }
  }

  // pass through event
  this.notify(target, domEvent.type, event3D);
  if (domEvent.type === 'mousedown' || domEvent.type === 'touchstart' || domEvent.type === 'touchmove') {
    this.dragging = true;
  }
  this.lastTarget = target;
};

/**
 * Notify the listener of the type of event that occurred.
 *
 * @param target - the target of the event
 * @param type - the type of event that occurred
 * @param event3D - the 3D mouse even information
 * @returns if an event was canceled
 */
ROS3D.MouseHandler.prototype.notify = function(target, type, event3D) {
  // ensure the type is set
  event3D.type = type;

  // make the event cancelable
  event3D.cancelBubble = false;
  event3D.stopPropagation = function() {
    event3D.cancelBubble = true;
  };
  // walk up graph until event is canceled or root node has been reached
  event3D.currentTarget = target;
  while (event3D.currentTarget) {
    // try to fire event on object
    if (event3D.currentTarget.dispatchEvent
        && event3D.currentTarget.dispatchEvent instanceof Function) {
      event3D.currentTarget.dispatchEvent(event3D);
      if (event3D.cancelBubble) {
        this.dispatchEvent(event3D);
        return true;
      }
    }
    // walk up
    event3D.currentTarget = event3D.currentTarget.parent;
  }
  return false;
};

THREE.EventDispatcher.prototype.apply( ROS3D.MouseHandler.prototype );
