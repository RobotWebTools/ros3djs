/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * The main interactive marker object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * handle - the ROS3D.InteractiveMarkerHandle for this marker
 *  * camera - the main camera associated with the viewer for this marker
 *  * path (optional) - the base path to any meshes that will be loaded
 *  * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER)
 */
ROS3D.InteractiveMarker = function(options) {
  THREE.Object3D.call(this);

  var that = this;
  options = options || {};
  var handle = options.handle;
  this.name = handle.name;
  var camera = options.camera;
  var path = options.path || '/';
  var loader = options.loader;
  this.dragging = false;

  // set the initial pose
  this.onServerSetPose({
    pose : handle.pose
  });

  // information on where the drag started
  this.dragStart = {
    position : new THREE.Vector3(),
    orientation : new THREE.Quaternion(),
    positionWorld : new THREE.Vector3(),
    orientationWorld : new THREE.Quaternion(),
    event3d : {}
  };

  // add each control message
  handle.controls.forEach(function(controlMessage) {
    that.add(new ROS3D.InteractiveMarkerControl({
      parent : that,
      handle : handle,
      message : controlMessage,
      camera : camera,
      path : path,
      loader : loader
    }));
  });

  // check for any menus
  if (handle.menuEntries.length > 0) {
    this.menu = new ROS3D.InteractiveMarkerMenu({
      menuEntries : handle.menuEntries,
      menuFontSize : handle.menuFontSize
    });

    // forward menu select events
    this.menu.addEventListener('menu-select', function(event) {
      that.dispatchEvent(event);
    });
  }
};
ROS3D.InteractiveMarker.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Show the interactive marker menu associated with this marker.
 *
 * @param control - the control to use
 * @param event - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.showMenu = function(control, event) {
  if (this.menu) {
    this.menu.show(control, event);
  }
};

/**
 * Move the axis based on the given event information.
 *
 * @param control - the control to use
 * @param origAxis - the origin of the axis
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.moveAxis = function(control, origAxis, event3d) {
  if (this.dragging) {
    var currentControlOri = control.currentControlOri;
    var axis = origAxis.clone().applyQuaternion(currentControlOri);
    // get move axis in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var axisWorld = axis.clone().applyQuaternion(this.dragStart.orientationWorld.clone());

    var axisRay = new THREE.Ray(originWorld, axisWorld);

    // find closest point to mouse on axis
    var t = ROS3D.closestAxisPoint(axisRay, event3d.camera, event3d.mousePos);

    // offset from drag start position
    var p = new THREE.Vector3();
    p.addVectors(this.dragStart.position, axis.clone().applyQuaternion(this.dragStart.orientation)
        .multiplyScalar(t));
    this.setPosition(control, p);


    event3d.stopPropagation();
  }
};


/**
 * Move with respect to the plane based on the contorl and event.
 *
 * @param control - the control to use
 * @param origNormal - the normal of the origin
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.move3d = function(control, origNormal, event3d) {
  // by default, move in a plane
  if (this.dragging) {

    if(control.isShift){
      // this doesn't work
      // // use the camera position and the marker position to determine the axis
      // var newAxis = control.camera.position.clone();
      // newAxis.sub(this.position);
      // // now mimic same steps constructor uses to create origAxis
      // var controlOri = new THREE.Quaternion(newAxis.x, newAxis.y,
      //     newAxis.z, 1);
      // controlOri.normalize();
      // var controlAxis = new THREE.Vector3(1, 0, 0);
      // controlAxis.applyQuaternion(controlOri);
      // origAxis = controlAxis;
    }else{
      // we want to use the origin plane that is closest to the camera
      var cameraVector = control.camera.getWorldDirection();
      var x = Math.abs(cameraVector.x);
      var y = Math.abs(cameraVector.y);
      var z = Math.abs(cameraVector.z);
      var controlOri = new THREE.Quaternion(1, 0, 0, 1);
      if(y > x && y > z){
        // orientation for the control
        controlOri = new THREE.Quaternion(0, 0, 1, 1);
      }else if(z > x && z > y){
        // orientation for the control
        controlOri = new THREE.Quaternion(0, 1, 0, 1);
      }
      controlOri.normalize();

      // transform x axis into local frame
      origNormal = new THREE.Vector3(1, 0, 0);
      origNormal.applyQuaternion(controlOri);
      this.movePlane(control, origNormal, event3d);
    }
  }
};

/**
 * Move with respect to the plane based on the contorl and event.
 *
 * @param control - the control to use
 * @param origNormal - the normal of the origin
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.movePlane = function(control, origNormal, event3d) {
  if (this.dragging) {
    var currentControlOri = control.currentControlOri;
    var normal = origNormal.clone().applyQuaternion(currentControlOri);
    // get plane params in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var normalWorld = normal.clone().applyQuaternion(this.dragStart.orientationWorld);

    // intersect mouse ray with plane
    var intersection = ROS3D.intersectPlane(event3d.mouseRay, originWorld, normalWorld);

    // offset from drag start position
    var p = new THREE.Vector3();
    p.subVectors(intersection, originWorld);
    p.add(this.dragStart.positionWorld);
    this.setPosition(control, p);
    event3d.stopPropagation();
  }
};

/**
 * Rotate based on the control and event given.
 *
 * @param control - the control to use
 * @param origOrientation - the orientation of the origin
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.rotateAxis = function(control, origOrientation, event3d) {
  if (this.dragging) {
    control.updateMatrixWorld();

    var currentControlOri = control.currentControlOri;
    var orientation = currentControlOri.clone().multiply(origOrientation.clone());

    var normal = (new THREE.Vector3(1, 0, 0)).applyQuaternion(orientation);

    // get plane params in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var normalWorld = normal.applyQuaternion(this.dragStart.orientationWorld);

    // intersect mouse ray with plane
    var intersection = ROS3D.intersectPlane(event3d.mouseRay, originWorld, normalWorld);

    // offset local origin to lie on intersection plane
    var normalRay = new THREE.Ray(this.dragStart.positionWorld, normalWorld);
    var rotOrigin = ROS3D.intersectPlane(normalRay, originWorld, normalWorld);

    // rotates from world to plane coords
    var orientationWorld = this.dragStart.orientationWorld.clone().multiply(orientation);
    var orientationWorldInv = orientationWorld.clone().inverse();

    // rotate original and current intersection into local coords
    intersection.sub(rotOrigin);
    intersection.applyQuaternion(orientationWorldInv);

    var origIntersection = this.dragStart.event3d.intersection.point.clone();
    origIntersection.sub(rotOrigin);
    origIntersection.applyQuaternion(orientationWorldInv);

    // compute relative 2d angle
    var a1 = Math.atan2(intersection.y, intersection.z);
    var a2 = Math.atan2(origIntersection.y, origIntersection.z);
    var a = a2 - a1;

    var rot = new THREE.Quaternion();
    rot.setFromAxisAngle(normal, a);

    // rotate
    this.setOrientation(control, rot.multiply(this.dragStart.orientationWorld));

    // offset from drag start position
    event3d.stopPropagation();
  }
};

/**
 * Dispatch the given event type.
 *
 * @param type - the type of event
 * @param control - the control to use
 */
ROS3D.InteractiveMarker.prototype.feedbackEvent = function(type, control) {
  this.dispatchEvent({
    type : type,
    position : this.position.clone(),
    orientation : this.quaternion.clone(),
    controlName : control.name
  });
};

/**
 * Start a drag action.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.startDrag = function(control, event3d) {
  if (event3d.domEvent.button === 0) {
    event3d.stopPropagation();
    this.dragging = true;
    this.updateMatrixWorld(true);
    var scale = new THREE.Vector3();
    this.matrixWorld
        .decompose(this.dragStart.positionWorld, this.dragStart.orientationWorld, scale);
    this.dragStart.position = this.position.clone();
    this.dragStart.orientation = this.quaternion.clone();
    this.dragStart.event3d = event3d;

    this.feedbackEvent('user-mousedown', control);
  }
};

/**
 * Stop a drag action.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.stopDrag = function(control, event3d) {
  if (event3d.domEvent.button === 0) {
    event3d.stopPropagation();
    this.dragging = false;
    this.dragStart.event3d = {};
    this.onServerSetPose(this.bufferedPoseEvent);
    this.bufferedPoseEvent = undefined;

    this.feedbackEvent('user-mouseup', control);
  }
};

/**
 * Handle a button click.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.buttonClick = function(control, event3d) {
  event3d.stopPropagation();
  this.feedbackEvent('user-button-click', control);
};

/**
 * Handle a user pose change for the position.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.setPosition = function(control, position) {
  this.position.copy(position);
  this.feedbackEvent('user-pose-change', control);
};

/**
 * Handle a user pose change for the orientation.
 *
 * @param control - the control to use
 * @param event3d - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.setOrientation = function(control, orientation) {
  orientation.normalize();
  this.quaternion.copy(orientation);
  this.feedbackEvent('user-pose-change', control);
};

/**
 * Update the marker based when the pose is set from the server.
 *
 * @param event - the event that caused this
 */
ROS3D.InteractiveMarker.prototype.onServerSetPose = function(event) {
  if (event !== undefined) {
    // don't update while dragging
    if (this.dragging) {
      this.bufferedPoseEvent = event;
    } else {
      var pose = event.pose;
      this.position.copy(pose.position);
      this.quaternion.copy(pose.orientation);
      this.updateMatrixWorld(true);
    }
  }
};

/**
 * Free memory of elements in this marker.
 */
ROS3D.InteractiveMarker.prototype.dispose = function() {
  var that = this;
  this.children.forEach(function(intMarkerControl) {
    intMarkerControl.children.forEach(function(marker) {
      marker.dispose();
      intMarkerControl.remove(marker);
    });
    that.remove(intMarkerControl);
  });
};

Object.assign(ROS3D.InteractiveMarker.prototype, THREE.EventDispatcher.prototype);
