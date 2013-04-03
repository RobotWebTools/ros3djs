/**
* @author Russell Toris - rctoris@wpi.edu
* @author David Gossow - dgossow@willowgarage.com
*/

var ROS3D = ROS3D || {
  REVISION : '2'
};

// Marker types
ROS3D.MARKER_ARROW = 0;
ROS3D.MARKER_CUBE = 1;
ROS3D.MARKER_SPHERE = 2;
ROS3D.MARKER_CYLINDER = 3;
ROS3D.MARKER_LINE_STRIP = 4;
ROS3D.MARKER_LINE_LIST = 5;
ROS3D.MARKER_CUBE_LIST = 6;
ROS3D.MARKER_SPHERE_LIST = 7;
ROS3D.MARKER_POINTS = 8;
ROS3D.MARKER_TEXT_VIEW_FACING = 9;
ROS3D.MARKER_MESH_RESOURCE = 10;
ROS3D.MARKER_TRIANGLE_LIST = 11;

// Interactive marker feedback types
ROS3D.INTERACTIVE_MARKER_KEEP_ALIVE = 0;
ROS3D.INTERACTIVE_MARKER_POSE_UPDATE = 1;
ROS3D.INTERACTIVE_MARKER_MENU_SELECT = 2;
ROS3D.INTERACTIVE_MARKER_BUTTON_CLICK = 3;
ROS3D.INTERACTIVE_MARKER_MOUSE_DOWN = 4;
ROS3D.INTERACTIVE_MARKER_MOUSE_UP = 5;

// Interactive marker control types
ROS3D.INTERACTIVE_MARKER_NONE = 0;
ROS3D.INTERACTIVE_MARKER_MENU = 1;
ROS3D.INTERACTIVE_MARKER_BUTTON = 2;
ROS3D.INTERACTIVE_MARKER_MOVE_AXIS = 3;
ROS3D.INTERACTIVE_MARKER_MOVE_PLANE = 4;
ROS3D.INTERACTIVE_MARKER_ROTATE_AXIS = 5;
ROS3D.INTERACTIVE_MARKER_MOVE_ROTATE = 6;

// Interactive marker rotation behavior
ROS3D.INTERACTIVE_MARKER_INHERIT = 0;
ROS3D.INTERACTIVE_MARKER_FIXED = 1;
ROS3D.INTERACTIVE_MARKER_VIEW_FACING = 2;

/**
 * Create a THREE material based on the given RGBA values.
 * 
 * @param r - the red value
 * @param g - the green value
 * @param b - the blue value
 * @param a - the alpha value
 * @returns the THREE material
 */
ROS3D.makeColorMaterial = function(r, g, b, a) {
  var color = new THREE.Color();
  color.setRGB(r, g, b);
  if (a <= 0.99) {
    return new THREE.MeshBasicMaterial({
      color : color.getHex(),
      opacity : a + 0.1,
      transparent : true,
      depthWrite : true,
      blendSrc : THREE.SrcAlphaFactor,
      blendDst : THREE.OneMinusSrcAlphaFactor,
      blendEquation : THREE.ReverseSubtractEquation,
      blending : THREE.NormalBlending
    });
  } else {
    return new THREE.MeshLambertMaterial({
      color : color.getHex(),
      opacity : a,
      blending : THREE.NormalBlending
    });
  }
};

/**
 * Return the intersection between the mouseray and the plane.
 * 
 * @param mouseRay - the mouse ray
 * @param planeOrigin - the origin of the plane
 * @param planeNormal - the normal of the plane
 * @returns the intersection point
 */
ROS3D.intersectPlane = function(mouseRay, planeOrigin, planeNormal) {
  var vector = new THREE.Vector3();
  var intersectPoint = new THREE.Vector3();
  vector.subVectors(planeOrigin, mouseRay.origin);
  dot = mouseRay.direction.dot(planeNormal);

  // bail if ray and plane are parallel
  if (Math.abs(dot) < mouseRay.precision) {
    return undefined;
  }

  // calc distance to plane
  scalar = planeNormal.dot(vector) / dot;

  intersectPoint.addVectors(mouseRay.origin, mouseRay.direction.clone().multiplyScalar(scalar));
  return intersectPoint;
};

/**
 * Find the closest point on targetRay to any point on mouseRay. Math taken from 
 * http://paulbourke.net/geometry/lineline3d/
 * 
 * @param targetRay - the target ray to use
 * @param mouseRay - the mouse ray
 * @param the closest point between the two rays
 */
ROS3D.findClosestPoint = function(targetRay, mouseRay) {
  var v13 = new THREE.Vector3;
  v13.subVectors(targetRay.origin, mouseRay.origin);
  var v43 = mouseRay.direction.clone();
  var v21 = targetRay.direction.clone();
  var d1343 = v13.dot(v43);
  var d4321 = v43.dot(v21);
  var d1321 = v13.dot(v21);
  var d4343 = v43.dot(v43);
  var d2121 = v21.dot(v21);

  var denom = d2121 * d4343 - d4321 * d4321;
  // check within a delta
  if (Math.abs(denom) <= 0.0001) {
    return undefined;
  }
  var numer = d1343 * d4321 - d1321 * d4343;

  var mua = numer / denom;
  return mua;
};

/**
 * Find the closest point between the axis and the mouse.
 * 
 * @param axisRay - the ray from the axis
 * @param camera - the camera to project from
 * @param mousePos - the mouse position
 * @returns the closest axis point
 */
ROS3D.closestAxisPoint = function(axisRay, camera, mousePos) {
  var projector = new THREE.Projector();

  // project axis onto screen
  var o = axisRay.origin.clone();
  projector.projectVector(o, camera);
  var o2 = axisRay.direction.clone().add(axisRay.origin);
  projector.projectVector(o2, camera);

  // d is the axis vector in screen space (d = o2-o)
  var d = o2.clone().sub(o);

  // t is the 2d ray param of perpendicular projection of mousePos onto o
  var tmp = new THREE.Vector2;
  // (t = (mousePos - o) * d / (d*d))
  var t = tmp.subVectors(mousePos, o).dot(d) / d.dot(d);

  // mp is the final 2d-projected mouse pos (mp = o + d*t)
  var mp = new THREE.Vector2;
  mp.addVectors(o, d.clone().multiplyScalar(t));

  // go back to 3d by shooting a ray
  var vector = new THREE.Vector3(mp.x, mp.y, 0.5);
  projector.unprojectVector(vector, camera);
  var mpRay = new THREE.Ray(camera.position, vector.sub(camera.position).normalize());

  return ROS3D.findClosestPoint(axisRay, mpRay);
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * The main interactive marker object.
 *
 * @constructor
 * @param options - object with following keys:
 *  * handle - the ROS3D.InteractiveMarkerHandle for this marker
 *  * camera - the main camera associated with the viewer for this marker
 *  * path (optional) - the base path to any meshes that will be loaded
 */
ROS3D.InteractiveMarker = function(options) {
  THREE.Object3D.call(this);
  THREE.EventDispatcher.call(this);

  var that = this;
  var options = options || {};
  var handle = options.handle;
  this.name = handle.name;
  var camera = options.camera;
  var path = options.path || '/';
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
      message : controlMessage,
      camera : camera,
      path : path
    }));
  });

  // check for any menus
  if (handle.menuEntries.length > 0) {
    this.menu = new ROS3D.InteractiveMarkerMenu({
      menuEntries : handle.menuEntries
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
    var p = new THREE.Vector3;
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
    var p = new THREE.Vector3;
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
  this.position = position;
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
  this.quaternion = orientation;
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

      this.position.x = pose.position.x;
      this.position.y = pose.position.y;
      this.position.z = pose.position.z;

      this.useQuaternion = true;
      this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y,
          pose.orientation.z, pose.orientation.w);

      this.updateMatrixWorld(true);
    }
  }
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A client for an interactive marker topic.
 *
 * @constructor
 * @param options - object with following keys:
 *  * ros - a handle to the ROS connection
 *  * tfClient - a handle to the TF client
 *  * topic (optional) - the topic to subscribe to, like '/basic_controls'
 *  * path (optional) - the base path to any meshes that will be loaded
 *  * camera - the main camera associated with the viewer for this marker client
 *  * rootObject (optional) - the root THREE 3D object to render to
 */
ROS3D.InteractiveMarkerClient = function(options) {
  var that = this;
  var options = options || {};
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.topic = options.topic;
  this.path = options.path || '/';
  this.camera = options.camera;
  this.rootObject = options.rootObject || new THREE.Object3D();

  this.interactiveMarkers = {};
  this.updateTopic = null;
  this.feedbackTopic = null;

  // check for an initial topic
  if (this.topic) {
    this.subscribe(this.topic);
  }
};

/**
 * Subscribe to the given interactive marker topic. This will unsubscribe from any current topics.
 * 
 * @param topic - the topic to subscribe to, like '/basic_controls'
 */
ROS3D.InteractiveMarkerClient.prototype.subscribe = function(topic) {
  // unsubscribe to the other topics
  this.unsubscribe();

  this.updateTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : topic + '/tunneled/update',
    messageType : 'visualization_msgs/InteractiveMarkerUpdate',
    compression : 'png'
  });
  this.updateTopic.subscribe(this.processUpdate.bind(this));

  this.feedbackTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : topic + '/feedback',
    messageType : 'visualization_msgs/InteractiveMarkerFeedback',
    compression : 'png'
  });
  this.feedbackTopic.advertise();

  this.initService = new ROSLIB.Service({
    ros : this.ros,
    name : topic + '/tunneled/get_init',
    serviceType : 'demo_interactive_markers/GetInit'
  });
  var request = new ROSLIB.ServiceRequest({});
  this.initService.callService(request, this.processInit.bind(this));
};

/**
 * Unsubscribe from the current interactive marker topic.
 */
ROS3D.InteractiveMarkerClient.prototype.unsubscribe = function() {
  if (this.updateTopic) {
    this.updateTopic.unsubscribe();
  }
  if (this.feedbackTopic) {
    this.feedbackTopic.unadvertise();
  }
  // erase all markers
  for (intMarkerName in this.interactiveMarkers) {
    this.eraseIntMarker(intMarkerName);
  }
  this.interactiveMarkers = {};
};

/**
 * Process the given interactive marker initialization message.
 * 
 * @param initMessage - the interactive marker initialization message to process
 */
ROS3D.InteractiveMarkerClient.prototype.processInit = function(initMessage) {
  var message = initMessage.msg;

  // erase any old markers
  message.erases = [];
  for (intMarkerName in this.interactiveMarkers) {
    message.erases.push(intMarkerName);
  };
  message.poses = [];

  // treat it as an update
  this.processUpdate(message);
};

/**
 * Process the given interactive marker update message.
 * 
 * @param initMessage - the interactive marker update message to process
 */
ROS3D.InteractiveMarkerClient.prototype.processUpdate = function(message) {
  var that = this;

  // erase any markers
  message.erases.forEach(function(name) {
    var marker = that.interactiveMarkers[name];
    that.eraseIntMarker(name);
  });

  // updates marker poses
  message.poses.forEach(function(poseMessage) {
    var marker = that.interactiveMarkers[poseMessage.name];
    if (marker) {
      marker.setPoseFromServer(poseMessage.pose);
    }
  });

  // add new markers
  message.markers.forEach(function(msg) {
    // get rid of anything with the same name
    var oldhandle = that.interactiveMarkers[msg.name];
    if (oldhandle) {
      that.eraseIntMarker(oldhandle.name);
    }

    // create the handle
    var handle = new ROS3D.InteractiveMarkerHandle({
      message : msg,
      feedbackTopic : that.feedbackTopic,
      tfClient : that.tfClient
    });
    that.interactiveMarkers[msg.name] = handle;

    // create the actual marker
    var intMarker = new ROS3D.InteractiveMarker({
      handle : handle,
      camera : that.camera,
      path : that.path
    });
    // add it to the scene
    that.rootObject.add(intMarker);

    // listen for any pose updates from the server
    handle.on('pose', function(pose) {
      intMarker.onServerSetPose({
        pose : pose
      });
    });

    intMarker.addEventListener('user-pose-change', handle.setPoseFromClient.bind(handle));
    intMarker.addEventListener('user-mousedown', handle.onMouseDown.bind(handle));
    intMarker.addEventListener('user-mouseup', handle.onMouseUp.bind(handle));
    intMarker.addEventListener('user-button-click', handle.onButtonClick.bind(handle));
    intMarker.addEventListener('menu-select', handle.onMenuSelect.bind(handle));

    // now list for any TF changes
    handle.subscribeTf();
  });
};

/**
 * Erase the interactive marker with the given name.
 * 
 * @param intMarkerName - the interactive marker name to delete
 */
ROS3D.InteractiveMarkerClient.prototype.eraseIntMarker = function(intMarkerName) {
  if (this.interactiveMarkers[intMarkerName]) {
    delete this.interactiveMarkers[intMarkerName];
  }
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * The main marker control object for an interactive marker.
 *
 * @constructor
 * @param options - object with following keys:
 *  * parent - the parent of this control
 *  * message - the interactive marker control message
 *  * camera - the main camera associated with the viewer for this marker client
 *  * path (optional) - the base path to any meshes that will be loaded
 */
ROS3D.InteractiveMarkerControl = function(options) {
  var that = this;
  THREE.Object3D.call(this);
  THREE.EventDispatcher.call(this);

  var options = options || {};
  this.parent = options.parent;
  var message = options.message;
  this.name = message.name;
  this.camera = options.camera;
  this.path = options.path || '/';
  this.dragging = false;

  // orientation for the control
  var controlOri = new THREE.Quaternion(message.orientation.x, message.orientation.y,
      message.orientation.z, message.orientation.w);
  controlOri.normalize();

  // transform x axis into local frame
  var controlAxis = new THREE.Vector3(1, 0, 0);
  controlAxis.applyQuaternion(controlOri);

  this.currentControlOri = new THREE.Quaternion();

  // determine mouse interaction
  switch (message.interaction_mode) {
    case ROS3D.INTERACTIVE_MARKER_MOVE_AXIS:
      this.addEventListener('mousemove', this.parent.moveAxis.bind(this.parent, this, controlAxis));
      this.addEventListener('touchmove', this.parent.moveAxis.bind(this.parent, this, controlAxis));
      break;
    case ROS3D.INTERACTIVE_MARKER_ROTATE_AXIS:
      this
          .addEventListener('mousemove', this.parent.rotateAxis.bind(this.parent, this, controlOri));
      break;
    case ROS3D.INTERACTIVE_MARKER_MOVE_PLANE:
      this
          .addEventListener('mousemove', this.parent.movePlane.bind(this.parent, this, controlAxis));
      break;
    case ROS3D.INTERACTIVE_MARKER_BUTTON:
      this.addEventListener('click', this.parent.buttonClick.bind(this.parent, this));
      break;
    default:
      break;
  }

  /**
   * Install default listeners for highlighting / dragging.
   * 
   * @param event - the event to stop
   */
  function stopPropagation(event) {
    event.stopPropagation();
  }

  // check the mode
  if (message.interaction_mode != ROS3D.INTERACTIVE_MARKER_NONE) {
    this.addEventListener('mousedown', this.parent.startDrag.bind(this.parent, this));
    this.addEventListener('mouseup', this.parent.stopDrag.bind(this.parent, this));
    this.addEventListener('contextmenu', this.parent.showMenu.bind(this.parent, this));
    this.addEventListener('mouseover', stopPropagation);
    this.addEventListener('mouseout', stopPropagation);
    this.addEventListener('click', stopPropagation);

    // touch support
    this.addEventListener('touchstart', function(event3d) {
      console.log(event3d.domEvent);
      if (event3d.domEvent.touches.length == 1) {
        event3d.type = 'mousedown';
        event3d.domEvent.button = 0;
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('touchmove', function(event3d) {
      if (event3d.domEvent.touches.length == 1) {
        console.log(event3d.domEvent);
        event3d.type = 'mousemove';
        event3d.domEvent.button = 0;
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('touchend', function(event3d) {
      if (event3d.domEvent.touches.length == 0) {
        event3d.domEvent.button = 0;
        event3d.type = 'mouseup';
        that.dispatchEvent(event3d);
        event3d.type = 'click';
        that.dispatchEvent(event3d);
      }
    });
  }

  // rotation behavior
  var rotInv = new THREE.Quaternion();
  var posInv = this.parent.position.clone().multiplyScalar(-1);
  switch (message.orientation_mode) {
    case ROS3D.INTERACTIVE_MARKER_INHERIT:
      rotInv = this.parent.quaternion.clone().inverse();
      this.updateMatrixWorld = function(force) {
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
        that.currentControlOri.copy(that.quaternion);
        that.currentControlOri.normalize();
      };
      break;
    case ROS3D.INTERACTIVE_MARKER_FIXED:
      this.updateMatrixWorld = function(force) {
        that.useQuaternion = true;
        that.quaternion = that.parent.quaternion.clone().inverse();
        that.updateMatrix();
        that.matrixWorldNeedsUpdate = true;
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
        that.currentControlOri.copy(that.quaternion);
      };
      break;
    case ROS3D.INTERACTIVE_MARKER_VIEW_FACING:
      var independentMarkerOrientation = message.independentMarkerOrientation;
      this.updateMatrixWorld = function(force) {
        that.camera.updateMatrixWorld();
        var cameraRot = new THREE.Matrix4().extractRotation(that.camera.matrixWorld);

        var ros2Gl = new THREE.Matrix4();
        var r90 = Math.PI * 0.5;
        var rv = new THREE.Vector3(-r90, 0, r90);
        ros2Gl.setRotationFromEuler(rv);

        var worldToLocal = new THREE.Matrix4();
        worldToLocal.getInverse(that.parent.matrixWorld);

        cameraRot.multiplyMatrices(cameraRot, ros2Gl);
        cameraRot.multiplyMatrices(worldToLocal, cameraRot);

        that.currentControlOri.setFromRotationMatrix(cameraRot);

        // check the orientation
        if (!independentMarkerOrientation) {
          that.useQuaternion = true;
          that.quaternion.copy(that.currentControlOri);
          that.updateMatrix();
          that.matrixWorldNeedsUpdate = true;
        }
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
      };
      break;
    default:
      console.error('Unkown orientation mode: ' + message.orientation_mode);
      break;
  }

  // create visuals (markers)
  message.markers.forEach(function(markerMsg) {
    var markerHelper = new ROS3D.Marker({
      message : markerMsg,
      path : that.path
    });

    if (markerMsg.header.frame_id !== '') {
      // if the marker lives in its own coordinate frame, convert position into IM's local frame
      markerHelper.position.add(posInv);
      markerHelper.position.applyQuaternion(rotInv);
      markerHelper.quaternion.multiplyQuaternions(rotInv, markerHelper.quaternion);
      markerHelper.updateMatrixWorld();
    }

    // add the marker
    that.add(markerHelper);
  });
};
ROS3D.InteractiveMarkerControl.prototype.__proto__ = THREE.Object3D.prototype;
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * Handle with signals for a single interactive marker.
 *
 * Emits the following events:
 *  * 'pose' - emitted when a new pose comes from the server
 *
 * @constructor
 * @param options - object with following keys:
 *  * message - the interactive marker message
 *  * feedbackTopic - the ROSLIB.Topic associated with the feedback
 *  * tfClient - a handle to the TF client to use
 */
ROS3D.InteractiveMarkerHandle = function(options) {
  var options = options || {};
  this.message = options.message;
  this.feedbackTopic = options.feedbackTopic;
  this.tfClient = options.tfClient;
  this.name = this.message.name;
  this.header = this.message.header;
  this.controls = this.message.controls;
  this.menuEntries = this.message.menu_entries;
  this.dragging = false;
  this.timeoutHandle = null;
  this.tfTransform = new ROSLIB.Transform();
  this.pose = new ROSLIB.Pose();

  // start by setting the pose
  this.setPoseFromServer(this.message.pose);
};
ROS3D.InteractiveMarkerHandle.prototype.__proto__ = EventEmitter2.prototype;

/**
 * Subscribe to the TF associated with this interactive marker.
 */
ROS3D.InteractiveMarkerHandle.prototype.subscribeTf = function() {
  // subscribe to tf updates if frame-fixed
  if (this.message.header.stamp.secs === 0.0 && this.message.header.stamp.nsecs === 0.0) {
    this.tfClient.subscribe(this.message.header.frame_id, this.tfUpdate.bind(this));
  }
};

/**
 * Emit the new pose that has come from the server.
 */
ROS3D.InteractiveMarkerHandle.prototype.emitServerPoseUpdate = function() {
  var poseTransformed = new ROSLIB.Pose(this.pose);
  poseTransformed.applyTransform(this.tfTransform);
  this.emit('pose', poseTransformed);
};

/**
 * Update the pose based on the pose given by the server.
 * 
 * @param poseMsg - the pose given by the server
 */
ROS3D.InteractiveMarkerHandle.prototype.setPoseFromServer = function(poseMsg) {
  this.pose = new ROSLIB.Pose(poseMsg);
  this.emitServerPoseUpdate();
};

/**
 * Update the pose based on the TF given by the server.
 * 
 * @param transformMsg - the TF given by the server
 */
ROS3D.InteractiveMarkerHandle.prototype.tfUpdate = function(transformMsg) {
  this.tfTransform = new ROSLIB.Transform(transformMsg);
  this.emitServerPoseUpdate();
};

/**
 * Set the pose from the client based on the given event.
 * 
 * @param event - the event to base the change off of
 */
ROS3D.InteractiveMarkerHandle.prototype.setPoseFromClient = function(event) {
  // apply the transform
  this.pose = new ROSLIB.Pose(event);
  var inv = this.tfTransform.clone();
  inv.rotation.invert();
  this.pose.applyTransform(inv);

  // send feedback to the server
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_POSE_UPDATE, undefined, 0, event.controlName);

  // keep sending pose feedback until the mouse goes up
  if (this.dragging) {
    if (this.timeoutHandle) {
      clearTimeout(this.timeoutHandle);
    }
    this.timeoutHandle = setTimeout(this.setPoseFromClient.bind(this, event), 250);
  }
};

/**
 * Send the button click feedback to the server.
 * 
 * @param event - the event associated with the button click
 */
ROS3D.InteractiveMarkerHandle.prototype.onButtonClick = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_BUTTON_CLICK, event.clickPosition, 0,
      event.controlName);
};

/**
 * Send the mousedown feedback to the server.
 * 
 * @param event - the event associated with the mousedown
 */
ROS3D.InteractiveMarkerHandle.prototype.onMouseDown = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_MOUSE_DOWN, event.clickPosition, 0, event.controlName);
  this.dragging = true;
};

/**
 * Send the mouseup feedback to the server.
 * 
 * @param event - the event associated with the mouseup
 */
ROS3D.InteractiveMarkerHandle.prototype.onMouseUp = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_MOUSE_UP, event.clickPosition, 0, event.controlName);
  this.dragging = false;
  if (this.timeoutHandle) {
    clearTimeout(this.timeoutHandle);
  }
};

/**
 * Send the menu select feedback to the server.
 * 
 * @param event - the event associated with the menu select
 */
ROS3D.InteractiveMarkerHandle.prototype.onMenuSelect = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_MENU_SELECT, undefined, event.id, event.controlName);
};

/**
 * Send feedback to the interactive marker server.
 * 
 * @param eventType - the type of event that happened
 * @param clickPosition (optional) - the position in ROS space the click happened
 * @param menuEntryID (optional) - the menu entry ID that is associated
 * @param controlName - the name of the control
 */
ROS3D.InteractiveMarkerHandle.prototype.sendFeedback = function(eventType, clickPosition,
    menuEntryID, controlName) {

  // check for the click position
  var mousePointValid = clickPosition !== undefined;
  var clickPosition = clickPosition || {
    x : 0,
    y : 0,
    z : 0
  };

  var feedback = {
    header : this.header,
    client_id : this.clientID,
    marker_name : this.name,
    control_name : controlName,
    event_type : eventType,
    pose : this.pose,
    mouse_point : clickPosition,
    mouse_point_valid : mousePointValid,
    menu_entry_id : menuEntryID
  };
  this.feedbackTopic.publish(feedback);
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A menu for an interactive marker. This will be overlayed on the canvas.
 *
 * @constructor
 * @param options - object with following keys:
 *  * menuEntries - the menu entries to add
 *  * className (optional) - a custom CSS class for the menu div
 *  * entryClassName (optional) - a custom CSS class for the menu entry
 *  * overlayClassName (optional) - a  custom CSS class for the menu overlay
 */
ROS3D.InteractiveMarkerMenu = function(options) {
  var that = this;
  var options = options || {};
  var menuEntries = options.menuEntries;
  var className = options.className || 'default-interactive-marker-menu';
  var entryClassName = options.entryClassName || 'default-interactive-marker-menu-entry';
  var overlayClassName = options.overlayClassName || 'default-interactive-marker-overlay';

  // holds the menu tree
  var allMenus = [];
  allMenus[0] = {
    children : []
  };

  THREE.EventDispatcher.call(this);

  // create the CSS for this marker if it has not been created
  if (document.getElementById('default-interactive-marker-menu-css') === null) {
    var style = document.createElement('style');
    style.id = 'default-interactive-marker-menu-css';
    style.type = 'text/css';
    style.innerHTML = '.default-interactive-marker-menu {' + 'background-color: #444444;'
        + 'border: 1px solid #888888;' + 'border: 1px solid #888888;' + 'padding: 0px 0px 0px 0px;'
        + 'color: #FFFFFF;' + 'font-family: sans-serif;' + 'font-size: 0.8em;' + 'z-index: 1002;'
        + '}' + '.default-interactive-marker-menu ul {' + 'padding: 0px 0px 5px 0px;'
        + 'margin: 0px;' + 'list-style-type: none;' + '}'
        + '.default-interactive-marker-menu ul li div {' + '-webkit-touch-callout: none;'
        + '-webkit-user-select: none;' + '-khtml-user-select: none;' + '-moz-user-select: none;'
        + '-ms-user-select: none;' + 'user-select: none;' + 'cursor: default;'
        + 'padding: 3px 10px 3px 10px;' + '}' + '.default-interactive-marker-menu-entry:hover {'
        + '  background-color: #666666;' + '  cursor: pointer;' + '}'
        + '.default-interactive-marker-menu ul ul {' + '  font-style: italic;'
        + '  padding-left: 10px;' + '}' + '.default-interactive-marker-overlay {'
        + '  position: absolute;' + '  top: 0%;' + '  left: 0%;' + '  width: 100%;'
        + '  height: 100%;' + '  background-color: black;' + '  z-index: 1001;'
        + '  -moz-opacity: 0.0;' + '  opacity: .0;' + '  filter: alpha(opacity = 0);' + '}';
    document.getElementsByTagName('head')[0].appendChild(style);
  }

  // place the menu in a div
  this.menuDomElem = document.createElement('div');
  this.menuDomElem.style.position = 'absolute';
  this.menuDomElem.className = className;
  this.menuDomElem.addEventListener('contextmenu', function(event) {
    event.preventDefault();
  });

  // create the overlay DOM
  this.overlayDomElem = document.createElement('div');
  this.overlayDomElem.className = overlayClassName;

  this.hideListener = this.hide.bind(this);
  this.overlayDomElem.addEventListener('contextmenu', this.hideListener);
  this.overlayDomElem.addEventListener('click', this.hideListener);

  // parse all entries
  for ( var i = 0; i < menuEntries.length; i++) {
    var entry = menuEntries[i];
    var id = entry.id;
    allMenus[id] = {
      title : entry.title,
      id : id,
      children : []
    };
  }

  // link children to parents
  for ( var i = 0; i < menuEntries.length; i++) {
    var entry = menuEntries[i];
    var id = entry.id;
    var menu = allMenus[id];
    var parent = allMenus[entry.parent_id];
    parent.children.push(menu);
  }

  function emitMenuSelect(menuEntry, domEvent) {
    this.dispatchEvent({
      type : 'menu-select',
      domEvent : domEvent,
      id : menuEntry.id,
      controlName : this.controlName
    });
    this.hide(domEvent);
  }

  /**
   * Create the HTML UL element for the menu and link it to the parent.
   * 
   * @param parentDomElem - the parent DOM element
   * @param parentMenu - the parent menu
   */
  function makeUl(parentDomElem, parentMenu) {

    var ulElem = document.createElement('ul');
    parentDomElem.appendChild(ulElem);

    var children = parentMenu.children;

    for ( var i = 0; i < children.length; i++) {
      var liElem = document.createElement('li');
      var divElem = document.createElement('div');
      divElem.appendChild(document.createTextNode(children[i].title));
      ulElem.appendChild(liElem);
      liElem.appendChild(divElem);

      if (children[i].children.length > 0) {
        makeUl(liElem, children[i]);
        divElem.addEventListener('click', that.hide.bind(that));
      } else {
        divElem.addEventListener('click', emitMenuSelect.bind(that, children[i]));
        divElem.className = 'default-interactive-marker-menu-entry';
      }
    }

  }

  // construct DOM element
  makeUl(this.menuDomElem, allMenus[0]);
};

/**
 * Shoe the menu DOM element.
 * 
 * @param control - the control for the menu
 * @param event - the event that caused this
 */
ROS3D.InteractiveMarkerMenu.prototype.show = function(control, event) {
  if (event && event.preventDefault) {
    event.preventDefault();
  }

  this.controlName = control.name;

  // position it on the click
  this.menuDomElem.style.left = event.domEvent.clientX + 'px';
  this.menuDomElem.style.top = event.domEvent.clientY + 'px';
  document.body.appendChild(this.overlayDomElem);
  document.body.appendChild(this.menuDomElem);
};

/**
 * Hide the menu DOM element.
 * 
 * @param event (optional) - the event that caused this
 */
ROS3D.InteractiveMarkerMenu.prototype.hide = function(event) {
  if (event && event.preventDefault) {
    event.preventDefault();
  }

  document.body.removeChild(this.overlayDomElem);
  document.body.removeChild(this.menuDomElem);
};
/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An OccupancyGrid can convert a ROS occupancy grid message into a THREE object.
 *
 * @constructor
 * @param options - object with following keys:
 *   * message - the occupancy grid message
 */
ROS3D.OccupancyGrid = function(options) {
  var options = options || {};
  var message = options.message;

  // create the geometry
  var width = message.info.width;
  var height = message.info.height;
  var geom = new THREE.PlaneGeometry(width, height);

  // create the color material
  var dataColor = new Uint8Array(width * height * 3);
  for ( var row = 0; row < height; row++) {
    for ( var col = 0; col < width; col++) {
      // determine the index into the map data
      var mapI = col + ((height - row - 1) * width);
      // determine the value
      var data = message.data[mapI];
      if (data === 100) {
        var val = 0;
      } else if (data === 0) {
        var val = 255;
      } else {
        var val = 127;
      }

      // determine the index into the image data array
      var i = (col + (row * width)) * 3;
      // r
      dataColor[i] = val;
      // g
      dataColor[++i] = val;
      // b
      dataColor[++i] = val;
    }
  }
  var texture = new THREE.DataTexture(dataColor, width, height, THREE.RGBFormat);
  texture.needsUpdate = true;
  var material = new THREE.MeshBasicMaterial({
    map : texture
  });
  material.side = THREE.DoubleSide;

  // create the mesh
  THREE.Mesh.call(this, geom, material);
  this.scale.x = message.info.resolution;
  this.scale.y = message.info.resolution;
};
ROS3D.OccupancyGrid.prototype.__proto__ = THREE.Mesh.prototype;
/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An occupancy grid client that listens to a given map topic.
 * 
 * Emits the following events:
 *  * 'change' - there was an update or change in the marker
 *  
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * rootObject (optional) - the root object to add this marker to
 */
ROS3D.OccupancyGridClient = function(options) {
  var that = this;
  var options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/map';
  this.continuous = options.continuous;
  this.rootObject = options.rootObject || new THREE.Object3D();

  // current grid that is displayed
  this.currentGrid = null;

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'nav_msgs/OccupancyGrid',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {
    // check for an old map
    if (that.currentGrid) {
      that.rootObject.remove(that.currentGrid);
    }

    that.currentGrid = new ROS3D.OccupancyGrid({
      message : message
    });
    that.rootObject.add(that.currentGrid);

    that.emit('change');
    
    // check if we should unsubscribe
    if(!that.continuous) {
      rosTopic.unsubscribe();
    }
  });
};
ROS3D.OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;
/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A Marker can convert a ROS marker message into a THREE object.
 *
 * @constructor
 * @param options - object with following keys:
 *   * path - the base path or URL for any mesh files that will be loaded for this marker
 *   * message - the marker message
 */
ROS3D.Marker = function(options) {
  var options = options || {};
  var path = options.path || '/';
  var message = options.message;

  // check for a trailing '/'
  if (path.substr(path.length - 1) !== '/') {
    path += '/';
  }

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // set the pose and get the color
  this.setPose(message.pose);
  var colorMaterial = ROS3D.makeColorMaterial(message.color.r, message.color.g, message.color.b,
      message.color.a);

  // create the object based on the type
  switch (message.type) {
    case ROS3D.MARKER_ARROW:
      // get the sizes for the arrow
      var len = message.scale.x;
      var headLength = len * 0.23;
      var headDiameter = message.scale.y;
      var shaftDiameter = headDiameter * 0.5;

      // determine the points
      if (message.points.length === 2) {
        var p1 = new THREE.Vector3(message.points[0].x, message.points[0].y, message.points[0].z);
        var p2 = new THREE.Vector3(message.points[1].x, message.points[1].y, message.points[1].z);
        var direction = p1.clone().negate().add(p2);
        // direction = p2 - p1;
        len = direction.length();
        headDiameter = message.scale.y;
        shaftDiameter = message.scale.x;

        if (message.scale.z !== 0.0) {
          headLength = message.scale.z;
        }
      }

      // add the marker
      this.add(new ROS3D.Arrow({
        direction : direction,
        origin : p1,
        length : len,
        headLength : headLength,
        shaftDiameter : shaftDiameter,
        headDiameter : headDiameter,
        material : colorMaterial
      }));
      break;
    case ROS3D.MARKER_CUBE:
      // set the cube dimensions
      var geom = new THREE.CubeGeometry(message.scale.x, message.scale.y, message.scale.z);
      this.add(new THREE.Mesh(geom, colorMaterial));
      break;
    case ROS3D.MARKER_SPHERE:
      // set the sphere dimensions
      var geom = new THREE.SphereGeometry(0.5);
      var mesh = new THREE.Mesh(geom, colorMaterial);
      mesh.scale.x = message.scale.x;
      mesh.scale.y = message.scale.y;
      mesh.scale.z = message.scale.z;
      this.add(mesh);
      break;
    case ROS3D.MARKER_CYLINDER:
      // set the cylinder dimensions
      var geom = new THREE.CylinderGeometry(0.5, 0.5, 1, 16, 1, false);
      var mesh = new THREE.Mesh(geom, colorMaterial);
      mesh.useQuaternion = true;
      mesh.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * 0.5);
      mesh.scale = new THREE.Vector3(message.scale.x, message.scale.y, message.scale.z);
      this.add(mesh);
      break;
    case ROS3D.MARKER_CUBE_LIST:
    case ROS3D.MARKER_SPHERE_LIST:
    case ROS3D.MARKER_POINTS:
      // for now, use a particle system for the lists
      var geometry = new THREE.Geometry();
      var material = new THREE.ParticleBasicMaterial({
        size : message.scale.x
      });

      // add the points
      for ( var i = 0; i < message.points.length; i++) {
        var vertex = new THREE.Vector3();
        vertex.x = message.points[i].x;
        vertex.y = message.points[i].y;
        vertex.z = message.points[i].z;
        geometry.vertices.push(vertex);
      }

      // determine the colors for each
      if (message.colors.length === message.points.length) {
        material.vertexColors = true;
        for ( var i = 0; i < message.points.length; i++) {
          var color = new THREE.Color();
          color.setRGB(message.colors[i].r, message.colors[i].g, message.colors[i].b);
          geometry.colors.push(color);
        }
      } else {
        material.color.setRGB(message.color.r, message.color.g, message.color.b);
      }

      // add the particle system
      this.add(new THREE.ParticleSystem(geometry, material));
      break;
    case ROS3D.MARKER_TEXT_VIEW_FACING:
      // setup the text
      var textGeo = new THREE.TextGeometry(message.text, {
        size : message.scale.x * 0.5,
        height : 0.1 * message.scale.x,
        curveSegments : 4,
        font : 'helvetiker',
        bevelEnabled : false,
        bevelThickness : 2,
        bevelSize : 2,
        material : 0,
        extrudeMaterial : 0
      });
      textGeo.computeVertexNormals();
      textGeo.computeBoundingBox();

      // position the text and add it
      var mesh = new THREE.Mesh(textGeo, colorMaterial);
      var centerOffset = -0.5 * (textGeo.boundingBox.max.x - textGeo.boundingBox.min.x);
      mesh.position.y = -centerOffset;
      mesh.rotation.x = Math.PI * 0.5;
      mesh.rotation.y = Math.PI * 1.5;
      this.add(mesh);
      break;
    case ROS3D.MARKER_MESH_RESOURCE:
      // load and add the mesh
      this.add(new ROS3D.MeshResource({
        path : path,
        resource : message.mesh_resource.substr(10)
      }));
      break;
    case ROS3D.MARKER_TRIANGLE_LIST:
      // create the list of triangles
      var tri = new ROS3D.TriangleList({
        material : colorMaterial,
        vertices : message.points,
        colors : message.colors
      });
      tri.scale = new THREE.Vector3(message.scale.x, message.scale.y, message.scale.z);
      this.add(tri);
      break;
    default:
      console.error('Currently unsupported marker type: ' + message.type);
      break;
  }
};
ROS3D.Marker.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Set the pose of this marker to the given values.
 * 
 * @param pose - the pose to set for this marker
 */
ROS3D.Marker.prototype.setPose = function(pose) {
  // set position information
  this.position.x = pose.position.x;
  this.position.y = pose.position.y;
  this.position.z = pose.position.z;

  // set the rotation
  this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);
  this.quaternion.normalize();

  // update the world
  this.updateMatrixWorld();
};
/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A marker client that listens to a given marker topic.
 * 
 * Emits the following events:
 *  * 'change' - there was an update or change in the marker
 *  
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic - the marker topic to listen to
 *   * tfClient - the TF client handle to use
 *   * rootObject (optional) - the root object to add this marker to
 */
ROS3D.MarkerClient = function(options) {
  var that = this;
  var options = options || {};
  var ros = options.ros;
  var topic = options.topic;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();

  // current marker that is displayed
  this.currentMarker = null;

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'visualization_msgs/Marker',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {
    var newMarker = new ROS3D.Marker({
      message : message
    });
    // check for an old marker
    if (that.currentMarker) {
      that.rootObject.remove(that.currentMarker);
    }

    that.currentMarker = new ROS3D.SceneNode({
      frameID : message.header.frame_id,
      tfClient : that.tfClient,
      object : newMarker
    });
    that.rootObject.add(that.currentMarker);
    
    that.emit('change');
  });
};
ROS3D.MarkerClient.prototype.__proto__ = EventEmitter2.prototype;
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A Arrow is a THREE object that can be used to display an arrow model.
 *
 * @constructor
 * @param options - object with following keys:
 *   * origin (optional) - the origin of the arrow
 *   * direction (optional) - the direction vector of the arrow
 *   * length (optional) - the length of the arrow
 *   * headLength (optional) - the head length of the arrow
 *   * shaftDiameter (optional) - the shaft diameter of the arrow
 *   * headDiameter (optional) - the head diameter of the arrow
 *   * material (optional) - the material to use for this arrow
 */
ROS3D.Arrow = function(options) {
  var options = options || {};
  var origin = options.origin || new THREE.Vector3(0, 0, 0);
  var direction = options.direction || new THREE.Vector3(1, 0, 0);
  var length = options.length || 1;
  var headLength = options.headLength || 0.2;
  var shaftDiameter = options.shaftDiameter || 0.05;
  var headDiameter = options.headDiameter || 0.1;
  var material = options.material || new THREE.MeshBasicMaterial();

  var shaftLength = length - headLength;

  // create and merge geometry
  var geometry = new THREE.CylinderGeometry(shaftDiameter * 0.5, shaftDiameter * 0.5, shaftLength,
      12, 1);
  var m = new THREE.Matrix4();
  m.setPosition(new THREE.Vector3(0, shaftLength * 0.5, 0));
  geometry.applyMatrix(m);

  // create the head
  var coneGeometry = new THREE.CylinderGeometry(0, headDiameter * 0.5, headLength, 12, 1);
  m.setPosition(new THREE.Vector3(0, shaftLength + (headLength * 0.5), 0));
  coneGeometry.applyMatrix(m);

  // put the arrow together
  THREE.GeometryUtils.merge(geometry, coneGeometry);

  THREE.Mesh.call(this, geometry, material);

  this.position = origin;
  this.setDirection(direction);
};
ROS3D.Arrow.prototype.__proto__ = THREE.Mesh.prototype;

/**
 * Set the direction of this arrow to that of the given vector.
 * 
 * @param direction - the direction to set this arrow
 */
ROS3D.Arrow.prototype.setDirection = function(direction) {
  var axis = new THREE.Vector3(0, 1, 0).cross(direction);
  var radians = Math.acos(new THREE.Vector3(0, 1, 0).dot(direction.clone().normalize()));
  this.matrix = new THREE.Matrix4().makeRotationAxis(axis.normalize(), radians);
  this.rotation.setEulerFromRotationMatrix(this.matrix, this.eulerOrder);
};

/**
 * Set this arrow to be the given length.
 * 
 * @param length - the new length of the arrow
 */
ROS3D.Arrow.prototype.setLength = function(length) {
  this.scale.set(length, length, length);
};

/**
 * Set the color of this arrow to the given hex value.
 * 
 * @param hex - the hex value of the color to use
 */
ROS3D.Arrow.prototype.setColor = function(hex) {
  this.line.material.color.setHex(hex);
  this.cone.material.color.setHex(hex);
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * An Axes object can be used to display the axis of a particular coordinate frame.
 * 
 * @constructor
 * @param options - object with following keys:
 *   * shaftRadius (optional) - the radius of the shaft to render
 *   * headRadius (optional) - the radius of the head to render 
 *   * headLength (optional) - the length of the head to render
 */
ROS3D.Axes = function(options) {
  var that = this;
  var options = options || {};
  var shaftRadius = options.shaftRadius || 0.008;
  var headRadius = options.headRadius || 0.023;
  var headLength = options.headLength || 0.1;

  THREE.Object3D.call(this);

  // create the cylinders for the objects
  this.lineGeom = new THREE.CylinderGeometry(shaftRadius, shaftRadius, 1.0 - headLength);
  this.headGeom = new THREE.CylinderGeometry(0, headRadius, headLength);

  /**
   * Adds an axis marker to this axes object.
   * 
   * @param axis - the 3D vector representing the axis to add
   */
  function addAxis(axis) {
    // set the color of the axis
    var color = new THREE.Color();
    color.setRGB(axis.x, axis.y, axis.z);
    var material = new THREE.MeshBasicMaterial({
      color : color.getHex()
    });

    // setup the rotation information
    var rotAxis = new THREE.Vector3;
    rotAxis.crossVectors(axis, new THREE.Vector3(0, -1, 0));
    var rot = new THREE.Quaternion;
    rot.setFromAxisAngle(rotAxis, 0.5 * Math.PI);

    // create the arrow
    var arrow = new THREE.Mesh(that.headGeom, material);
    arrow.position = axis.clone();
    arrow.position.multiplyScalar(0.95);
    arrow.useQuaternion = true;
    arrow.quaternion = rot;
    arrow.updateMatrix();
    that.add(arrow);

    // create the line
    var line = new THREE.Mesh(that.lineGeom, material);
    line.position = axis.clone();
    line.position.multiplyScalar(0.45);
    line.useQuaternion = true;
    line.quaternion = rot;
    line.updateMatrix();
    that.add(line);
  };

  // add the three markers to the axes
  addAxis(new THREE.Vector3(1, 0, 0));
  addAxis(new THREE.Vector3(0, 1, 0));
  addAxis(new THREE.Vector3(0, 0, 1));
};
ROS3D.Axes.prototype.__proto__ = THREE.Object3D.prototype;
/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * Create a grid object.
 * 
 * @constructor
 * @param options - object with following keys:
 *  * size (optional) - the size of the grid
 *  * color (optional) - the line color of the grid, like '#cccccc'
 *  * lineWidth (optional) - the width of the lines in the grid
 */
ROS3D.Grid = function(options) {
  var options = options || {};
  var size = options.size || 50;
  var color = options.color || '#cccccc';
  var lineWidth = options.lineWidth || 1;

  // create the mesh
  THREE.Mesh.call(this, new THREE.PlaneGeometry(size, size, size, size),
      new THREE.MeshBasicMaterial({
        color : color,
        wireframe : true,
        wireframeLinewidth : lineWidth,
        transparent : true
      }));
};
ROS3D.Grid.prototype.__proto__ = THREE.Mesh.prototype;
/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A MeshResource is an THREE object that will load from a external mesh file. Currently loads
 * Collada files.
 * 
 * @constructor
 * @param options - object with following keys:
 *  * path (optional) - the base path to the associated models that will be loaded
 *  * resource - the resource file name to load
 */
ROS3D.MeshResource = function(options) {
  var that = this;
  var options = options || {};
  var path = options.path || '/';
  var resource = options.resource;

  THREE.Object3D.call(this);

  // check for a trailing '/'
  if (path.substr(path.length - 1) !== '/') {
    this.path += '/';
  }

  var uri = path + resource;
  var fileType = uri.substr(-4).toLowerCase();

  // check the type
  if (uri.substr(-4).toLowerCase() === '.dae') {
    var loader = new THREE.ColladaLoader();
    loader.load(uri, function colladaReady(collada) {
      that.add(collada.scene);
    });
  }
};
ROS3D.MeshResource.prototype.__proto__ = THREE.Object3D.prototype;
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A TriangleList is a THREE object that can be used to display a list of triangles as a geometry.
 *
 * @constructor
 * @param options - object with following keys:
 *   * material (optional) - the material to use for the object
 *   * vertices - the array of vertices to use
 *   * colors - the associated array of colors to use
 */
ROS3D.TriangleList = function(options) {
  var options = options || {};
  var material = options.material || new THREE.MeshBasicMaterial();
  var vertices = options.vertices;
  var colors = options.colors;

  THREE.Object3D.call(this);

  // set the material to be double sided
  material.side = THREE.DoubleSide;

  // construct the geometry
  var geometry = new THREE.Geometry();
  for (i = 0; i < vertices.length; i++) {
    geometry.vertices.push(new THREE.Vector3(vertices[i].x, vertices[i].y, vertices[i].z));
  }

  // set the colors
  if (colors.length === vertices.length) {
    // use per-vertex color
    for (i = 0; i < vertices.length; i += 3) {
      var face = new THREE.Face3(i, i + 1, i + 2);
      for (j = i * 3; j < i * 3 + 3; i++) {
        var color = new THREE.Color();
        color.setRGB(colors[i].r, colors[i].g, colors[i].b);
        face.vertexColors.push(color);
      }
      geometry.faces.push(face);
    }
    material.vertexColors = THREE.VertexColors;
  } else if (colors.length === vertices.length / 3) {
    // use per-triangle color
    for (i = 0; i < vertices.length; i += 3) {
      var face = new THREE.Face3(i, i + 1, i + 2);
      face.color.setRGB(colors[i / 3].r, colors[i / 3].g, colors[i / 3].b);
      geometry.faces.push(face);
    }
    material.vertexColors = THREE.FaceColors;
  } else {
    // use marker color
    for (i = 0; i < vertices.length; i += 3) {
      var face = new THREE.Face3(i, i + 1, i + 2);
      geometry.faces.push(face);
    }
  }

  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  geometry.computeCentroids();
  geometry.computeFaceNormals();

  this.add(new THREE.Mesh(geometry, material));
};
ROS3D.TriangleList.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Set the color of this object to the given hex value.
 * 
 * @param hex - the hex value of the color to set
 */
ROS3D.TriangleList.prototype.setColor = function(hex) {
  this.mesh.material.color.setHex(hex);
};
/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A URDF can be used to load a ROSLIB.UrdfModel and its associated models into a 3D object.
 *  
 * @constructor
 * @param options - object with following keys:
 *   * urdfModel - the ROSLIB.UrdfModel to load
 *   * tfClient - the TF client handle to use
 *   * path (optional) - the base path to the associated Collada models that will be loaded
 */
ROS3D.Urdf = function(options) {
  var options = options || {};
  var urdfModel = options.urdfModel;
  var path = options.path || '/';
  var tfClient = options.tfClient;

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // load all models
  var links = urdfModel.links;
  for ( var l in links) {
    var link = links[l];
    if (link.visual && link.visual.geometry) {
      if (link.visual.geometry.type === ROSLIB.URDF_MESH) {
        var frameID = '/' + link.name;
        var uri = link.visual.geometry.filename;
        var fileType = uri.substr(-4).toLowerCase();

        // ignore mesh files which are not in Collada format
        if (fileType === '.dae') {
          // create a scene node with the model
          var sceneNode = new ROS3D.SceneNode({
            frameID : frameID,
            pose : link.visual.origin,
            tfClient : tfClient,
            object : new ROS3D.MeshResource({
              path : path,
              resource : uri.substring(10)
            })
          });
          this.add(sceneNode);
        }
      }
    }
  }
};
ROS3D.Urdf.prototype.__proto__ = THREE.Object3D.prototype;
/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A URDF client can be used to load a URDF and its associated models into a 3D object from the ROS
 * parameter server.
 *
 * Emits the following events:
 * * 'change' - emited after the URDF and its meshes have been loaded into the root object
 * 
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * param (optional) - the paramter to load the URDF from, like 'robot_description'
 *   * tfClient - the TF client handle to use
 *   * path (optional) - the base path to the associated Collada models that will be loaded
 *   * rootObject (optional) - the root object to add this marker to
 */
ROS3D.UrdfClient = function(options) {
  var that = this;
  var options = options || {};
  var ros = options.ros;
  var param = options.param || 'robot_description';
  this.path = options.path || '/';
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();

  // get the URDF value from ROS
  var getParam = new ROSLIB.Param({
    ros : ros,
    name : param
  });
  getParam.get(function(string) {
    // hand off the XML string to the URDF model
    var urdfModel = new ROSLIB.UrdfModel({
      string : string
    });

    // load all models
    that.rootObject.add(new ROS3D.Urdf({
      urdfModel : urdfModel,
      path : that.path,
      tfClient : that.tfClient
    }));
  });
};
/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A SceneNode can be used to keep track of a 3D object with respect to a ROS frame within a scene.
 *
 * @constructor
 * @param options - object with following keys:
 *  * tfClient - a handle to the TF client
 *  * frameID - the frame ID this object belongs to
 *  * pose (optional) - the pose associated with this object
 *  * object - the THREE 3D object to be rendered
 */
ROS3D.SceneNode = function(options) {
  var options = options || {};
  var that = this;
  var tfClient = options.tfClient;
  var frameID = options.frameID;
  var object = options.object;
  this.pose = options.pose || new ROSLIB.Pose();

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // add the model
  this.add(object);

  // listen for TF updates
  tfClient.subscribe(frameID,
      function(msg) {
        // apply the transform
        var tf = new ROSLIB.Transform(msg);
        var poseTransformed = new ROSLIB.Pose(that.pose);
        poseTransformed.applyTransform(tf);

        // update the world
        that.position.x = poseTransformed.position.x;
        that.position.y = poseTransformed.position.y;
        that.position.z = poseTransformed.position.z;
        that.quaternion = new THREE.Quaternion(poseTransformed.orientation.x,
            poseTransformed.orientation.y, poseTransformed.orientation.z,
            poseTransformed.orientation.w);
        that.updateMatrixWorld(true);
      });
};
ROS3D.SceneNode.prototype.__proto__ = THREE.Object3D.prototype;/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 */

/**
 * A Viewer can be used to render an interactive 3D scene to a HTML5 canvas.
 *
 * @constructor
 * @param options - object with following keys:
 *  * divID - the ID of the div to place the viewer in
 *  * width - the initial width, in pixels, of the canvas
 *  * height - the initial height, in pixels, of the canvas
 *  * background - the color to render the background, like '#efefef'
 *  * antialias - if antialiasing should be used
 */
ROS3D.Viewer = function(options) {
  var that = this;
  var options = options || {};
  var divID = options.divID;
  var width = options.width;
  var height = options.height;
  var background = options.background || '#111111';
  var antialias = options.antialias;

  // create the canvas to render to
  this.renderer = new THREE.WebGLRenderer({
    antialias : this.antialias
  });
  this.renderer.setClearColorHex(background.replace('#', '0x'), 1.0);
  this.renderer.sortObjects = false;
  this.renderer.setSize(width, height);
  this.renderer.shadowMapEnabled = false;
  this.renderer.autoClear = false;

  // create the global scene
  this.scene = new THREE.Scene();

  // create the global camera
  this.camera = new THREE.PerspectiveCamera(40, width / height, 0.01, 1000);
  this.camera.position.x = 3;
  this.camera.position.y = 3;
  this.camera.position.z = 3;
  // add controls to the camera
  this.cameraControls = new ROS3D.OrbitControls({
    scene : this.scene,
    camera : this.camera
  });
  this.cameraControls.userZoomSpeed = 0.5;

  // lights
  this.scene.add(new THREE.AmbientLight(0x555555));
  this.directionalLight = new THREE.DirectionalLight(0xffffff);
  this.scene.add(this.directionalLight);

  // propagates mouse events to three.js objects
  this.selectableObjects = new THREE.Object3D;
  this.scene.add(this.selectableObjects);
  var mouseHandler = new ROS3D.MouseHandler({
    renderer : this.renderer,
    camera : this.camera,
    rootObject : this.selectableObjects,
    fallbackTarget : this.cameraControls
  });

  // highlights the receiver of mouse events
  this.highlighter = new ROS3D.Highlighter({
    mouseHandler : mouseHandler
  });

  /**
   * Renders the associated scene to the that.
   */
  function draw() {
    // update the controls
    that.cameraControls.update();

    // put light to the top-left of the camera
    that.directionalLight.position = that.camera.localToWorld(new THREE.Vector3(-1, 1, 0));
    that.directionalLight.position.normalize();

    // set the scene
    that.renderer.clear(true, true, true);
    that.renderer.render(that.scene, that.camera);

    // render any mouseovers
    that.highlighter.renderHighlight(that.renderer, that.scene, that.camera);

    // draw the frame
    requestAnimationFrame(draw);
  };

  // add the renderer to the page
  document.getElementById(divID).appendChild(this.renderer.domElement);

  // begin the animation
  draw();
};

/**
 * Add the given THREE Object3D to the global scene in the viewer.
 * 
 * @param object - the THREE Object3D to add
 * @param selectable (optional) - if the object should be added to the selectable list
 */
ROS3D.Viewer.prototype.addObject = function(object, selectable) {
  if (selectable) {
    this.selectableObjects.add(object);
  } else {
    this.scene.add(object);
  }
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A mouseover highlighter for 3D objects in the scene.
 *
 * @constructor
 * @param options - object with following keys:
 *   * mouseHandler - the handler for the mouseover and mouseout events
 */
ROS3D.Highlighter = function(options) {
  var options = options || {};
  var mouseHandler = options.mouseHandler;
  this.hoverObjs = [];

  // bind the mouse events
  mouseHandler.addEventListener('mouseover', this.onMouseOver.bind(this));
  mouseHandler.addEventListener('mouseout', this.onMouseOut.bind(this));
};

/**
 * Add the current target of the mouseover to the hover list.
 * @param event - the event that contains the target of the mouseover
 */
ROS3D.Highlighter.prototype.onMouseOver = function(event) {
  this.hoverObjs.push(event.currentTarget);
};

/**
 * Remove the current target of the mouseover from the hover list.
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

  // define highlight material
  scene.overrideMaterial = new THREE.MeshBasicMaterial({
    fog : false,
    opacity : 0.5,
    depthTest : true,
    depthWrite : false,
    polygonOffset : true,
    polygonOffsetUnits : -1,
    side : THREE.DoubleSide
  });;

  // swap render lists, render, undo
  var oldWebglObjects = scene.__webglObjects;
  scene.__webglObjects = renderList;

  renderer.render(scene, camera);

  scene.__webglObjects = oldWebglObjects;
  scene.overrideMaterial = null;
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A handler for mouse events within a 3D viewer.
 *
 * @constructor
 * @param options - object with following keys:
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
  this.projector = new THREE.Projector();

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
  var left = domEvent.clientX - rect.left - target.clientLeft + target.scrollLeft;
  var top = domEvent.clientY - rect.top - target.clientTop + target.scrollTop;
  var deviceX = left / target.clientWidth * 2 - 1;
  var deviceY = -top / target.clientHeight * 2 + 1;
  var vector = new THREE.Vector3(deviceX, deviceY, 0.5);
  this.projector.unprojectVector(vector, this.camera);
  // use the THREE raycaster
  var mouseRaycaster = new THREE.Raycaster(this.camera.position.clone(), vector.sub(
      this.camera.position).normalize());
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
  if (domEvent.type == 'mouseout') {
    if (this.dragging) {
      this.notify(this.lastTarget, 'mouseup', event3D);
      this.dragging = false;
    }
    this.notify(this.lastTarget, 'mouseout', event3D);
    this.lastTarget = null;
    return;
  }

  // while the user is holding the mouse down, stay on the same target
  if (this.dragging) {
    this.notify(this.lastTarget, domEvent.type, event3D);
    // for check for right or left mouse button
    if ((domEvent.type === 'mouseup' && domEvent.button === 2) || domEvent.type === 'click') {
      this.dragging = false;
    }
    return;
  }

  // in the normal case, we need to check what is under the mouse
  var target = this.lastTarget;
  var intersections = [];
  intersections = mouseRaycaster.intersectObject(this.rootObject, true);
  if (intersections.length > 0) {
    target = intersections[0].object;
    event3D.intersection = this.lastIntersection = intersections[0];
  } else {
    target = this.fallbackTarget;
  }

  // if the mouse moves from one object to another (or from/to the 'null' object), notify both
  if (target !== this.lastTarget) {
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

  // pass through event
  this.notify(target, domEvent.type, event3D);
  if (domEvent.type === 'mousedown') {
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

/**
 * Destroy this mouse handler and its associated listeners.
 */
ROS3D.MouseHandler.prototype.destroy = function() {
  this.listeners.forEach(function(listener) {
    this.renderer.domElement.removeEventListener(eventName, listener, false);
  }, this);
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Xueqiao Xu - xueqiaoxu@gmail.com
 * @author Mr.doob - http://mrdoob.com
 * @author AlteredQualia - http://alteredqualia.com
 */

/**
 * Behaves like THREE.OrbitControls, but uses right-handed coordinates and z as up vector.
 *
 * @constructor
 * @param scene - the global scene to use
 * @param camera - the camera to use
 * @param userZoomSpeed (optional) - the speed for zooming
 * @param userRotateSpeed (optional) - the speed for rotating
 * @param autoRotate (optional) - if the orbit should auto rotate
 * @param autoRotate (optional) - the speed for auto rotating 
 */
ROS3D.OrbitControls = function(options) {
  THREE.EventDispatcher.call(this);
  var that = this;
  var options = options || {};
  var scene = options.scene;
  this.camera = options.camera;
  this.center = new THREE.Vector3();
  this.userZoom = true;
  this.userZoomSpeed = options.userZoomSpeed || 1.0;
  this.userRotate = true;
  this.userRotateSpeed = options.userRotateSpeed || 1.0;
  this.autoRotate = options.autoRotate;
  this.autoRotateSpeed = options.autoRotateSpeed || 2.0;

  // In ROS, z is pointing upwards
  this.camera.up = new THREE.Vector3(0, 0, 1);

  // internals
  var pixlesPerRound = 1800;
  var rotateStart = new THREE.Vector2();
  var rotateEnd = new THREE.Vector2();
  var rotateDelta = new THREE.Vector2();
  var zoomStart = new THREE.Vector2();
  var zoomEnd = new THREE.Vector2();
  var zoomDelta = new THREE.Vector2();
  var moveStartCenter = new THREE.Vector3();
  var moveStartNormal = new THREE.Vector3();
  var moveStartPosition = new THREE.Vector3();
  var moveStartIntersection = new THREE.Vector3();
  this.phiDelta = 0;
  this.thetaDelta = 0;
  this.scale = 1;
  this.lastPosition = new THREE.Vector3();
  // internal states
  var STATE = {
    NONE : -1,
    ROTATE : 0,
    ZOOM : 1,
    MOVE : 2
  };
  var state = STATE.NONE;

  // add the axes for the main coordinate frame
  this.axes = new ROS3D.Axes({
    shaftRadius : 0.025,
    headRadius : 0.07,
    headLength : 0.2
  });
  // initially not visible
  scene.add(this.axes);
  this.axes.traverse(function(obj) {
    obj.visible = false;
  });

  /**
   * Handle the mousedown 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onMouseDown(event3D) {
    var event = event3D.domEvent;
    event.preventDefault();

    switch (event.button) {
      case 0:
        state = STATE.ROTATE;
        rotateStart.set(event.clientX, event.clientY);
        break;
      case 1:
        state = STATE.MOVE;

        moveStartNormal = new THREE.Vector3(0, 0, 1);
        var rMat = new THREE.Matrix4().extractRotation(this.camera.matrix);
        // rMat.multiplyVector3( moveStartNormal );
        moveStartNormal.applyMatrix4(rMat);

        moveStartCenter = that.center.clone();
        moveStartPosition = that.camera.position.clone();
        moveStartIntersection = ROS3D.intersectPlane(event3D.mouseRay, moveStartCenter,
            moveStartNormal);
        break;
      case 2:
        state = STATE.ZOOM;
        zoomStart.set(event.clientX, event.clientY);
        break;
    }

    this.showAxes();
  };

  /**
   * Handle the movemove 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onMouseMove(event3D) {
    var event = event3D.domEvent;
    if (state === STATE.ROTATE) {

      rotateEnd.set(event.clientX, event.clientY);
      rotateDelta.subVectors(rotateEnd, rotateStart);

      that.rotateLeft(2 * Math.PI * rotateDelta.x / pixlesPerRound * that.userRotateSpeed);
      that.rotateUp(2 * Math.PI * rotateDelta.y / pixlesPerRound * that.userRotateSpeed);

      rotateStart.copy(rotateEnd);
      this.showAxes();
    } else if (state === STATE.ZOOM) {
      zoomEnd.set(event.clientX, event.clientY);
      zoomDelta.subVectors(zoomEnd, zoomStart);

      if (zoomDelta.y > 0) {
        that.zoomIn();
      } else {
        that.zoomOut();
      }

      zoomStart.copy(zoomEnd);
      this.showAxes();

    } else if (state === STATE.MOVE) {
      var intersection = ROS3D.intersectPlane(event3D.mouseRay, that.center, moveStartNormal);

      if (!intersection) {
        return;
      }

      var delta = new THREE.Vector3().subVectors(moveStartIntersection.clone(), intersection
          .clone());

      that.center.addVectors(moveStartCenter.clone(), delta.clone());
      that.camera.position.addVectors(moveStartPosition.clone(), delta.clone());
      that.update();
      that.camera.updateMatrixWorld();
      this.showAxes();
    }
  };

  /**
   * Handle the mouseup 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onMouseUp(event3D) {
    if (!that.userRotate) {
      return;
    }

    state = STATE.NONE;
  };

  /**
   * Handle the mousewheel 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onMouseWheel(event3D) {
    if (!that.userZoom) {
      return;
    }

    var event = event3D.domEvent;
    // wheelDelta --> Chrome, detail --> Firefox
    if (typeof (event.wheelDelta) !== 'undefined') {
      var delta = event.wheelDelta;
    } else {
      var delta = -event.detail;
    }
    if (delta > 0) {
      that.zoomOut();
    } else {
      that.zoomIn();
    }

    this.showAxes();
  };

  /**
   * Handle the touchdown 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onTouchDown(event) {
    onMouseDown(event);
    event.preventDefault();
  };

  /**
   * Handle the touchmove 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onTouchMove(event) {
    onMouseMove(event);
    event.preventDefault();
  };

  // add event listeners
  this.addEventListener('mousedown', onMouseDown);
  this.addEventListener('mouseup', onMouseUp);
  this.addEventListener('mousemove', onMouseMove);
  this.addEventListener('touchstart', onTouchDown);
  this.addEventListener('touchmove', onTouchMove);
  // Chrome/Firefox have different events here
  this.addEventListener('mousewheel', onMouseWheel);
  this.addEventListener('DOMMouseScroll', onMouseWheel);
};

/**
 * Display the main axes for 1 second.
 */
ROS3D.OrbitControls.prototype.showAxes = function() {
  var that = this;

  this.axes.traverse(function(obj) {
    obj.visible = true;
  });
  if (this.hideTimeout) {
    clearTimeout(this.hideTimeout);
  }
  this.hideTimeout = setTimeout(function() {
    that.axes.traverse(function(obj) {
      obj.visible = false;
    });
    that.hideTimeout = false;
  }, 1000);
};

/**
 * Rotate the camera to the left by the given angle.
 * 
 * @param angle (optional) - the angle to rotate by
 */
ROS3D.OrbitControls.prototype.rotateLeft = function(angle) {
  if (angle === undefined) {
    angle = 2 * Math.PI / 60 / 60 * this.autoRotateSpeed;
  }
  this.thetaDelta -= angle;
};

/**
 * Rotate the camera to the right by the given angle.
 * 
 * @param angle (optional) - the angle to rotate by
 */
ROS3D.OrbitControls.prototype.rotateRight = function(angle) {
  if (angle === undefined) {
    angle = 2 * Math.PI / 60 / 60 * this.autoRotateSpeed;
  }
  this.thetaDelta += angle;
};

/**
 * Rotate the camera up by the given angle.
 * 
 * @param angle (optional) - the angle to rotate by
 */
ROS3D.OrbitControls.prototype.rotateUp = function(angle) {
  if (angle === undefined) {
    angle = 2 * Math.PI / 60 / 60 * this.autoRotateSpeed;
  }
  this.phiDelta -= angle;
};

/**
 * Rotate the camera down by the given angle.
 * 
 * @param angle (optional) - the angle to rotate by
 */
ROS3D.OrbitControls.prototype.rotateDown = function(angle) {
  if (angle === undefined) {
    angle = 2 * Math.PI / 60 / 60 * this.autoRotateSpeed;
  }
  this.phiDelta += angle;
};

/**
 * Zoom in by the given scale.
 * 
 * @param zoomScale (optional) - the scale to zoom in by
 */
ROS3D.OrbitControls.prototype.zoomIn = function(zoomScale) {
  if (zoomScale === undefined) {
    zoomScale = Math.pow(0.95, this.userZoomSpeed);
  }
  this.scale /= zoomScale;
};

/**
 * Zoom out by the given scale.
 * 
 * @param zoomScale (optional) - the scale to zoom in by
 */
ROS3D.OrbitControls.prototype.zoomOut = function(zoomScale) {
  if (zoomScale === undefined) {
    zoomScale = Math.pow(0.95, this.userZoomSpeed);
  }
  this.scale *= zoomScale;
};

/**
 * Update the camera to the current settings.
 */
ROS3D.OrbitControls.prototype.update = function() {
  // x->y, y->z, z->x
  var position = this.camera.position;
  var offset = position.clone().sub(this.center);

  // angle from z-axis around y-axis
  var theta = Math.atan2(offset.y, offset.x);

  // angle from y-axis
  var phi = Math.atan2(Math.sqrt(offset.y * offset.y + offset.x * offset.x), offset.z);

  if (this.autoRotate) {
    this.rotateLeft(2 * Math.PI / 60 / 60 * this.autoRotateSpeed);
  }

  theta += this.thetaDelta;
  phi += this.phiDelta;

  // restrict phi to be betwee EPS and PI-EPS
  var eps = 0.000001;
  phi = Math.max(eps, Math.min(Math.PI - eps, phi));

  var radius = offset.length();
  offset.y = radius * Math.sin(phi) * Math.sin(theta);
  offset.z = radius * Math.cos(phi);
  offset.x = radius * Math.sin(phi) * Math.cos(theta);
  offset.multiplyScalar(this.scale);

  position.copy(this.center).add(offset);

  this.camera.lookAt(this.center);

  radius = offset.length();
  this.axes.position = this.center.clone();
  this.axes.scale.x = this.axes.scale.y = this.axes.scale.z = radius * 0.05;
  this.axes.updateMatrixWorld(true);

  this.thetaDelta = 0;
  this.phiDelta = 0;
  this.scale = 1;

  if (this.lastPosition.distanceTo(this.camera.position) > 0) {
    this.dispatchEvent({
      type : 'change'
    });
    this.lastPosition.copy(this.camera.position);
  }
};
/**
 * @author Tim Knip / http://www.floorplanner.com/ / tim at floorplanner.com
 */
THREE.ColladaLoader = function() {

  var COLLADA = null;
  var scene = null;
  var daeScene;

  var readyCallbackFunc = null;

  var sources = {};
  var images = {};
  var animations = {};
  var controllers = {};
  var geometries = {};
  var materials = {};
  var effects = {};
  var cameras = {};

  var animData;
  var visualScenes;
  var baseUrl;
  var morphs;
  var skins;

  var flip_uv = true;
  var preferredShading = THREE.SmoothShading;

  var options = {
    // Force Geometry to always be centered at the local origin of the
    // containing Mesh.
    centerGeometry : false,

    // Axis conversion is done for geometries, animations, and controllers.
    // If we ever pull cameras or lights out of the COLLADA file, they'll
    // need extra work.
    convertUpAxis : false,

    subdivideFaces : true,

    upAxis : 'Y',

    // For reflective or refractive materials we'll use this cubemap
    defaultEnvMap : null

  };

  var colladaUnit = 1.0;
  var colladaUp = 'Y';
  var upConversion = null;

  var TO_RADIANS = Math.PI / 180;

  function load(url, readyCallback, progressCallback) {

    var length = 0;

    if (document.implementation && document.implementation.createDocument) {

      var request = new XMLHttpRequest();

      request.onreadystatechange = function() {

        if (request.readyState == 4) {

          if (request.status == 0 || request.status == 200) {

            if (request.responseXML) {

              readyCallbackFunc = readyCallback;
              parse(request.responseXML, undefined, url);

            } else if (request.responseText) {

              readyCallbackFunc = readyCallback;
              var xmlParser = new DOMParser();
              var responseXML = xmlParser.parseFromString(request.responseText, "application/xml");
              parse(responseXML, undefined, url);

            } else {

              console.error("ColladaLoader: Empty or non-existing file (" + url + ")");

            }

          }

        } else if (request.readyState == 3) {

          if (progressCallback) {

            if (length == 0) {

              length = request.getResponseHeader("Content-Length");

            }

            progressCallback({
              total : length,
              loaded : request.responseText.length
            });

          }

        }

      };

      request.open("GET", url, true);
      request.send(null);

    } else {

      alert("Don't know how to parse XML!");

    }

  };

  function parse(doc, callBack, url) {

    COLLADA = doc;
    callBack = callBack || readyCallbackFunc;

    if (url !== undefined) {

      var parts = url.split('/');
      parts.pop();
      baseUrl = (parts.length < 1 ? '.' : parts.join('/')) + '/';

    }

    parseAsset();
    setUpConversion();
    images = parseLib("//dae:library_images/dae:image", _Image, "image");
    materials = parseLib("//dae:library_materials/dae:material", Material, "material");
    effects = parseLib("//dae:library_effects/dae:effect", Effect, "effect");
    geometries = parseLib("//dae:library_geometries/dae:geometry", Geometry, "geometry");
    cameras = parseLib(".//dae:library_cameras/dae:camera", Camera, "camera");
    controllers = parseLib("//dae:library_controllers/dae:controller", Controller, "controller");
    animations = parseLib("//dae:library_animations/dae:animation", Animation, "animation");
    visualScenes = parseLib(".//dae:library_visual_scenes/dae:visual_scene", VisualScene,
        "visual_scene");

    morphs = [];
    skins = [];

    daeScene = parseScene();
    scene = new THREE.Object3D();

    for ( var i = 0; i < daeScene.nodes.length; i++) {
      scene.add(createSceneGraph(daeScene.nodes[i]));
    }

    createAnimations();

    var result = {

      scene : scene,
      morphs : morphs,
      skins : skins,
      animations : animData,
      dae : {
        images : images,
        materials : materials,
        cameras : cameras,
        effects : effects,
        geometries : geometries,
        controllers : controllers,
        animations : animations,
        visualScenes : visualScenes,
        scene : daeScene
      }

    };

    if (callBack) {

      callBack(result);

    }

    return result;

  };

  function setPreferredShading(shading) {

    preferredShading = shading;

  };

  function parseAsset() {

    var elements = COLLADA.evaluate('//dae:asset', COLLADA, _nsResolver,
        XPathResult.ORDERED_NODE_ITERATOR_TYPE, null);

    var element = elements.iterateNext();

    if (element && element.childNodes) {

      for ( var i = 0; i < element.childNodes.length; i++) {

        var child = element.childNodes[i];

        switch (child.nodeName) {

          case 'unit':

            var meter = child.getAttribute('meter');

            if (meter) {

              colladaUnit = parseFloat(meter);

            }

            break;

          case 'up_axis':

            colladaUp = child.textContent.charAt(0);
            break;

        }

      }

    }

  };

  function parseLib(q, classSpec, prefix) {

    var elements = COLLADA.evaluate(q, COLLADA, _nsResolver,
        XPathResult.ORDERED_NODE_ITERATOR_TYPE, null);

    var lib = {};
    var element = elements.iterateNext();
    var i = 0;

    while (element) {

      var daeElement = (new classSpec()).parse(element);
      if (!daeElement.id || daeElement.id.length == 0)
        daeElement.id = prefix + (i++);
      lib[daeElement.id] = daeElement;

      element = elements.iterateNext();

    }

    return lib;

  };

  function parseScene() {

    var sceneElement = COLLADA.evaluate('.//dae:scene/dae:instance_visual_scene', COLLADA,
        _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null).iterateNext();

    if (sceneElement) {

      var url = sceneElement.getAttribute('url').replace(/^#/, '');
      return visualScenes[url.length > 0 ? url : 'visual_scene0'];

    } else {

      return null;

    }

  };

  function createAnimations() {

    animData = [];

    // fill in the keys
    recurseHierarchy(scene);

  };

  function recurseHierarchy(node) {

    var n = daeScene.getChildById(node.name, true), newData = null;

    if (n && n.keys) {

      newData = {
        fps : 60,
        hierarchy : [ {
          node : n,
          keys : n.keys,
          sids : n.sids
        } ],
        node : node,
        name : 'animation_' + node.name,
        length : 0
      };

      animData.push(newData);

      for ( var i = 0, il = n.keys.length; i < il; i++) {

        newData.length = Math.max(newData.length, n.keys[i].time);

      }

    } else {

      newData = {
        hierarchy : [ {
          keys : [],
          sids : []
        } ]
      };

    }

    for ( var i = 0, il = node.children.length; i < il; i++) {

      var d = recurseHierarchy(node.children[i]);

      for ( var j = 0, jl = d.hierarchy.length; j < jl; j++) {

        newData.hierarchy.push({
          keys : [],
          sids : []
        });

      }

    }

    return newData;

  };

  function calcAnimationBounds() {

    var start = 1000000;
    var end = -start;
    var frames = 0;

    for ( var id in animations) {

      var animation = animations[id];

      for ( var i = 0; i < animation.sampler.length; i++) {

        var sampler = animation.sampler[i];
        sampler.create();

        start = Math.min(start, sampler.startTime);
        end = Math.max(end, sampler.endTime);
        frames = Math.max(frames, sampler.input.length);

      }

    }

    return {
      start : start,
      end : end,
      frames : frames
    };

  };

  function createMorph(geometry, ctrl) {

    var morphCtrl = ctrl instanceof InstanceController ? controllers[ctrl.url] : ctrl;

    if (!morphCtrl || !morphCtrl.morph) {

      console.log("could not find morph controller!");
      return;

    }

    var morph = morphCtrl.morph;

    for ( var i = 0; i < morph.targets.length; i++) {

      var target_id = morph.targets[i];
      var daeGeometry = geometries[target_id];

      if (!daeGeometry.mesh || !daeGeometry.mesh.primitives || !daeGeometry.mesh.primitives.length) {
        continue;
      }

      var target = daeGeometry.mesh.primitives[0].geometry;

      if (target.vertices.length === geometry.vertices.length) {

        geometry.morphTargets.push({
          name : "target_1",
          vertices : target.vertices
        });

      }

    }

    geometry.morphTargets.push({
      name : "target_Z",
      vertices : geometry.vertices
    });

  };

  function createSkin(geometry, ctrl, applyBindShape) {

    var skinCtrl = controllers[ctrl.url];

    if (!skinCtrl || !skinCtrl.skin) {

      console.log("could not find skin controller!");
      return;

    }

    if (!ctrl.skeleton || !ctrl.skeleton.length) {

      console.log("could not find the skeleton for the skin!");
      return;

    }

    var skin = skinCtrl.skin;
    var skeleton = daeScene.getChildById(ctrl.skeleton[0]);
    var hierarchy = [];

    applyBindShape = applyBindShape !== undefined ? applyBindShape : true;

    var bones = [];
    geometry.skinWeights = [];
    geometry.skinIndices = [];

    // createBones( geometry.bones, skin, hierarchy, skeleton, null, -1 );
    // createWeights( skin, geometry.bones, geometry.skinIndices, geometry.skinWeights );

    /*
    geometry.animation = {
    	name: 'take_001',
    	fps: 30,
    	length: 2,
    	JIT: true,
    	hierarchy: hierarchy
    };
    */

    if (applyBindShape) {

      for ( var i = 0; i < geometry.vertices.length; i++) {

        skin.bindShapeMatrix.multiplyVector3(geometry.vertices[i]);

      }

    }

  };

  function setupSkeleton(node, bones, frame, parent) {

    node.world = node.world || new THREE.Matrix4();
    node.world.copy(node.matrix);

    if (node.channels && node.channels.length) {

      var channel = node.channels[0];
      var m = channel.sampler.output[frame];

      if (m instanceof THREE.Matrix4) {

        node.world.copy(m);

      }

    }

    if (parent) {

      node.world.multiply(parent, node.world);

    }

    bones.push(node);

    for ( var i = 0; i < node.nodes.length; i++) {

      setupSkeleton(node.nodes[i], bones, frame, node.world);

    }

  };

  function setupSkinningMatrices(bones, skin) {

    for ( var i = 0; i < bones.length; i++) {

      var bone = bones[i];
      var found = -1;

      if (bone.type != 'JOINT')
        continue;

      for ( var j = 0; j < skin.joints.length; j++) {

        if (bone.sid == skin.joints[j]) {

          found = j;
          break;

        }

      }

      if (found >= 0) {

        var inv = skin.invBindMatrices[found];

        bone.invBindMatrix = inv;
        bone.skinningMatrix = new THREE.Matrix4();
        bone.skinningMatrix.multiply(bone.world, inv); // (IBMi * JMi)

        bone.weights = [];

        for ( var j = 0; j < skin.weights.length; j++) {

          for ( var k = 0; k < skin.weights[j].length; k++) {

            var w = skin.weights[j][k];

            if (w.joint == found) {

              bone.weights.push(w);

            }

          }

        }

      } else {

        throw 'ColladaLoader: Could not find joint \'' + bone.sid + '\'.';

      }

    }

  };

  function applySkin(geometry, instanceCtrl, frame) {

    var skinController = controllers[instanceCtrl.url];

    frame = frame !== undefined ? frame : 40;

    if (!skinController || !skinController.skin) {

      console.log('ColladaLoader: Could not find skin controller.');
      return;

    }

    if (!instanceCtrl.skeleton || !instanceCtrl.skeleton.length) {

      console.log('ColladaLoader: Could not find the skeleton for the skin. ');
      return;

    }

    var animationBounds = calcAnimationBounds();
    var skeleton = daeScene.getChildById(instanceCtrl.skeleton[0], true)
        || daeScene.getChildBySid(instanceCtrl.skeleton[0], true);

    var i, j, w, vidx, weight;
    var v = new THREE.Vector3(), o, s;

    // move vertices to bind shape

    for (i = 0; i < geometry.vertices.length; i++) {

      skinController.skin.bindShapeMatrix.multiplyVector3(geometry.vertices[i]);

    }

    // process animation, or simply pose the rig if no animation

    for (frame = 0; frame < animationBounds.frames; frame++) {

      var bones = [];
      var skinned = [];

      // zero skinned vertices

      for (i = 0; i < geometry.vertices.length; i++) {

        skinned.push(new THREE.Vector3());

      }

      // process the frame and setup the rig with a fresh
      // transform, possibly from the bone's animation channel(s)

      setupSkeleton(skeleton, bones, frame);
      setupSkinningMatrices(bones, skinController.skin);

      // skin 'm

      for (i = 0; i < bones.length; i++) {

        if (bones[i].type != 'JOINT')
          continue;

        for (j = 0; j < bones[i].weights.length; j++) {

          w = bones[i].weights[j];
          vidx = w.index;
          weight = w.weight;

          o = geometry.vertices[vidx];
          s = skinned[vidx];

          v.x = o.x;
          v.y = o.y;
          v.z = o.z;

          bones[i].skinningMatrix.multiplyVector3(v);

          s.x += (v.x * weight);
          s.y += (v.y * weight);
          s.z += (v.z * weight);

        }

      }

      geometry.morphTargets.push({
        name : "target_" + frame,
        vertices : skinned
      });

    }

  };

  function createSceneGraph(node, parent) {

    var obj = new THREE.Object3D();
    var skinned = false;
    var skinController;
    var morphController;
    var i, j;

    for (i = 0; i < node.controllers.length; i++) {

      var controller = controllers[node.controllers[i].url];

      switch (controller.type) {

        case 'skin':

          if (geometries[controller.skin.source]) {

            var inst_geom = new InstanceGeometry();

            inst_geom.url = controller.skin.source;
            inst_geom.instance_material = node.controllers[i].instance_material;

            node.geometries.push(inst_geom);
            skinned = true;
            skinController = node.controllers[i];

          } else if (controllers[controller.skin.source]) {

            // urgh: controller can be chained
            // handle the most basic case...

            var second = controllers[controller.skin.source];
            morphController = second;
            // skinController = node.controllers[i];

            if (second.morph && geometries[second.morph.source]) {

              var inst_geom = new InstanceGeometry();

              inst_geom.url = second.morph.source;
              inst_geom.instance_material = node.controllers[i].instance_material;

              node.geometries.push(inst_geom);

            }

          }

          break;

        case 'morph':

          if (geometries[controller.morph.source]) {

            var inst_geom = new InstanceGeometry();

            inst_geom.url = controller.morph.source;
            inst_geom.instance_material = node.controllers[i].instance_material;

            node.geometries.push(inst_geom);
            morphController = node.controllers[i];

          }

          console.log('ColladaLoader: Morph-controller partially supported.');

        default:
          break;

      }

    }

    // geometries

    var double_sided_materials = {};

    for (i = 0; i < node.geometries.length; i++) {

      var instance_geometry = node.geometries[i];
      var instance_materials = instance_geometry.instance_material;
      var geometry = geometries[instance_geometry.url];
      var used_materials = {};
      var used_materials_array = [];
      var num_materials = 0;
      var first_material;

      if (geometry) {

        if (!geometry.mesh || !geometry.mesh.primitives)
          continue;

        if (obj.name.length == 0) {

          obj.name = geometry.id;

        }

        // collect used fx for this geometry-instance

        if (instance_materials) {

          for (j = 0; j < instance_materials.length; j++) {

            var instance_material = instance_materials[j];
            var mat = materials[instance_material.target];
            var effect_id = mat.instance_effect.url;
            var shader = effects[effect_id].shader;
            var material3js = shader.material;

            if (geometry.doubleSided) {

              if (!(material3js in double_sided_materials)) {

                var _copied_material = material3js.clone();
                _copied_material.side = THREE.DoubleSide;
                double_sided_materials[material3js] = _copied_material;

              }

              material3js = double_sided_materials[material3js];

            }

            material3js.opacity = !material3js.opacity ? 1 : material3js.opacity;
            used_materials[instance_material.symbol] = num_materials;
            used_materials_array.push(material3js);
            first_material = material3js;
            first_material.name = mat.name == null || mat.name === '' ? mat.id : mat.name;
            num_materials++;

          }

        }

        var mesh;
        var material = first_material || new THREE.MeshLambertMaterial({
          color : 0xdddddd,
          shading : THREE.FlatShading,
          side : geometry.doubleSided ? THREE.DoubleSide : THREE.FrontSide
        });
        var geom = geometry.mesh.geometry3js;

        if (num_materials > 1) {

          material = new THREE.MeshFaceMaterial(used_materials_array);

          for (j = 0; j < geom.faces.length; j++) {

            var face = geom.faces[j];
            face.materialIndex = used_materials[face.daeMaterial];

          }

        }

        if (skinController !== undefined) {

          applySkin(geom, skinController);

          material.morphTargets = true;

          mesh = new THREE.SkinnedMesh(geom, material, false);
          mesh.skeleton = skinController.skeleton;
          mesh.skinController = controllers[skinController.url];
          mesh.skinInstanceController = skinController;
          mesh.name = 'skin_' + skins.length;

          skins.push(mesh);

        } else if (morphController !== undefined) {

          createMorph(geom, morphController);

          material.morphTargets = true;

          mesh = new THREE.Mesh(geom, material);
          mesh.name = 'morph_' + morphs.length;

          morphs.push(mesh);

        } else {

          mesh = new THREE.Mesh(geom, material);
          // mesh.geom.name = geometry.id;

        }

        node.geometries.length > 1 ? obj.add(mesh) : obj = mesh;

      }

    }

    for (i = 0; i < node.cameras.length; i++) {

      var instance_camera = node.cameras[i];
      var cparams = cameras[instance_camera.url];

      obj = new THREE.PerspectiveCamera(cparams.fov, cparams.aspect_ratio, cparams.znear,
          cparams.zfar);

    }

    obj.name = node.id || "";
    obj.matrix = node.matrix;
    var props = node.matrix.decompose();
    obj.position = props[0];
    obj.quaternion = props[1];
    obj.useQuaternion = true;
    obj.scale = props[2];
    obj.position.multiplyScalar(colladaUnit);
    obj.scale.multiplyScalar(colladaUnit);

    if (options.centerGeometry && obj.geometry) {

      var delta = THREE.GeometryUtils.center(obj.geometry);
      obj.quaternion.multiplyVector3(delta.multiply(obj.scale));
      obj.position.sub(delta);

    }

    for (i = 0; i < node.nodes.length; i++) {

      obj.add(createSceneGraph(node.nodes[i], node));

    }

    return obj;

  };

  function getJointId(skin, id) {

    for ( var i = 0; i < skin.joints.length; i++) {

      if (skin.joints[i] == id) {

        return i;

      }

    }

  };

  function getLibraryNode(id) {

    return COLLADA.evaluate('.//dae:library_nodes//dae:node[@id=\'' + id + '\']', COLLADA,
        _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null).iterateNext();

  };

  function getChannelsForNode(node) {

    var channels = [];
    var startTime = 1000000;
    var endTime = -1000000;

    for ( var id in animations) {

      var animation = animations[id];

      for ( var i = 0; i < animation.channel.length; i++) {

        var channel = animation.channel[i];
        var sampler = animation.sampler[i];
        var id = channel.target.split('/')[0];

        if (id == node.id) {

          sampler.create();
          channel.sampler = sampler;
          startTime = Math.min(startTime, sampler.startTime);
          endTime = Math.max(endTime, sampler.endTime);
          channels.push(channel);

        }

      }

    }

    if (channels.length) {

      node.startTime = startTime;
      node.endTime = endTime;

    }

    return channels;

  };

  function calcFrameDuration(node) {

    var minT = 10000000;

    for ( var i = 0; i < node.channels.length; i++) {

      var sampler = node.channels[i].sampler;

      for ( var j = 0; j < sampler.input.length - 1; j++) {

        var t0 = sampler.input[j];
        var t1 = sampler.input[j + 1];
        minT = Math.min(minT, t1 - t0);

      }
    }

    return minT;

  };

  function calcMatrixAt(node, t) {

    var animated = {};

    var i, j;

    for (i = 0; i < node.channels.length; i++) {

      var channel = node.channels[i];
      animated[channel.sid] = channel;

    }

    var matrix = new THREE.Matrix4();

    for (i = 0; i < node.transforms.length; i++) {

      var transform = node.transforms[i];
      var channel = animated[transform.sid];

      if (channel !== undefined) {

        var sampler = channel.sampler;
        var value;

        for (j = 0; j < sampler.input.length - 1; j++) {

          if (sampler.input[j + 1] > t) {

            value = sampler.output[j];
            // console.log(value.flatten)
            break;

          }

        }

        if (value !== undefined) {

          if (value instanceof THREE.Matrix4) {

            matrix = matrix.multiply(matrix, value);

          } else {

            matrix = matrix.multiply(matrix, transform.matrix);

          }

        } else {

          matrix = matrix.multiply(matrix, transform.matrix);

        }

      } else {

        matrix = matrix.multiply(matrix, transform.matrix);

      }

    }

    return matrix;

  };

  function bakeAnimations(node) {

    if (node.channels && node.channels.length) {

      var keys = [], sids = [];

      for ( var i = 0, il = node.channels.length; i < il; i++) {

        var channel = node.channels[i], fullSid = channel.fullSid, sampler = channel.sampler, input = sampler.input, transform = node
            .getTransformBySid(channel.sid), member;

        if (channel.arrIndices) {

          member = [];

          for ( var j = 0, jl = channel.arrIndices.length; j < jl; j++) {

            member[j] = getConvertedIndex(channel.arrIndices[j]);

          }

        } else {

          member = getConvertedMember(channel.member);

        }

        if (transform) {

          if (sids.indexOf(fullSid) === -1) {

            sids.push(fullSid);

          }

          for ( var j = 0, jl = input.length; j < jl; j++) {

            var time = input[j], data = sampler.getData(transform.type, j), key = findKey(keys,
                time);

            if (!key) {

              key = new Key(time);
              var timeNdx = findTimeNdx(keys, time);
              keys.splice(timeNdx == -1 ? keys.length : timeNdx, 0, key);

            }

            key.addTarget(fullSid, transform, member, data);

          }

        } else {

          console.log('Could not find transform "' + channel.sid + '" in node ' + node.id);

        }

      }

      // post process
      for ( var i = 0; i < sids.length; i++) {

        var sid = sids[i];

        for ( var j = 0; j < keys.length; j++) {

          var key = keys[j];

          if (!key.hasTarget(sid)) {

            interpolateKeys(keys, key, j, sid);

          }

        }

      }

      node.keys = keys;
      node.sids = sids;

    }

  };

  function findKey(keys, time) {

    var retVal = null;

    for ( var i = 0, il = keys.length; i < il && retVal == null; i++) {

      var key = keys[i];

      if (key.time === time) {

        retVal = key;

      } else if (key.time > time) {

        break;

      }

    }

    return retVal;

  };

  function findTimeNdx(keys, time) {

    var ndx = -1;

    for ( var i = 0, il = keys.length; i < il && ndx == -1; i++) {

      var key = keys[i];

      if (key.time >= time) {

        ndx = i;

      }

    }

    return ndx;

  };

  function interpolateKeys(keys, key, ndx, fullSid) {

    var prevKey = getPrevKeyWith(keys, fullSid, ndx ? ndx - 1 : 0), nextKey = getNextKeyWith(keys,
        fullSid, ndx + 1);

    if (prevKey && nextKey) {

      var scale = (key.time - prevKey.time) / (nextKey.time - prevKey.time), prevTarget = prevKey
          .getTarget(fullSid), nextData = nextKey.getTarget(fullSid).data, prevData = prevTarget.data, data;

      if (prevTarget.type === 'matrix') {

        data = prevData;

      } else if (prevData.length) {

        data = [];

        for ( var i = 0; i < prevData.length; ++i) {

          data[i] = prevData[i] + (nextData[i] - prevData[i]) * scale;

        }

      } else {

        data = prevData + (nextData - prevData) * scale;

      }

      key.addTarget(fullSid, prevTarget.transform, prevTarget.member, data);

    }

  };

  // Get next key with given sid

  function getNextKeyWith(keys, fullSid, ndx) {

    for (; ndx < keys.length; ndx++) {

      var key = keys[ndx];

      if (key.hasTarget(fullSid)) {

        return key;

      }

    }

    return null;

  };

  // Get previous key with given sid

  function getPrevKeyWith(keys, fullSid, ndx) {

    ndx = ndx >= 0 ? ndx : ndx + keys.length;

    for (; ndx >= 0; ndx--) {

      var key = keys[ndx];

      if (key.hasTarget(fullSid)) {

        return key;

      }

    }

    return null;

  };

  function _Image() {

    this.id = "";
    this.init_from = "";

  };

  _Image.prototype.parse = function(element) {

    this.id = element.getAttribute('id');

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      if (child.nodeName == 'init_from') {

        this.init_from = child.textContent;

      }

    }

    return this;

  };

  function Controller() {

    this.id = "";
    this.name = "";
    this.type = "";
    this.skin = null;
    this.morph = null;

  };

  Controller.prototype.parse = function(element) {

    this.id = element.getAttribute('id');
    this.name = element.getAttribute('name');
    this.type = "none";

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      switch (child.nodeName) {

        case 'skin':

          this.skin = (new Skin()).parse(child);
          this.type = child.nodeName;
          break;

        case 'morph':

          this.morph = (new Morph()).parse(child);
          this.type = child.nodeName;
          break;

        default:
          break;

      }
    }

    return this;

  };

  function Morph() {

    this.method = null;
    this.source = null;
    this.targets = null;
    this.weights = null;

  };

  Morph.prototype.parse = function(element) {

    var sources = {};
    var inputs = [];
    var i;

    this.method = element.getAttribute('method');
    this.source = element.getAttribute('source').replace(/^#/, '');

    for (i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'source':

          var source = (new Source()).parse(child);
          sources[source.id] = source;
          break;

        case 'targets':

          inputs = this.parseInputs(child);
          break;

        default:

          console.log(child.nodeName);
          break;

      }

    }

    for (i = 0; i < inputs.length; i++) {

      var input = inputs[i];
      var source = sources[input.source];

      switch (input.semantic) {

        case 'MORPH_TARGET':

          this.targets = source.read();
          break;

        case 'MORPH_WEIGHT':

          this.weights = source.read();
          break;

        default:
          break;

      }
    }

    return this;

  };

  Morph.prototype.parseInputs = function(element) {

    var inputs = [];

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'input':

          inputs.push((new Input()).parse(child));
          break;

        default:
          break;
      }
    }

    return inputs;

  };

  function Skin() {

    this.source = "";
    this.bindShapeMatrix = null;
    this.invBindMatrices = [];
    this.joints = [];
    this.weights = [];

  };

  Skin.prototype.parse = function(element) {

    var sources = {};
    var joints, weights;

    this.source = element.getAttribute('source').replace(/^#/, '');
    this.invBindMatrices = [];
    this.joints = [];
    this.weights = [];

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'bind_shape_matrix':

          var f = _floats(child.textContent);
          this.bindShapeMatrix = getConvertedMat4(f);
          break;

        case 'source':

          var src = new Source().parse(child);
          sources[src.id] = src;
          break;

        case 'joints':

          joints = child;
          break;

        case 'vertex_weights':

          weights = child;
          break;

        default:

          console.log(child.nodeName);
          break;

      }
    }

    this.parseJoints(joints, sources);
    this.parseWeights(weights, sources);

    return this;

  };

  Skin.prototype.parseJoints = function(element, sources) {

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'input':

          var input = (new Input()).parse(child);
          var source = sources[input.source];

          if (input.semantic == 'JOINT') {

            this.joints = source.read();

          } else if (input.semantic == 'INV_BIND_MATRIX') {

            this.invBindMatrices = source.read();

          }

          break;

        default:
          break;
      }

    }

  };

  Skin.prototype.parseWeights = function(element, sources) {

    var v, vcount, inputs = [];

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'input':

          inputs.push((new Input()).parse(child));
          break;

        case 'v':

          v = _ints(child.textContent);
          break;

        case 'vcount':

          vcount = _ints(child.textContent);
          break;

        default:
          break;

      }

    }

    var index = 0;

    for ( var i = 0; i < vcount.length; i++) {

      var numBones = vcount[i];
      var vertex_weights = [];

      for ( var j = 0; j < numBones; j++) {

        var influence = {};

        for ( var k = 0; k < inputs.length; k++) {

          var input = inputs[k];
          var value = v[index + input.offset];

          switch (input.semantic) {

            case 'JOINT':

              influence.joint = value;// this.joints[value];
              break;

            case 'WEIGHT':

              influence.weight = sources[input.source].data[value];
              break;

            default:
              break;

          }

        }

        vertex_weights.push(influence);
        index += inputs.length;
      }

      for ( var j = 0; j < vertex_weights.length; j++) {

        vertex_weights[j].index = i;

      }

      this.weights.push(vertex_weights);

    }

  };

  function VisualScene() {

    this.id = "";
    this.name = "";
    this.nodes = [];
    this.scene = new THREE.Object3D();

  };

  VisualScene.prototype.getChildById = function(id, recursive) {

    for ( var i = 0; i < this.nodes.length; i++) {

      var node = this.nodes[i].getChildById(id, recursive);

      if (node) {

        return node;

      }

    }

    return null;

  };

  VisualScene.prototype.getChildBySid = function(sid, recursive) {

    for ( var i = 0; i < this.nodes.length; i++) {

      var node = this.nodes[i].getChildBySid(sid, recursive);

      if (node) {

        return node;

      }

    }

    return null;

  };

  VisualScene.prototype.parse = function(element) {

    this.id = element.getAttribute('id');
    this.name = element.getAttribute('name');
    this.nodes = [];

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'node':

          this.nodes.push((new Node()).parse(child));
          break;

        default:
          break;

      }

    }

    return this;

  };

  function Node() {

    this.id = "";
    this.name = "";
    this.sid = "";
    this.nodes = [];
    this.controllers = [];
    this.transforms = [];
    this.geometries = [];
    this.channels = [];
    this.matrix = new THREE.Matrix4();

  };

  Node.prototype.getChannelForTransform = function(transformSid) {

    for ( var i = 0; i < this.channels.length; i++) {

      var channel = this.channels[i];
      var parts = channel.target.split('/');
      var id = parts.shift();
      var sid = parts.shift();
      var dotSyntax = (sid.indexOf(".") >= 0);
      var arrSyntax = (sid.indexOf("(") >= 0);
      var arrIndices;
      var member;

      if (dotSyntax) {

        parts = sid.split(".");
        sid = parts.shift();
        member = parts.shift();

      } else if (arrSyntax) {

        arrIndices = sid.split("(");
        sid = arrIndices.shift();

        for ( var j = 0; j < arrIndices.length; j++) {

          arrIndices[j] = parseInt(arrIndices[j].replace(/\)/, ''));

        }

      }

      if (sid == transformSid) {

        channel.info = {
          sid : sid,
          dotSyntax : dotSyntax,
          arrSyntax : arrSyntax,
          arrIndices : arrIndices
        };
        return channel;

      }

    }

    return null;

  };

  Node.prototype.getChildById = function(id, recursive) {

    if (this.id == id) {

      return this;

    }

    if (recursive) {

      for ( var i = 0; i < this.nodes.length; i++) {

        var n = this.nodes[i].getChildById(id, recursive);

        if (n) {

          return n;

        }

      }

    }

    return null;

  };

  Node.prototype.getChildBySid = function(sid, recursive) {

    if (this.sid == sid) {

      return this;

    }

    if (recursive) {

      for ( var i = 0; i < this.nodes.length; i++) {

        var n = this.nodes[i].getChildBySid(sid, recursive);

        if (n) {

          return n;

        }

      }
    }

    return null;

  };

  Node.prototype.getTransformBySid = function(sid) {

    for ( var i = 0; i < this.transforms.length; i++) {

      if (this.transforms[i].sid == sid)
        return this.transforms[i];

    }

    return null;

  };

  Node.prototype.parse = function(element) {

    var url;

    this.id = element.getAttribute('id');
    this.sid = element.getAttribute('sid');
    this.name = element.getAttribute('name');
    this.type = element.getAttribute('type');

    this.type = this.type == 'JOINT' ? this.type : 'NODE';

    this.nodes = [];
    this.transforms = [];
    this.geometries = [];
    this.cameras = [];
    this.controllers = [];
    this.matrix = new THREE.Matrix4();

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'node':

          this.nodes.push((new Node()).parse(child));
          break;

        case 'instance_camera':

          this.cameras.push((new InstanceCamera()).parse(child));
          break;

        case 'instance_controller':

          this.controllers.push((new InstanceController()).parse(child));
          break;

        case 'instance_geometry':

          this.geometries.push((new InstanceGeometry()).parse(child));
          break;

        case 'instance_light':

          break;

        case 'instance_node':

          url = child.getAttribute('url').replace(/^#/, '');
          var iNode = getLibraryNode(url);

          if (iNode) {

            this.nodes.push((new Node()).parse(iNode));

          }

          break;

        case 'rotate':
        case 'translate':
        case 'scale':
        case 'matrix':
        case 'lookat':
        case 'skew':

          this.transforms.push((new Transform()).parse(child));
          break;

        case 'extra':
          break;

        default:

          console.log(child.nodeName);
          break;

      }

    }

    this.channels = getChannelsForNode(this);
    bakeAnimations(this);

    this.updateMatrix();

    return this;

  };

  Node.prototype.updateMatrix = function() {

    this.matrix.identity();

    for ( var i = 0; i < this.transforms.length; i++) {

      this.transforms[i].apply(this.matrix);

    }

  };

  function Transform() {

    this.sid = "";
    this.type = "";
    this.data = [];
    this.obj = null;

  };

  Transform.prototype.parse = function(element) {

    this.sid = element.getAttribute('sid');
    this.type = element.nodeName;
    this.data = _floats(element.textContent);
    this.convert();

    return this;

  };

  Transform.prototype.convert = function() {

    switch (this.type) {

      case 'matrix':

        this.obj = getConvertedMat4(this.data);
        break;

      case 'rotate':

        this.angle = this.data[3] * TO_RADIANS;

      case 'translate':

        fixCoords(this.data, -1);
        this.obj = new THREE.Vector3(this.data[0], this.data[1], this.data[2]);
        break;

      case 'scale':

        fixCoords(this.data, 1);
        this.obj = new THREE.Vector3(this.data[0], this.data[1], this.data[2]);
        break;

      default:
        console.log('Can not convert Transform of type ' + this.type);
        break;

    }

  };

  Transform.prototype.apply = function(matrix) {

    switch (this.type) {

      case 'matrix':

        matrix.multiply(this.obj);
        break;

      case 'translate':

        matrix.translate(this.obj);
        break;

      case 'rotate':

        matrix.rotateByAxis(this.obj, this.angle);
        break;

      case 'scale':

        matrix.scale(this.obj);
        break;

    }

  };

  Transform.prototype.update = function(data, member) {

    var members = [ 'X', 'Y', 'Z', 'ANGLE' ];

    switch (this.type) {

      case 'matrix':

        if (!member) {

          this.obj.copy(data);

        } else if (member.length === 1) {

          switch (member[0]) {

            case 0:

              this.obj.n11 = data[0];
              this.obj.n21 = data[1];
              this.obj.n31 = data[2];
              this.obj.n41 = data[3];

              break;

            case 1:

              this.obj.n12 = data[0];
              this.obj.n22 = data[1];
              this.obj.n32 = data[2];
              this.obj.n42 = data[3];

              break;

            case 2:

              this.obj.n13 = data[0];
              this.obj.n23 = data[1];
              this.obj.n33 = data[2];
              this.obj.n43 = data[3];

              break;

            case 3:

              this.obj.n14 = data[0];
              this.obj.n24 = data[1];
              this.obj.n34 = data[2];
              this.obj.n44 = data[3];

              break;

          }

        } else if (member.length === 2) {

          var propName = 'n' + (member[0] + 1) + (member[1] + 1);
          this.obj[propName] = data;

        } else {

          console.log('Incorrect addressing of matrix in transform.');

        }

        break;

      case 'translate':
      case 'scale':

        if (Object.prototype.toString.call(member) === '[object Array]') {

          member = members[member[0]];

        }

        switch (member) {

          case 'X':

            this.obj.x = data;
            break;

          case 'Y':

            this.obj.y = data;
            break;

          case 'Z':

            this.obj.z = data;
            break;

          default:

            this.obj.x = data[0];
            this.obj.y = data[1];
            this.obj.z = data[2];
            break;

        }

        break;

      case 'rotate':

        if (Object.prototype.toString.call(member) === '[object Array]') {

          member = members[member[0]];

        }

        switch (member) {

          case 'X':

            this.obj.x = data;
            break;

          case 'Y':

            this.obj.y = data;
            break;

          case 'Z':

            this.obj.z = data;
            break;

          case 'ANGLE':

            this.angle = data * TO_RADIANS;
            break;

          default:

            this.obj.x = data[0];
            this.obj.y = data[1];
            this.obj.z = data[2];
            this.angle = data[3] * TO_RADIANS;
            break;

        }
        break;

    }

  };

  function InstanceController() {

    this.url = "";
    this.skeleton = [];
    this.instance_material = [];

  };

  InstanceController.prototype.parse = function(element) {

    this.url = element.getAttribute('url').replace(/^#/, '');
    this.skeleton = [];
    this.instance_material = [];

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType !== 1)
        continue;

      switch (child.nodeName) {

        case 'skeleton':

          this.skeleton.push(child.textContent.replace(/^#/, ''));
          break;

        case 'bind_material':

          var instances = COLLADA.evaluate('.//dae:instance_material', child, _nsResolver,
              XPathResult.ORDERED_NODE_ITERATOR_TYPE, null);

          if (instances) {

            var instance = instances.iterateNext();

            while (instance) {

              this.instance_material.push((new InstanceMaterial()).parse(instance));
              instance = instances.iterateNext();

            }

          }

          break;

        case 'extra':
          break;

        default:
          break;

      }
    }

    return this;

  };

  function InstanceMaterial() {

    this.symbol = "";
    this.target = "";

  };

  InstanceMaterial.prototype.parse = function(element) {

    this.symbol = element.getAttribute('symbol');
    this.target = element.getAttribute('target').replace(/^#/, '');
    return this;

  };

  function InstanceGeometry() {

    this.url = "";
    this.instance_material = [];

  };

  InstanceGeometry.prototype.parse = function(element) {

    this.url = element.getAttribute('url').replace(/^#/, '');
    this.instance_material = [];

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      if (child.nodeName == 'bind_material') {

        var instances = COLLADA.evaluate('.//dae:instance_material', child, _nsResolver,
            XPathResult.ORDERED_NODE_ITERATOR_TYPE, null);

        if (instances) {

          var instance = instances.iterateNext();

          while (instance) {

            this.instance_material.push((new InstanceMaterial()).parse(instance));
            instance = instances.iterateNext();

          }

        }

        break;

      }

    }

    return this;

  };

  function Geometry() {

    this.id = "";
    this.mesh = null;

  };

  Geometry.prototype.parse = function(element) {

    this.id = element.getAttribute('id');

    extractDoubleSided(this, element);

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      switch (child.nodeName) {

        case 'mesh':

          this.mesh = (new Mesh(this)).parse(child);
          break;

        case 'extra':

          // console.log( child );
          break;

        default:
          break;
      }
    }

    return this;

  };

  function Mesh(geometry) {

    this.geometry = geometry.id;
    this.primitives = [];
    this.vertices = null;
    this.geometry3js = null;

  };

  Mesh.prototype.parse = function(element) {

    this.primitives = [];

    var i, j;

    for (i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      switch (child.nodeName) {

        case 'source':

          _source(child);
          break;

        case 'vertices':

          this.vertices = (new Vertices()).parse(child);
          break;

        case 'triangles':

          this.primitives.push((new Triangles().parse(child)));
          break;

        case 'polygons':

          this.primitives.push((new Polygons().parse(child)));
          break;

        case 'polylist':

          this.primitives.push((new Polylist().parse(child)));
          break;

        default:
          break;

      }

    }

    this.geometry3js = new THREE.Geometry();

    var vertexData = sources[this.vertices.input['POSITION'].source].data;

    for (i = 0; i < vertexData.length; i += 3) {

      this.geometry3js.vertices.push(getConvertedVec3(vertexData, i).clone());

    }

    for (i = 0; i < this.primitives.length; i++) {

      var primitive = this.primitives[i];
      primitive.setVertices(this.vertices);
      this.handlePrimitive(primitive, this.geometry3js);

    }

    this.geometry3js.computeCentroids();
    this.geometry3js.computeFaceNormals();

    if (this.geometry3js.calcNormals) {

      this.geometry3js.computeVertexNormals();
      delete this.geometry3js.calcNormals;

    }

    this.geometry3js.computeBoundingBox();

    return this;

  };

  Mesh.prototype.handlePrimitive = function(primitive, geom) {

    var j, k, pList = primitive.p, inputs = primitive.inputs;
    var input, index, idx32;
    var source, numParams;
    var vcIndex = 0, vcount = 3, maxOffset = 0;
    var texture_sets = [];

    for (j = 0; j < inputs.length; j++) {

      input = inputs[j];
      var offset = input.offset + 1;
      maxOffset = (maxOffset < offset) ? offset : maxOffset;

      switch (input.semantic) {

        case 'TEXCOORD':
          texture_sets.push(input.set);
          break;

      }

    }

    for ( var pCount = 0; pCount < pList.length; ++pCount) {

      var p = pList[pCount], i = 0;

      while (i < p.length) {

        var vs = [];
        var ns = [];
        var ts = null;
        var cs = [];

        if (primitive.vcount) {

          vcount = primitive.vcount.length ? primitive.vcount[vcIndex++] : primitive.vcount;

        } else {

          vcount = p.length / maxOffset;

        }

        for (j = 0; j < vcount; j++) {

          for (k = 0; k < inputs.length; k++) {

            input = inputs[k];
            source = sources[input.source];

            index = p[i + (j * maxOffset) + input.offset];
            numParams = source.accessor.params.length;
            idx32 = index * numParams;

            switch (input.semantic) {

              case 'VERTEX':

                vs.push(index);

                break;

              case 'NORMAL':

                ns.push(getConvertedVec3(source.data, idx32));

                break;

              case 'TEXCOORD':

                ts = ts || {};
                if (ts[input.set] === undefined)
                  ts[input.set] = [];
                // invert the V
                ts[input.set].push(new THREE.Vector2(source.data[idx32], source.data[idx32 + 1]));

                break;

              case 'COLOR':

                cs.push(new THREE.Color().setRGB(source.data[idx32], source.data[idx32 + 1],
                    source.data[idx32 + 2]));

                break;

              default:

                break;

            }

          }

        }

        if (ns.length == 0) {

          // check the vertices inputs
          input = this.vertices.input.NORMAL;

          if (input) {

            source = sources[input.source];
            numParams = source.accessor.params.length;

            for ( var ndx = 0, len = vs.length; ndx < len; ndx++) {

              ns.push(getConvertedVec3(source.data, vs[ndx] * numParams));

            }

          } else {

            geom.calcNormals = true;

          }

        }

        if (!ts) {

          ts = {};
          // check the vertices inputs
          input = this.vertices.input.TEXCOORD;

          if (input) {

            texture_sets.push(input.set);
            source = sources[input.source];
            numParams = source.accessor.params.length;

            for ( var ndx = 0, len = vs.length; ndx < len; ndx++) {

              idx32 = vs[ndx] * numParams;
              if (ts[input.set] === undefined)
                ts[input.set] = [];
              // invert the V
              ts[input.set]
                  .push(new THREE.Vector2(source.data[idx32], 1.0 - source.data[idx32 + 1]));

            }

          }

        }

        if (cs.length == 0) {

          // check the vertices inputs
          input = this.vertices.input.COLOR;

          if (input) {

            source = sources[input.source];
            numParams = source.accessor.params.length;

            for ( var ndx = 0, len = vs.length; ndx < len; ndx++) {

              idx32 = vs[ndx] * numParams;
              cs.push(new THREE.Color().setRGB(source.data[idx32], source.data[idx32 + 1],
                  source.data[idx32 + 2]));

            }

          }

        }

        var face = null, faces = [], uv, uvArr;

        if (vcount === 3) {

          faces.push(new THREE.Face3(vs[0], vs[1], vs[2], ns, cs.length ? cs : new THREE.Color()));

        } else if (vcount === 4) {
          faces.push(new THREE.Face4(vs[0], vs[1], vs[2], vs[3], ns, cs.length ? cs
              : new THREE.Color()));

        } else if (vcount > 4 && options.subdivideFaces) {

          var clr = cs.length ? cs : new THREE.Color(), vec1, vec2, vec3, v1, v2, norm;

          // subdivide into multiple Face3s

          for (k = 1; k < vcount - 1;) {

            faces.push(new THREE.Face3(vs[0], vs[k], vs[k + 1], [ ns[0], ns[k++], ns[k] ], clr));

          }

        }

        if (faces.length) {

          for ( var ndx = 0, len = faces.length; ndx < len; ndx++) {

            face = faces[ndx];
            face.daeMaterial = primitive.material;
            geom.faces.push(face);

            for (k = 0; k < texture_sets.length; k++) {

              uv = ts[texture_sets[k]];

              if (vcount > 4) {

                // Grab the right UVs for the vertices in this face
                uvArr = [ uv[0], uv[ndx + 1], uv[ndx + 2] ];

              } else if (vcount === 4) {

                uvArr = [ uv[0], uv[1], uv[2], uv[3] ];

              } else {

                uvArr = [ uv[0], uv[1], uv[2] ];

              }

              if (!geom.faceVertexUvs[k]) {

                geom.faceVertexUvs[k] = [];

              }

              geom.faceVertexUvs[k].push(uvArr);

            }

          }

        } else {

          console.log('dropped face with vcount ' + vcount + ' for geometry with id: ' + geom.id);

        }

        i += maxOffset * vcount;

      }
    }

  };

  function Polygons() {

    this.material = "";
    this.count = 0;
    this.inputs = [];
    this.vcount = null;
    this.p = [];
    this.geometry = new THREE.Geometry();

  };

  Polygons.prototype.setVertices = function(vertices) {

    for ( var i = 0; i < this.inputs.length; i++) {

      if (this.inputs[i].source == vertices.id) {

        this.inputs[i].source = vertices.input['POSITION'].source;

      }

    }

  };

  Polygons.prototype.parse = function(element) {

    this.material = element.getAttribute('material');
    this.count = _attr_as_int(element, 'count', 0);

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      switch (child.nodeName) {

        case 'input':

          this.inputs.push((new Input()).parse(element.childNodes[i]));
          break;

        case 'vcount':

          this.vcount = _ints(child.textContent);
          break;

        case 'p':

          this.p.push(_ints(child.textContent));
          break;

        case 'ph':

          console.warn('polygon holes not yet supported!');
          break;

        default:
          break;

      }

    }

    return this;

  };

  function Polylist() {

    Polygons.call(this);

    this.vcount = [];

  };

  Polylist.prototype = Object.create(Polygons.prototype);

  function Triangles() {

    Polygons.call(this);

    this.vcount = 3;

  };

  Triangles.prototype = Object.create(Polygons.prototype);

  function Accessor() {

    this.source = "";
    this.count = 0;
    this.stride = 0;
    this.params = [];

  };

  Accessor.prototype.parse = function(element) {

    this.params = [];
    this.source = element.getAttribute('source');
    this.count = _attr_as_int(element, 'count', 0);
    this.stride = _attr_as_int(element, 'stride', 0);

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      if (child.nodeName == 'param') {

        var param = {};
        param['name'] = child.getAttribute('name');
        param['type'] = child.getAttribute('type');
        this.params.push(param);

      }

    }

    return this;

  };

  function Vertices() {

    this.input = {};

  };

  Vertices.prototype.parse = function(element) {

    this.id = element.getAttribute('id');

    for ( var i = 0; i < element.childNodes.length; i++) {

      if (element.childNodes[i].nodeName == 'input') {

        var input = (new Input()).parse(element.childNodes[i]);
        this.input[input.semantic] = input;

      }

    }

    return this;

  };

  function Input() {

    this.semantic = "";
    this.offset = 0;
    this.source = "";
    this.set = 0;

  };

  Input.prototype.parse = function(element) {

    this.semantic = element.getAttribute('semantic');
    this.source = element.getAttribute('source').replace(/^#/, '');
    this.set = _attr_as_int(element, 'set', -1);
    this.offset = _attr_as_int(element, 'offset', 0);

    if (this.semantic == 'TEXCOORD' && this.set < 0) {

      this.set = 0;

    }

    return this;

  };

  function Source(id) {

    this.id = id;
    this.type = null;

  };

  Source.prototype.parse = function(element) {

    this.id = element.getAttribute('id');

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      switch (child.nodeName) {

        case 'bool_array':

          this.data = _bools(child.textContent);
          this.type = child.nodeName;
          break;

        case 'float_array':

          this.data = _floats(child.textContent);
          this.type = child.nodeName;
          break;

        case 'int_array':

          this.data = _ints(child.textContent);
          this.type = child.nodeName;
          break;

        case 'IDREF_array':
        case 'Name_array':

          this.data = _strings(child.textContent);
          this.type = child.nodeName;
          break;

        case 'technique_common':

          for ( var j = 0; j < child.childNodes.length; j++) {

            if (child.childNodes[j].nodeName == 'accessor') {

              this.accessor = (new Accessor()).parse(child.childNodes[j]);
              break;

            }
          }
          break;

        default:
          // console.log(child.nodeName);
          break;

      }

    }

    return this;

  };

  Source.prototype.read = function() {

    var result = [];

    // for (var i = 0; i < this.accessor.params.length; i++) {

    var param = this.accessor.params[0];

    // console.log(param.name + " " + param.type);

    switch (param.type) {

      case 'IDREF':
      case 'Name':
      case 'name':
      case 'float':

        return this.data;

      case 'float4x4':

        for ( var j = 0; j < this.data.length; j += 16) {

          var s = this.data.slice(j, j + 16);
          var m = getConvertedMat4(s);
          result.push(m);
        }

        break;

      default:

        console.log('ColladaLoader: Source: Read dont know how to read ' + param.type + '.');
        break;

    }

    // }

    return result;

  };

  function Material() {

    this.id = "";
    this.name = "";
    this.instance_effect = null;

  };

  Material.prototype.parse = function(element) {

    this.id = element.getAttribute('id');
    this.name = element.getAttribute('name');

    for ( var i = 0; i < element.childNodes.length; i++) {

      if (element.childNodes[i].nodeName == 'instance_effect') {

        this.instance_effect = (new InstanceEffect()).parse(element.childNodes[i]);
        break;

      }

    }

    return this;

  };

  function ColorOrTexture() {

    this.color = new THREE.Color(0);
    this.color.setRGB(Math.random(), Math.random(), Math.random());
    this.color.a = 1.0;

    this.texture = null;
    this.texcoord = null;
    this.texOpts = null;

  };

  ColorOrTexture.prototype.isColor = function() {

    return (this.texture == null);

  };

  ColorOrTexture.prototype.isTexture = function() {

    return (this.texture != null);

  };

  ColorOrTexture.prototype.parse = function(element) {

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'color':

          var rgba = _floats(child.textContent);
          this.color = new THREE.Color(0);
          this.color.setRGB(rgba[0], rgba[1], rgba[2]);
          this.color.a = rgba[3];
          break;

        case 'texture':

          this.texture = child.getAttribute('texture');
          this.texcoord = child.getAttribute('texcoord');
          // Defaults from:
          // https://collada.org/mediawiki/index.php/Maya_texture_placement_MAYA_extension
          this.texOpts = {
            offsetU : 0,
            offsetV : 0,
            repeatU : 1,
            repeatV : 1,
            wrapU : 1,
            wrapV : 1
          };
          this.parseTexture(child);
          break;

        default:
          break;

      }

    }

    return this;

  };

  ColorOrTexture.prototype.parseTexture = function(element) {

    if (!element.childNodes)
      return this;

    // This should be supported by Maya, 3dsMax, and MotionBuilder

    if (element.childNodes[1] && element.childNodes[1].nodeName === 'extra') {

      element = element.childNodes[1];

      if (element.childNodes[1] && element.childNodes[1].nodeName === 'technique') {

        element = element.childNodes[1];

      }

    }

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      switch (child.nodeName) {

        case 'offsetU':
        case 'offsetV':
        case 'repeatU':
        case 'repeatV':

          this.texOpts[child.nodeName] = parseFloat(child.textContent);
          break;

        case 'wrapU':
        case 'wrapV':

          this.texOpts[child.nodeName] = parseInt(child.textContent);
          break;

        default:
          this.texOpts[child.nodeName] = child.textContent;
          break;

      }

    }

    return this;

  };

  function Shader(type, effect) {

    this.type = type;
    this.effect = effect;
    this.material = null;

  };

  Shader.prototype.parse = function(element) {

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'ambient':
        case 'emission':
        case 'diffuse':
        case 'specular':
        case 'transparent':

          this[child.nodeName] = (new ColorOrTexture()).parse(child);
          break;

        case 'shininess':
        case 'reflectivity':
        case 'index_of_refraction':
        case 'transparency':

          var f = evaluateXPath(child, './/dae:float');

          if (f.length > 0)
            this[child.nodeName] = parseFloat(f[0].textContent);

          break;

        default:
          break;

      }

    }

    this.create();
    return this;

  };

  Shader.prototype.create = function() {

    var props = {};
    var transparent = (this['transparency'] !== undefined && this['transparency'] < 1.0);

    for ( var prop in this) {

      switch (prop) {

        case 'ambient':
        case 'emission':
        case 'diffuse':
        case 'specular':

          var cot = this[prop];

          if (cot instanceof ColorOrTexture) {

            if (cot.isTexture()) {

              var samplerId = cot.texture;
              var surfaceId = this.effect.sampler[samplerId].source;

              if (surfaceId) {

                var surface = this.effect.surface[surfaceId];
                var image = images[surface.init_from];

                if (image) {

                  var texture = THREE.ImageUtils.loadTexture(baseUrl + image.init_from);
                  texture.wrapS = cot.texOpts.wrapU ? THREE.RepeatWrapping
                      : THREE.ClampToEdgeWrapping;
                  texture.wrapT = cot.texOpts.wrapV ? THREE.RepeatWrapping
                      : THREE.ClampToEdgeWrapping;
                  texture.offset.x = cot.texOpts.offsetU;
                  texture.offset.y = cot.texOpts.offsetV;
                  texture.repeat.x = cot.texOpts.repeatU;
                  texture.repeat.y = cot.texOpts.repeatV;
                  props['map'] = texture;

                  // Texture with baked lighting?
                  if (prop === 'emission')
                    props['emissive'] = 0xffffff;

                }

              }

            } else if (prop === 'diffuse' || !transparent) {

              if (prop === 'emission') {

                props['emissive'] = cot.color.getHex();

              } else {

                props[prop] = cot.color.getHex();

              }

            }

          }

          break;

        case 'shininess':

          props[prop] = this[prop];
          break;

        case 'reflectivity':

          props[prop] = this[prop];
          if (props[prop] > 0.0)
            props['envMap'] = options.defaultEnvMap;
          props['combine'] = THREE.MixOperation; // mix regular shading with reflective component
          break;

        case 'index_of_refraction':
          // "index_of_refraction" becomes "refractionRatio" in shader
          props['refractionRatio'] = this[prop];
          if (this[prop] !== 1.0)
            props['envMap'] = options.defaultEnvMap;
          break;

        case 'transparency':

          if (transparent) {

            props['transparent'] = true;
            props['opacity'] = this[prop];
            transparent = true;

          }

          break;

        default:
          break;

      }

    }

    props['shading'] = preferredShading;
    props['side'] = this.effect.doubleSided ? THREE.DoubleSide : THREE.FrontSide;

    switch (this.type) {

      case 'constant':

        if (props.emissive != undefined)
          props.color = props.emissive;
        this.material = new THREE.MeshBasicMaterial(props);
        break;

      case 'phong':
      case 'blinn':

        if (props.diffuse != undefined)
          props.color = props.diffuse;
        this.material = new THREE.MeshPhongMaterial(props);
        break;

      case 'lambert':
      default:

        if (props.diffuse != undefined)
          props.color = props.diffuse;
        this.material = new THREE.MeshLambertMaterial(props);
        break;

    }

    return this.material;

  };

  function Surface(effect) {

    this.effect = effect;
    this.init_from = null;
    this.format = null;

  };

  Surface.prototype.parse = function(element) {

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'init_from':

          this.init_from = child.textContent;
          break;

        case 'format':

          this.format = child.textContent;
          break;

        default:

          console.log("unhandled Surface prop: " + child.nodeName);
          break;

      }

    }

    return this;

  };

  function Sampler2D(effect) {

    this.effect = effect;
    this.source = null;
    this.wrap_s = null;
    this.wrap_t = null;
    this.minfilter = null;
    this.magfilter = null;
    this.mipfilter = null;

  };

  Sampler2D.prototype.parse = function(element) {

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'source':

          this.source = child.textContent;
          break;

        case 'minfilter':

          this.minfilter = child.textContent;
          break;

        case 'magfilter':

          this.magfilter = child.textContent;
          break;

        case 'mipfilter':

          this.mipfilter = child.textContent;
          break;

        case 'wrap_s':

          this.wrap_s = child.textContent;
          break;

        case 'wrap_t':

          this.wrap_t = child.textContent;
          break;

        default:

          console.log("unhandled Sampler2D prop: " + child.nodeName);
          break;

      }

    }

    return this;

  };

  function Effect() {

    this.id = "";
    this.name = "";
    this.shader = null;
    this.surface = {};
    this.sampler = {};

  };

  Effect.prototype.create = function() {

    if (this.shader == null) {

      return null;

    }

  };

  Effect.prototype.parse = function(element) {

    this.id = element.getAttribute('id');
    this.name = element.getAttribute('name');

    extractDoubleSided(this, element);

    this.shader = null;

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'profile_COMMON':

          this.parseTechnique(this.parseProfileCOMMON(child));
          break;

        default:
          break;

      }

    }

    return this;

  };

  Effect.prototype.parseNewparam = function(element) {

    var sid = element.getAttribute('sid');

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'surface':

          this.surface[sid] = (new Surface(this)).parse(child);
          break;

        case 'sampler2D':

          this.sampler[sid] = (new Sampler2D(this)).parse(child);
          break;

        case 'extra':

          break;

        default:

          console.log(child.nodeName);
          break;

      }

    }

  };

  Effect.prototype.parseProfileCOMMON = function(element) {

    var technique;

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'profile_COMMON':

          this.parseProfileCOMMON(child);
          break;

        case 'technique':

          technique = child;
          break;

        case 'newparam':

          this.parseNewparam(child);
          break;

        case 'image':

          var _image = (new _Image()).parse(child);
          images[_image.id] = _image;
          break;

        case 'extra':
          break;

        default:

          console.log(child.nodeName);
          break;

      }

    }

    return technique;

  };

  Effect.prototype.parseTechnique = function(element) {

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'constant':
        case 'lambert':
        case 'blinn':
        case 'phong':

          this.shader = (new Shader(child.nodeName, this)).parse(child);
          break;

        default:
          break;

      }

    }

  };

  function InstanceEffect() {

    this.url = "";

  };

  InstanceEffect.prototype.parse = function(element) {

    this.url = element.getAttribute('url').replace(/^#/, '');
    return this;

  };

  function Animation() {

    this.id = "";
    this.name = "";
    this.source = {};
    this.sampler = [];
    this.channel = [];

  };

  Animation.prototype.parse = function(element) {

    this.id = element.getAttribute('id');
    this.name = element.getAttribute('name');
    this.source = {};

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];

      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'animation':

          var anim = (new Animation()).parse(child);

          for ( var src in anim.source) {

            this.source[src] = anim.source[src];

          }

          for ( var j = 0; j < anim.channel.length; j++) {

            this.channel.push(anim.channel[j]);
            this.sampler.push(anim.sampler[j]);

          }

          break;

        case 'source':

          var src = (new Source()).parse(child);
          this.source[src.id] = src;
          break;

        case 'sampler':

          this.sampler.push((new Sampler(this)).parse(child));
          break;

        case 'channel':

          this.channel.push((new Channel(this)).parse(child));
          break;

        default:
          break;

      }

    }

    return this;

  };

  function Channel(animation) {

    this.animation = animation;
    this.source = "";
    this.target = "";
    this.fullSid = null;
    this.sid = null;
    this.dotSyntax = null;
    this.arrSyntax = null;
    this.arrIndices = null;
    this.member = null;

  };

  Channel.prototype.parse = function(element) {

    this.source = element.getAttribute('source').replace(/^#/, '');
    this.target = element.getAttribute('target');

    var parts = this.target.split('/');

    var id = parts.shift();
    var sid = parts.shift();

    var dotSyntax = (sid.indexOf(".") >= 0);
    var arrSyntax = (sid.indexOf("(") >= 0);

    if (dotSyntax) {

      parts = sid.split(".");
      this.sid = parts.shift();
      this.member = parts.shift();

    } else if (arrSyntax) {

      var arrIndices = sid.split("(");
      this.sid = arrIndices.shift();

      for ( var j = 0; j < arrIndices.length; j++) {

        arrIndices[j] = parseInt(arrIndices[j].replace(/\)/, ''));

      }

      this.arrIndices = arrIndices;

    } else {

      this.sid = sid;

    }

    this.fullSid = sid;
    this.dotSyntax = dotSyntax;
    this.arrSyntax = arrSyntax;

    return this;

  };

  function Sampler(animation) {

    this.id = "";
    this.animation = animation;
    this.inputs = [];
    this.input = null;
    this.output = null;
    this.strideOut = null;
    this.interpolation = null;
    this.startTime = null;
    this.endTime = null;
    this.duration = 0;

  };

  Sampler.prototype.parse = function(element) {

    this.id = element.getAttribute('id');
    this.inputs = [];

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'input':

          this.inputs.push((new Input()).parse(child));
          break;

        default:
          break;

      }

    }

    return this;

  };

  Sampler.prototype.create = function() {

    for ( var i = 0; i < this.inputs.length; i++) {

      var input = this.inputs[i];
      var source = this.animation.source[input.source];

      switch (input.semantic) {

        case 'INPUT':

          this.input = source.read();
          break;

        case 'OUTPUT':

          this.output = source.read();
          this.strideOut = source.accessor.stride;
          break;

        case 'INTERPOLATION':

          this.interpolation = source.read();
          break;

        case 'IN_TANGENT':

          break;

        case 'OUT_TANGENT':

          break;

        default:

          console.log(input.semantic);
          break;

      }

    }

    this.startTime = 0;
    this.endTime = 0;
    this.duration = 0;

    if (this.input.length) {

      this.startTime = 100000000;
      this.endTime = -100000000;

      for ( var i = 0; i < this.input.length; i++) {

        this.startTime = Math.min(this.startTime, this.input[i]);
        this.endTime = Math.max(this.endTime, this.input[i]);

      }

      this.duration = this.endTime - this.startTime;

    }

  };

  Sampler.prototype.getData = function(type, ndx) {

    var data;

    if (type === 'matrix' && this.strideOut === 16) {

      data = this.output[ndx];

    } else if (this.strideOut > 1) {

      data = [];
      ndx *= this.strideOut;

      for ( var i = 0; i < this.strideOut; ++i) {

        data[i] = this.output[ndx + i];

      }

      if (this.strideOut === 3) {

        switch (type) {

          case 'rotate':
          case 'translate':

            fixCoords(data, -1);
            break;

          case 'scale':

            fixCoords(data, 1);
            break;

        }

      } else if (this.strideOut === 4 && type === 'matrix') {

        fixCoords(data, -1);

      }

    } else {

      data = this.output[ndx];

    }

    return data;

  };

  function Key(time) {

    this.targets = [];
    this.time = time;

  };

  Key.prototype.addTarget = function(fullSid, transform, member, data) {

    this.targets.push({
      sid : fullSid,
      member : member,
      transform : transform,
      data : data
    });

  };

  Key.prototype.apply = function(opt_sid) {

    for ( var i = 0; i < this.targets.length; ++i) {

      var target = this.targets[i];

      if (!opt_sid || target.sid === opt_sid) {

        target.transform.update(target.data, target.member);

      }

    }

  };

  Key.prototype.getTarget = function(fullSid) {

    for ( var i = 0; i < this.targets.length; ++i) {

      if (this.targets[i].sid === fullSid) {

        return this.targets[i];

      }

    }

    return null;

  };

  Key.prototype.hasTarget = function(fullSid) {

    for ( var i = 0; i < this.targets.length; ++i) {

      if (this.targets[i].sid === fullSid) {

        return true;

      }

    }

    return false;

  };

  // currently only doing linear interpolation - should support full COLLADA spec.
  Key.prototype.interpolate = function(nextKey, time) {

    for ( var i = 0; i < this.targets.length; ++i) {

      var target = this.targets[i], nextTarget = nextKey.getTarget(target.sid), data;

      if (target.transform.type !== 'matrix' && nextTarget) {

        var scale = (time - this.time) / (nextKey.time - this.time), nextData = nextTarget.data, prevData = target.data;

        // check scale error

        if (scale < 0 || scale > 1) {

          console.log("Key.interpolate: Warning! Scale out of bounds:" + scale);
          scale = scale < 0 ? 0 : 1;

        }

        if (prevData.length) {

          data = [];

          for ( var j = 0; j < prevData.length; ++j) {

            data[j] = prevData[j] + (nextData[j] - prevData[j]) * scale;

          }

        } else {

          data = prevData + (nextData - prevData) * scale;

        }

      } else {

        data = target.data;

      }

      target.transform.update(data, target.member);

    }

  };

  function Camera() {

    this.id = "";
    this.name = "";
    this.technique = "";

  };

  Camera.prototype.parse = function(element) {

    this.id = element.getAttribute('id');
    this.name = element.getAttribute('name');

    for ( var i = 0; i < element.childNodes.length; i++) {

      var child = element.childNodes[i];
      if (child.nodeType != 1)
        continue;

      switch (child.nodeName) {

        case 'optics':

          this.parseOptics(child);
          break;

        default:
          break;

      }

    }

    return this;

  };

  Camera.prototype.parseOptics = function(element) {

    for ( var i = 0; i < element.childNodes.length; i++) {

      if (element.childNodes[i].nodeName == 'technique_common') {

        var technique = element.childNodes[i];

        for ( var j = 0; j < technique.childNodes.length; j++) {

          this.technique = technique.childNodes[j].nodeName;

          if (this.technique == 'perspective') {

            var perspective = technique.childNodes[j];

            for ( var k = 0; k < perspective.childNodes.length; k++) {

              var param = perspective.childNodes[k];

              switch (param.nodeName) {

                case 'yfov':
                  this.yfov = param.textContent;
                  break;
                case 'xfov':
                  this.xfov = param.textContent;
                  break;
                case 'znear':
                  this.znear = param.textContent;
                  break;
                case 'zfar':
                  this.zfar = param.textContent;
                  break;
                case 'aspect_ratio':
                  this.aspect_ratio = param.textContent;
                  break;

              }

            }

          } else if (this.technique == 'orthographic') {

            var orthographic = technique.childNodes[j];

            for ( var k = 0; k < orthographic.childNodes.length; k++) {

              var param = orthographic.childNodes[k];

              switch (param.nodeName) {

                case 'xmag':
                  this.xmag = param.textContent;
                  break;
                case 'ymag':
                  this.ymag = param.textContent;
                  break;
                case 'znear':
                  this.znear = param.textContent;
                  break;
                case 'zfar':
                  this.zfar = param.textContent;
                  break;
                case 'aspect_ratio':
                  this.aspect_ratio = param.textContent;
                  break;

              }

            }

          }

        }

      }

    }

    return this;

  };

  function InstanceCamera() {

    this.url = "";

  };

  InstanceCamera.prototype.parse = function(element) {

    this.url = element.getAttribute('url').replace(/^#/, '');

    return this;

  };

  function _source(element) {

    var id = element.getAttribute('id');

    if (sources[id] != undefined) {

      return sources[id];

    }

    sources[id] = (new Source(id)).parse(element);
    return sources[id];

  };

  function _nsResolver(nsPrefix) {

    if (nsPrefix == "dae") {

      return "http://www.collada.org/2005/11/COLLADASchema";

    }

    return null;

  };

  function _bools(str) {

    var raw = _strings(str);
    var data = [];

    for ( var i = 0, l = raw.length; i < l; i++) {

      data.push((raw[i] == 'true' || raw[i] == '1') ? true : false);

    }

    return data;

  };

  function _floats(str) {

    var raw = _strings(str);
    var data = [];

    for ( var i = 0, l = raw.length; i < l; i++) {

      data.push(parseFloat(raw[i]));

    }

    return data;

  };

  function _ints(str) {

    var raw = _strings(str);
    var data = [];

    for ( var i = 0, l = raw.length; i < l; i++) {

      data.push(parseInt(raw[i], 10));

    }

    return data;

  };

  function _strings(str) {

    return (str.length > 0) ? _trimString(str).split(/\s+/) : [];

  };

  function _trimString(str) {

    return str.replace(/^\s+/, "").replace(/\s+$/, "");

  };

  function _attr_as_float(element, name, defaultValue) {

    if (element.hasAttribute(name)) {

      return parseFloat(element.getAttribute(name));

    } else {

      return defaultValue;

    }

  };

  function _attr_as_int(element, name, defaultValue) {

    if (element.hasAttribute(name)) {

      return parseInt(element.getAttribute(name), 10);

    } else {

      return defaultValue;

    }

  };

  function _attr_as_string(element, name, defaultValue) {

    if (element.hasAttribute(name)) {

      return element.getAttribute(name);

    } else {

      return defaultValue;

    }

  };

  function _format_float(f, num) {

    if (f === undefined) {

      var s = '0.';

      while (s.length < num + 2) {

        s += '0';

      }

      return s;

    }

    num = num || 2;

    var parts = f.toString().split('.');
    parts[1] = parts.length > 1 ? parts[1].substr(0, num) : "0";

    while (parts[1].length < num) {

      parts[1] += '0';

    }

    return parts.join('.');

  };

  function evaluateXPath(node, query) {

    var instances = COLLADA.evaluate(query, node, _nsResolver,
        XPathResult.ORDERED_NODE_ITERATOR_TYPE, null);

    var inst = instances.iterateNext();
    var result = [];

    while (inst) {

      result.push(inst);
      inst = instances.iterateNext();

    }

    return result;

  };

  function extractDoubleSided(obj, element) {

    obj.doubleSided = false;

    var node = COLLADA.evaluate('.//dae:extra//dae:double_sided', element, _nsResolver,
        XPathResult.ORDERED_NODE_ITERATOR_TYPE, null);

    if (node) {

      node = node.iterateNext();

      if (node && parseInt(node.textContent, 10) === 1) {

        obj.doubleSided = true;

      }

    }

  };

  // Up axis conversion

  function setUpConversion() {

    if (!options.convertUpAxis || colladaUp === options.upAxis) {

      upConversion = null;

    } else {

      switch (colladaUp) {

        case 'X':

          upConversion = options.upAxis === 'Y' ? 'XtoY' : 'XtoZ';
          break;

        case 'Y':

          upConversion = options.upAxis === 'X' ? 'YtoX' : 'YtoZ';
          break;

        case 'Z':

          upConversion = options.upAxis === 'X' ? 'ZtoX' : 'ZtoY';
          break;

      }

    }

  };

  function fixCoords(data, sign) {

    if (!options.convertUpAxis || colladaUp === options.upAxis) {

      return;

    }

    switch (upConversion) {

      case 'XtoY':

        var tmp = data[0];
        data[0] = sign * data[1];
        data[1] = tmp;
        break;

      case 'XtoZ':

        var tmp = data[2];
        data[2] = data[1];
        data[1] = data[0];
        data[0] = tmp;
        break;

      case 'YtoX':

        var tmp = data[0];
        data[0] = data[1];
        data[1] = sign * tmp;
        break;

      case 'YtoZ':

        var tmp = data[1];
        data[1] = sign * data[2];
        data[2] = tmp;
        break;

      case 'ZtoX':

        var tmp = data[0];
        data[0] = data[1];
        data[1] = data[2];
        data[2] = tmp;
        break;

      case 'ZtoY':

        var tmp = data[1];
        data[1] = data[2];
        data[2] = sign * tmp;
        break;

    }

  };

  function getConvertedVec3(data, offset) {

    var arr = [ data[offset], data[offset + 1], data[offset + 2] ];
    fixCoords(arr, -1);
    return new THREE.Vector3(arr[0], arr[1], arr[2]);

  };

  function getConvertedMat4(data) {

    if (options.convertUpAxis) {

      // First fix rotation and scale

      // Columns first
      var arr = [ data[0], data[4], data[8] ];
      fixCoords(arr, -1);
      data[0] = arr[0];
      data[4] = arr[1];
      data[8] = arr[2];
      arr = [ data[1], data[5], data[9] ];
      fixCoords(arr, -1);
      data[1] = arr[0];
      data[5] = arr[1];
      data[9] = arr[2];
      arr = [ data[2], data[6], data[10] ];
      fixCoords(arr, -1);
      data[2] = arr[0];
      data[6] = arr[1];
      data[10] = arr[2];
      // Rows second
      arr = [ data[0], data[1], data[2] ];
      fixCoords(arr, -1);
      data[0] = arr[0];
      data[1] = arr[1];
      data[2] = arr[2];
      arr = [ data[4], data[5], data[6] ];
      fixCoords(arr, -1);
      data[4] = arr[0];
      data[5] = arr[1];
      data[6] = arr[2];
      arr = [ data[8], data[9], data[10] ];
      fixCoords(arr, -1);
      data[8] = arr[0];
      data[9] = arr[1];
      data[10] = arr[2];

      // Now fix translation
      arr = [ data[3], data[7], data[11] ];
      fixCoords(arr, -1);
      data[3] = arr[0];
      data[7] = arr[1];
      data[11] = arr[2];

    }

    return new THREE.Matrix4(data[0], data[1], data[2], data[3], data[4], data[5], data[6],
        data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

  };

  function getConvertedIndex(index) {

    if (index > -1 && index < 3) {

      var members = [ 'X', 'Y', 'Z' ], indices = {
        X : 0,
        Y : 1,
        Z : 2
      };

      index = getConvertedMember(members[index]);
      index = indices[index];

    }

    return index;

  };

  function getConvertedMember(member) {

    if (options.convertUpAxis) {

      switch (member) {

        case 'X':

          switch (upConversion) {

            case 'XtoY':
            case 'XtoZ':
            case 'YtoX':

              member = 'Y';
              break;

            case 'ZtoX':

              member = 'Z';
              break;

          }

          break;

        case 'Y':

          switch (upConversion) {

            case 'XtoY':
            case 'YtoX':
            case 'ZtoX':

              member = 'X';
              break;

            case 'XtoZ':
            case 'YtoZ':
            case 'ZtoY':

              member = 'Z';
              break;

          }

          break;

        case 'Z':

          switch (upConversion) {

            case 'XtoZ':

              member = 'X';
              break;

            case 'YtoZ':
            case 'ZtoX':
            case 'ZtoY':

              member = 'Y';
              break;

          }

          break;

      }

    }

    return member;

  };

  return {

    load : load,
    parse : parse,
    setPreferredShading : setPreferredShading,
    applySkin : applySkin,
    geometries : geometries,
    options : options

  };

};
