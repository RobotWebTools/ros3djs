ROS3D.InteractiveMarker = function(handle, camera, meshBaseUrl) {
  THREE.Object3D.call(this);
  THREE.EventDispatcher.call(this);

  var that = this;

  this.name = handle.name;

  this.dragging = false;
  this.onServerSetPose({
    pose : handle.pose
  });

  this.dragStart = {
    position : new THREE.Vector3(),
    orientation : new THREE.Quaternion(),
    positionWorld : new THREE.Vector3(),
    orientationWorld : new THREE.Quaternion(),
    event3d : {}
  };

  handle.controls.forEach(function(controlMsg) {
    that.add(new ROS3D.InteractiveMarkerControl({
      parent : that,
      controlMessage : controlMsg,
      camera : camera,
      path : meshBaseUrl
    }));
  });

  if (handle.menuEntries.length > 0) {
    this.menu = new ROS3D.InteractiveMarkerMenu({
      menuEntries : handle.menuEntries
    });

    // forward menu select events
    this.menu.addEventListener("menu-select", function(event) {
      that.dispatchEvent(event);
    });
  }
};

ROS3D.InteractiveMarker.prototype = Object.create(THREE.Object3D.prototype);

var projector = new THREE.Projector();

var findClosestPoint = function(target_ray, mouse_ray) {
  // Find the closest point on target_ray to any point on mouse_ray.
  // Math taken from http://paulbourke.net/geometry/lineline3d/
  // line P1->P2 is target_ray
  // line P3->P4 is mouse_ray

  var v13 = new THREE.Vector3;
  v13.subVectors(target_ray.origin, mouse_ray.origin);
  var v43 = mouse_ray.direction.clone();
  var v21 = target_ray.direction.clone();
  var d1343 = v13.dot(v43);
  var d4321 = v43.dot(v21);
  var d1321 = v13.dot(v21);
  var d4343 = v43.dot(v43);
  var d2121 = v21.dot(v21);

  var denom = d2121 * d4343 - d4321 * d4321;
  if (Math.abs(denom) <= 0.0001) {
    return undefined;
  }
  var numer = d1343 * d4321 - d1321 * d4343;

  var mua = numer / denom;
  return mua;
};

var closestAxisPoint = function(axisRay, camera, mousePos) {
  // project axis onto screen
  var o = axisRay.origin.clone();
  projector.projectVector(o, camera);

  var o2 = axisRay.direction.clone().add(axisRay.origin);
  projector.projectVector(o2, camera);

  // d is the axis vector in screen space
  var d = o2.clone().sub(o);
  // d = o2-o;

  // t is the 2d ray param of perpendicular projection
  // of mousePos onto o
  var tmp = new THREE.Vector2;
  var t = tmp.subVectors(mousePos, o).dot(d) / d.dot(d);
  // t = (mousePos - o) * d / (d*d);

  // mp is the final 2d-projected mouse pos DDD
  var mp = new THREE.Vector2;
  mp.addVectors(o, d.clone().multiplyScalar(t));
  // mp = o + d*t;

  // go back to 3d by shooting a ray
  var vector = new THREE.Vector3(mp.x, mp.y, 0.5);
  projector.unprojectVector(vector, camera);
  var mpRay = new THREE.Ray(camera.position, vector.sub(camera.position).normalize());
  var mua = findClosestPoint(axisRay, mpRay, mua);

  return mua;
};

var intersectPlane = function(mouseRay, planeOrigin, planeNormal) {

  var vector = new THREE.Vector3();
  var intersectPoint = new THREE.Vector3();

  vector.subVectors(planeOrigin, mouseRay.origin);
  dot = mouseRay.direction.dot(planeNormal);

  // bail if ray and plane are parallel
  if (Math.abs(dot) < mouseRay.precision)
    return null;

  // calc distance to plane
  scalar = planeNormal.dot(vector) / dot;

  // if negative distance, then plane is behind ray
  // if (scalar < 0)
  // return null;

  intersectPoint.addVectors(mouseRay.origin, mouseRay.direction.clone().multiplyScalar(scalar));
  return intersectPoint;
};

ROS3D.InteractiveMarker.prototype.showMenu = function(control, event) {
  if (this.menu) {
    this.menu.show(control, event);
  }
};

ROS3D.InteractiveMarker.prototype.moveAxis = function(control, origAxis, event3d) {
  if (this.dragging) {
    var currentControlOri = control.currentControlOri;
    var axis = origAxis.clone().applyQuaternion(currentControlOri);
    // get move axis in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var axisWorld = axis.clone().applyQuaternion(this.dragStart.orientationWorld.clone());

    var axisRay = new THREE.Ray(originWorld, axisWorld);

    // find closest point to mouse on axis
    var t = closestAxisPoint(axisRay, event3d.camera, event3d.mousePos);

    // offset from drag start position
    var p = new THREE.Vector3;
    p.addVectors(this.dragStart.position, axis.clone().applyQuaternion(this.dragStart.orientation)
        .multiplyScalar(t));
    this.setPosition(control, p);

    event3d.stopPropagation();
  }
};

ROS3D.InteractiveMarker.prototype.movePlane = function(control, origNormal, event3d) {
  if (this.dragging) {
    var currentControlOri = control.currentControlOri;
    var normal = origNormal.clone().applyQuaternion(currentControlOri);
    // get plane params in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var normalWorld = normal.clone().applyQuaternion(this.dragStart.orientationWorld);

    // intersect mouse ray with plane
    var intersection = intersectPlane(event3d.mouseRay, originWorld, normalWorld);

    // offset from drag start position
    var p = new THREE.Vector3;
    p.subVectors(intersection, originWorld);
    p.add(this.dragStart.positionWorld);
    this.setPosition(control, p);
    event3d.stopPropagation();
  }
};

function printVec(v) {
  // console.log(Math.round(v.x*10)/10,Math.round(v.y*10)/10,Math.round(v.y*10)/10);
}
function printQuat(v) {
  // console.log(Math.round(v.x*10)/10,Math.round(v.y*10)/10,Math.round(v.y*10)/10,Math.round(v.w*10)/10);
}

ROS3D.InteractiveMarker.prototype.rotateAxis = function(control, origOrientation, event3d) {
  if (this.dragging) {
    control.updateMatrixWorld();

    // console.log("------------------_");

    var currentControlOri = control.currentControlOri;
    var orientation = currentControlOri.clone().multiply(origOrientation.clone());

    printQuat(currentControlOri);
    printQuat(orientation);

    var normal = (new THREE.Vector3(1, 0, 0)).applyQuaternion(orientation);

    // get plane params in world coords
    var originWorld = this.dragStart.event3d.intersection.point;
    var normalWorld = normal.applyQuaternion(this.dragStart.orientationWorld);

    printVec(normal);
    printVec(normalWorld);

    // intersect mouse ray with plane
    var intersection = intersectPlane(event3d.mouseRay, originWorld, normalWorld);

    // offset local origin to lie on intersection plane
    var normalRay = new THREE.Ray(this.dragStart.positionWorld, normalWorld);
    var rotOrigin = intersectPlane(normalRay, originWorld, normalWorld);

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
    // this.setOrientation( rot.multiplySelf(this.dragStart.orientationWorld) );
    this.setOrientation(control, rot.multiply(this.dragStart.orientationWorld));

    // offset from drag start position
    event3d.stopPropagation();
  }
};

ROS3D.InteractiveMarker.prototype.feedbackEvent = function(type, control) {
  this.dispatchEvent({
    type : type,
    position : this.position.clone(),
    orientation : this.quaternion.clone(),
    controlName : control.name
  });
};

ROS3D.InteractiveMarker.prototype.startDrag = function(control, event3d) {
  if (event3d.domEvent.button !== 0) {
    return;
  }
  event3d.stopPropagation();
  this.dragging = true;
  this.updateMatrixWorld(true);
  var scale = new THREE.Vector3();
  this.matrixWorld.decompose(this.dragStart.positionWorld, this.dragStart.orientationWorld, scale);
  this.dragStart.position = this.position.clone();
  this.dragStart.orientation = this.quaternion.clone();
  this.dragStart.event3d = event3d;

  this.feedbackEvent("user-mousedown", control);
};

ROS3D.InteractiveMarker.prototype.stopDrag = function(control, event3d) {
  if (event3d.domEvent.button !== 0) {
    return;
  }
  event3d.stopPropagation();
  this.dragging = false;
  this.dragStart.event3d = {};
  this.onServerSetPose(this.bufferedPoseEvent);
  this.bufferedPoseEvent = undefined;

  this.feedbackEvent("user-mouseup", control);
};

ROS3D.InteractiveMarker.prototype.buttonClick = function(control, event3d) {
  event3d.stopPropagation();
  this.feedbackEvent("user-button-click", control);
};

ROS3D.InteractiveMarker.prototype.setPosition = function(control, position) {
  this.position = position;
  this.feedbackEvent("user-pose-change", control);
};

ROS3D.InteractiveMarker.prototype.setOrientation = function(control, orientation) {
  orientation.normalize();
  this.quaternion = orientation;
  this.feedbackEvent("user-pose-change", control);
};

ROS3D.InteractiveMarker.prototype.onServerSetPose = function(event) {
  if (event === undefined) {
    return;
  }

  if (this.dragging) {
    this.bufferedPoseEvent = event;
    return;
  }

  var pose = event.pose;

  this.position.x = pose.position.x;
  this.position.y = pose.position.y;
  this.position.z = pose.position.z;

  this.useQuaternion = true;
  this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);

  this.updateMatrixWorld(true);
};

// --------------------------------------------------------

