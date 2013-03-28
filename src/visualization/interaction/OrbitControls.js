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
 */
ROS3D.OrbitControls = function(options) {
  THREE.EventDispatcher.call(this);
  var that = this;
  var options = options || {};
  var scene = options.scene;
  this.camera = options.camera;

  // In ROS, z is pointing upwards
  this.camera.up = new THREE.Vector3(0, 0, 1);

  // API
  this.center = new THREE.Vector3();
  this.userZoom = true;
  this.userZoomSpeed = 1.0;
  this.userRotate = true;
  this.userRotateSpeed = 1.0;
  this.autoRotate = false;
  this.autoRotateSpeed = 2.0;

  // internals
  var EPS = 0.000001;
  var PIXELS_PER_ROUND = 1800;
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
  var phiDelta = 0;
  var thetaDelta = 0;
  var scale = 1;
  var lastPosition = new THREE.Vector3();
  var STATE = {
    NONE : -1,
    ROTATE : 0,
    ZOOM : 1,
    MOVE : 2
  };
  var state = STATE.NONE;

  /**
   * Get the default, auto rotation angle.
   * 
   * @returns the default, auto rotation angle
   */
  function getAutoRotationAngle() {
    return 2 * Math.PI / 60 / 60 * that.autoRotateSpeed;
  };

  /**
   * Get the default, auto zoom scale.
   * 
   * @returns the default, auto zoom scale
   */
  function getZoomScale() {
    return Math.pow(0.95, that.userZoomSpeed);
  };

  /**
   * Intersect the main view plane based on the given information.
   * 
   * @param mouseRay - the ray from the mouse
   * @param planeOrigin - the origin of the plane
   * @param planeNormal - the normal of the plane
   * @returns the intersection
   */
  function intersectViewPlane(mouseRay, planeOrigin, planeNormal) {
    var vector = new THREE.Vector3();
    var intersection = new THREE.Vector3();

    vector.subVectors(planeOrigin, mouseRay.origin);
    dot = mouseRay.direction.dot(planeNormal);

    // bail if ray and plane are parallel
    if (Math.abs(dot) < mouseRay.precision) {
      return null;
    }

    // calc distance to plane
    scalar = planeNormal.dot(vector) / dot;

    intersection = mouseRay.direction.clone().multiplyScalar(scalar);
    return intersection;
  };

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
   * Display the main axes for 1 second.
   */
  this.showAxes = function() {
    that.axes.traverse(function(obj) {
      obj.visible = true;
    });
    if (this.hideTimeout) {
      clearTimeout(this.hideTimeout);
    }
    that.hideTimeout = setTimeout(function() {
      that.axes.traverse(function(obj) {
        obj.visible = false;
      });
      that.hideTimeout = false;
    }, 1000);
  };

  // events
  var changeEvent = {
    type : 'change'
  };

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
        moveStartIntersection = intersectViewPlane(event3D.mouseRay, moveStartCenter,
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

      that.rotateLeft(2 * Math.PI * rotateDelta.x / PIXELS_PER_ROUND * that.userRotateSpeed);
      that.rotateUp(2 * Math.PI * rotateDelta.y / PIXELS_PER_ROUND * that.userRotateSpeed);

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
      var intersection = intersectViewPlane(event3D.mouseRay, that.center, moveStartNormal);

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

  /**
   * Rotate the camera to the left by the given angle.
   * 
   * @param angle (optional) - the angle to rotate by
   */
  this.rotateLeft = function(angle) {
    if (angle === undefined) {
      angle = getAutoRotationAngle();
    }
    thetaDelta -= angle;
  };

  /**
   * Rotate the camera to the right by the given angle.
   * 
   * @param angle (optional) - the angle to rotate by
   */
  this.rotateRight = function(angle) {
    if (angle === undefined) {
      angle = getAutoRotationAngle();
    }
    thetaDelta += angle;
  };

  /**
   * Rotate the camera up by the given angle.
   * 
   * @param angle (optional) - the angle to rotate by
   */
  this.rotateUp = function(angle) {
    if (angle === undefined) {
      angle = getAutoRotationAngle();
    }
    phiDelta -= angle;
  };

  /**
   * Rotate the camera down by the given angle.
   * 
   * @param angle (optional) - the angle to rotate by
   */
  this.rotateDown = function(angle) {
    if (angle === undefined) {
      angle = getAutoRotationAngle();
    }
    phiDelta += angle;
  };

  /**
   * Zoom in by the given scale.
   * 
   * @param zoomScale (optional) - the scale to zoom in by
   */
  this.zoomIn = function(zoomScale) {
    if (zoomScale === undefined) {
      zoomScale = getZoomScale();
    }
    scale /= zoomScale;
  };

  /**
   * Zoom out by the given scale.
   * 
   * @param zoomScale (optional) - the scale to zoom in by
   */
  this.zoomOut = function(zoomScale) {
    if (zoomScale === undefined) {
      zoomScale = getZoomScale();
    }
    scale *= zoomScale;
  };

  /**
   * Update the camera to the current settings.
   */
  this.update = function() {
    // x->y, y->z, z->x
    var position = this.camera.position;
    var offset = position.clone().sub(this.center);

    // angle from z-axis around y-axis
    var theta = Math.atan2(offset.y, offset.x);

    // angle from y-axis
    var phi = Math.atan2(Math.sqrt(offset.y * offset.y + offset.x * offset.x), offset.z);

    if (this.autoRotate) {
      this.rotateLeft(getAutoRotationAngle());
    }

    theta += thetaDelta;
    phi += phiDelta;

    // restrict phi to be betwee EPS and PI-EPS
    phi = Math.max(EPS, Math.min(Math.PI - EPS, phi));

    var radius = offset.length();
    offset.y = radius * Math.sin(phi) * Math.sin(theta);
    offset.z = radius * Math.cos(phi);
    offset.x = radius * Math.sin(phi) * Math.cos(theta);
    offset.multiplyScalar(scale);

    position.copy(this.center).add(offset);

    this.camera.lookAt(this.center);

    radius = offset.length();
    this.axes.position = this.center.clone();
    this.axes.scale.x = this.axes.scale.y = this.axes.scale.z = radius * 0.05;
    this.axes.updateMatrixWorld(true);

    thetaDelta = 0;
    phiDelta = 0;
    scale = 1;

    if (lastPosition.distanceTo(this.camera.position) > 0) {
      this.dispatchEvent(changeEvent);
      lastPosition.copy(this.camera.position);
    }
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
