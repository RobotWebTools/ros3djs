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
  options = options || {};
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
  var pixelsPerRound = 1800;
  var touchMoveThreshold = 10;
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
  var touchStartPosition = new Array(2);
  var touchMoveVector = new Array(2);
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
        moveStartNormal.applyMatrix4(rMat);

        moveStartCenter = that.center.clone();
        moveStartPosition = that.camera.position.clone();
        moveStartIntersection = intersectViewPlane(event3D.mouseRay,
                                                   moveStartCenter,
                                                   moveStartNormal);
	this.showAxes();
        break;
      case 2:
        state = STATE.ZOOM;
        zoomStart.set(event.clientX, event.clientY);
	this.showAxes();
        break;
    }

  }

  /**
   * Handle the mousemove 3D event.
   *
   * @param event3D - the 3D event to handle
   */
  function onMouseMove(event3D) {
    var event = event3D.domEvent;
    if (state === STATE.ROTATE) {

      rotateEnd.set(event.clientX, event.clientY);
      rotateDelta.subVectors(rotateEnd, rotateStart);

      that.rotateLeft(2 * Math.PI * rotateDelta.x / pixelsPerRound * that.userRotateSpeed);
      that.rotateUp(2 * Math.PI * rotateDelta.y / pixelsPerRound * that.userRotateSpeed);

      rotateStart.copy(rotateEnd);

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
  }

  /**
   * Used to track the movement during camera movement.
   *
   * @param mouseRay - the mouse ray to intersect with
   * @param planeOrigin - the origin of the plane
   * @param planeNormal - the normal of the plane
   * @returns the intersection
   */
  function intersectViewPlane(mouseRay, planeOrigin, planeNormal) {

    var vector = new THREE.Vector3();
    var intersection = new THREE.Vector3();

    vector.subVectors(planeOrigin, mouseRay.origin);
    var dot = mouseRay.direction.dot(planeNormal);

    // bail if ray and plane are parallel
    if (Math.abs(dot) < mouseRay.precision) {
      return null;
    }

    // calc distance to plane
    var scalar = planeNormal.dot(vector) / dot;

    intersection = mouseRay.direction.clone().multiplyScalar(scalar);
    return intersection;
  }

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
  }

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
    var delta;
    if (typeof (event.wheelDelta) !== 'undefined') {
      delta = event.wheelDelta;
    } else {
      delta = -event.detail;
    }
    if (delta > 0) {
      that.zoomOut();
    } else {
      that.zoomIn();
    }

    this.showAxes();
  }

  /**
   * Handle the touchdown 3D event.
   *
   * @param event3D - the 3D event to handle
   */
  function onTouchDown(event3D) {
    var event = event3D.domEvent;
    switch (event.touches.length) {
      case 1:
        state = STATE.ROTATE;
        rotateStart.set(event.touches[0].pageX - window.scrollX,
                        event.touches[0].pageY - window.scrollY);
        break;
      case 2:
        state = STATE.NONE;
        /* ready for move */
        moveStartNormal = new THREE.Vector3(0, 0, 1);
        var rMat = new THREE.Matrix4().extractRotation(this.camera.matrix);
        moveStartNormal.applyMatrix4(rMat);
        moveStartCenter = that.center.clone();
        moveStartPosition = that.camera.position.clone();
        moveStartIntersection = intersectViewPlane(event3D.mouseRay,
                                                   moveStartCenter,
                                                   moveStartNormal);
        touchStartPosition[0] = new THREE.Vector2(event.touches[0].pageX,
                                                  event.touches[0].pageY);
        touchStartPosition[1] = new THREE.Vector2(event.touches[1].pageX,
                                                  event.touches[1].pageY);
        touchMoveVector[0] = new THREE.Vector2(0, 0);
        touchMoveVector[1] = new THREE.Vector2(0, 0);
        break;
    }

    this.showAxes();

    event.preventDefault();
  }

  /**
   * Handle the touchmove 3D event.
   *
   * @param event3D - the 3D event to handle
   */
  function onTouchMove(event3D) {
    var event = event3D.domEvent;
    if (state === STATE.ROTATE) {

      rotateEnd.set(event.touches[0].pageX - window.scrollX, event.touches[0].pageY - window.scrollY);
      rotateDelta.subVectors(rotateEnd, rotateStart);

      that.rotateLeft(2 * Math.PI * rotateDelta.x / pixelsPerRound * that.userRotateSpeed);
      that.rotateUp(2 * Math.PI * rotateDelta.y / pixelsPerRound * that.userRotateSpeed);

      rotateStart.copy(rotateEnd);
      this.showAxes();
    } else {
      touchMoveVector[0].set(touchStartPosition[0].x - event.touches[0].pageX,
                             touchStartPosition[0].y - event.touches[0].pageY);
      touchMoveVector[1].set(touchStartPosition[1].x - event.touches[1].pageX,
                             touchStartPosition[1].y - event.touches[1].pageY);
      if (touchMoveVector[0].lengthSq() > touchMoveThreshold &&
          touchMoveVector[1].lengthSq() > touchMoveThreshold) {
        touchStartPosition[0].set(event.touches[0].pageX,
                                  event.touches[0].pageY);
        touchStartPosition[1].set(event.touches[1].pageX,
                                  event.touches[1].pageY);
        if (touchMoveVector[0].dot(touchMoveVector[1]) > 0 &&
            state !== STATE.ZOOM) {
          state = STATE.MOVE;
        } else if (touchMoveVector[0].dot(touchMoveVector[1]) < 0 &&
                   state !== STATE.MOVE) {
          state = STATE.ZOOM;
        }
        if (state === STATE.ZOOM) {
          var tmpVector = new THREE.Vector2();
          tmpVector.subVectors(touchStartPosition[0],
                               touchStartPosition[1]);
          if (touchMoveVector[0].dot(tmpVector) < 0 &&
              touchMoveVector[1].dot(tmpVector) > 0) {
            that.zoomOut();
          } else if (touchMoveVector[0].dot(tmpVector) > 0 &&
                     touchMoveVector[1].dot(tmpVector) < 0) {
            that.zoomIn();
          }
        }
      }
      if (state === STATE.MOVE) {
        var intersection = intersectViewPlane(event3D.mouseRay,
                                              that.center,
                                              moveStartNormal);
        if (!intersection) {
          return;
        }
        var delta = new THREE.Vector3().subVectors(moveStartIntersection.clone(),
                                                   intersection.clone());
        that.center.addVectors(moveStartCenter.clone(), delta.clone());
        that.camera.position.addVectors(moveStartPosition.clone(), delta.clone());
        that.update();
        that.camera.updateMatrixWorld();
      }

      this.showAxes();

      event.preventDefault();
    }
  }

  function onTouchEnd(event3D) {
    var event = event3D.domEvent;
    if (event.touches.length === 1 &&
        state !== STATE.ROTATE) {
      state = STATE.ROTATE;
      rotateStart.set(event.touches[0].pageX - window.scrollX,
                      event.touches[0].pageY - window.scrollY);
    }
  }

  // add event listeners
  this.addEventListener('mousedown', onMouseDown);
  this.addEventListener('mouseup', onMouseUp);
  this.addEventListener('mousemove', onMouseMove);
  this.addEventListener('touchstart', onTouchDown);
  this.addEventListener('touchmove', onTouchMove);
  this.addEventListener('touchend', onTouchEnd);
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
  }, 300);
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

  // restrict phi to be between EPS and PI-EPS
  var eps = 0.000001;
  phi = Math.max(eps, Math.min(Math.PI - eps, phi));

  var radius = offset.length();
  offset.set( 
    radius * Math.sin(phi) * Math.cos(theta),
    radius * Math.sin(phi) * Math.sin(theta),
    radius * Math.cos(phi)
  );
  offset.multiplyScalar(this.scale);

  position.copy(this.center).add(offset);

  this.camera.lookAt(this.center);

  radius = offset.length();
  this.axes.position.copy( this.center );
  this.axes.scale.set( radius * 0.05, radius * 0.05, radius *0.05);
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

THREE.EventDispatcher.prototype.apply( ROS3D.OrbitControls.prototype );
