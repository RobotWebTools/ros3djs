ROS3D.InteractiveMarkerControl = function(parent, controlMsg,
    camera, meshBaseUrl) {
  THREE.Object3D.call(this);
  THREE.EventDispatcher.call(this);

  this.parent = parent;
  this.dragging = false;

  var that = this;

  this.name = controlMsg.name;

  var NONE = 0;
  var MENU = 1;
  var BUTTON = 2;
  var MOVE_AXIS = 3;
  var MOVE_PLANE = 4;
  var ROTATE_AXIS = 5;
  var MOVE_ROTATE = 6;

  var controlOri = new THREE.Quaternion(controlMsg.orientation.x, controlMsg.orientation.y,
      controlMsg.orientation.z, controlMsg.orientation.w);
  controlOri.normalize();

  // transform x axis into local frame
  var controlAxis = new THREE.Vector3(1, 0, 0);
  controlAxis.applyQuaternion(controlOri);

  this.currentControlOri = new THREE.Quaternion();

  // determine mouse interaction
  switch (controlMsg.interaction_mode) {
    case MOVE_AXIS:
      this.addEventListener("mousemove", parent.moveAxis.bind(parent, this, controlAxis));
      this.addEventListener("touchmove", parent.moveAxis.bind(parent, this, controlAxis));
      break;
    case ROTATE_AXIS:
      this.addEventListener("mousemove", parent.rotateAxis.bind(parent, this, controlOri));
      break;
    case MOVE_PLANE:
      this.addEventListener("mousemove", parent.movePlane.bind(parent, this, controlAxis));
      break;
    case BUTTON:
      this.addEventListener("click", parent.buttonClick.bind(parent, this));
      break;
    default:
      break;
  }

  // install default listeners for highlighting / dragging
  function stopPropagation(event) {
    event.stopPropagation();
  }

  if (controlMsg.interaction_mode != NONE) {
    this.addEventListener('mousedown', parent.startDrag.bind(parent, this));
    this.addEventListener('mouseup', parent.stopDrag.bind(parent, this));
    this.addEventListener('contextmenu', parent.showMenu.bind(parent, this));
    this.addEventListener('mouseover', stopPropagation);
    this.addEventListener('mouseout', stopPropagation);
    this.addEventListener('click', stopPropagation);

    // hacky touch support
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

  // define rotation behaviour
  var INHERIT = 0;
  var FIXED = 1;
  var VIEW_FACING = 2;

  var rotInv = new THREE.Quaternion();
  var posInv = parent.position.clone().multiplyScalar(-1);

  switch (controlMsg.orientation_mode) {
    case INHERIT:
      rotInv = parent.quaternion.clone().inverse();
      that.updateMatrixWorld = function(force) {
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
        that.currentControlOri.copy(that.quaternion);
        that.currentControlOri.normalize();
      };
      break;
    case FIXED:
      that.updateMatrixWorld = function(force) {
        that.useQuaternion = true;
        that.quaternion = that.parent.quaternion.clone().inverse();
        that.updateMatrix();
        that.matrixWorldNeedsUpdate = true;
        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
        that.currentControlOri.copy(that.quaternion);
      };
      break;
    case VIEW_FACING:
      var independent_marker_orientation = controlMsg.independent_marker_orientation;
      that.updateMatrixWorld = function(force) {

        camera.updateMatrixWorld();
        var cameraRot = new THREE.Matrix4().extractRotation(camera.matrixWorld);

        var ros2Gl = new THREE.Matrix4();
        var r90 = Math.PI * 0.5;
        var rv = new THREE.Vector3(-r90, 0, r90);
        ros2Gl.setRotationFromEuler(rv);

        var worldToLocal = new THREE.Matrix4();
        worldToLocal.getInverse(that.parent.matrixWorld);

        cameraRot.multiplyMatrices(cameraRot, ros2Gl);
        cameraRot.multiplyMatrices(worldToLocal, cameraRot);

        that.currentControlOri.setFromRotationMatrix(cameraRot);

        if (!independent_marker_orientation) {
          that.useQuaternion = true;
          that.quaternion.copy(that.currentControlOri);
          that.updateMatrix();
          that.matrixWorldNeedsUpdate = true;
        }

        ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
      };
      break;
    default:
      break;
  }

  // create visuals (markers)
  controlMsg.markers.forEach(function(markerMsg) {
    var markerHelper = new ROS3D.Marker({
      message : markerMsg,
      path : meshBaseUrl
    });

    if (markerMsg.header.frame_id !== "") {
      // if the marker lives in its own coordinate frame,
      // convert position into IM's local coordinate frame
      markerHelper.position.add(posInv);
      markerHelper.position.applyQuaternion(rotInv);
      markerHelper.quaternion.multiplyQuaternions(rotInv, markerHelper.quaternion);
      markerHelper.updateMatrixWorld();
    }

    that.add(markerHelper);
  });

};

ROS3D.InteractiveMarkerControl.prototype = Object.create(THREE.Object3D.prototype);