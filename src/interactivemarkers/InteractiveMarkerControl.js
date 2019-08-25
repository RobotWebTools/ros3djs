/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * The main marker control object for an interactive marker.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * parent - the parent of this control
 *  * message - the interactive marker control message
 *  * camera - the main camera associated with the viewer for this marker client
 *  * path (optional) - the base path to any meshes that will be loaded
 *  * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER)
 */
ROS3D.InteractiveMarkerControl = function(options) {
  THREE.Object3D.call(this);
  var that = this;

  options = options || {};
  this.parent = options.parent;
  var handle = options.handle;
  var message = options.message;
  this.message = message;
  this.name = message.name;
  this.camera = options.camera;
  this.path = options.path || '/';
  this.loader = options.loader;
  this.dragging = false;
  this.startMousePos = new THREE.Vector2();
  this.isShift = false;


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
    case ROS3D.INTERACTIVE_MARKER_MOVE_ROTATE_3D:
    case ROS3D.INTERACTIVE_MARKER_MOVE_3D:
      this.addEventListener('mousemove', this.parent.move3d.bind(this.parent, this, controlAxis));
      break;
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
  if (message.interaction_mode !== ROS3D.INTERACTIVE_MARKER_NONE) {
    this.addEventListener('mousedown', this.parent.startDrag.bind(this.parent, this));
    this.addEventListener('mouseup', this.parent.stopDrag.bind(this.parent, this));
    this.addEventListener('contextmenu', this.parent.showMenu.bind(this.parent, this));
    this.addEventListener('mouseup', function(event3d) {
      if (that.startMousePos.distanceToSquared(event3d.mousePos) === 0) {
        event3d.type = 'contextmenu';
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('mouseover', stopPropagation);
    this.addEventListener('mouseout', stopPropagation);
    this.addEventListener('click', stopPropagation);
    this.addEventListener('mousedown', function(event3d) {
      that.startMousePos = event3d.mousePos;
    });

    // touch support
    this.addEventListener('touchstart', function(event3d) {
      if (event3d.domEvent.touches.length === 1) {
        event3d.type = 'mousedown';
        event3d.domEvent.button = 0;
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('touchmove', function(event3d) {
      if (event3d.domEvent.touches.length === 1) {
        event3d.type = 'mousemove';
        event3d.domEvent.button = 0;
        that.dispatchEvent(event3d);
      }
    });
    this.addEventListener('touchend', function(event3d) {
      if (event3d.domEvent.touches.length === 0) {
        event3d.domEvent.button = 0;
        event3d.type = 'mouseup';
        that.dispatchEvent(event3d);
        event3d.type = 'click';
        that.dispatchEvent(event3d);
      }
    });

    window.addEventListener('keydown', function(event){
      if(event.keyCode === 16){
        that.isShift = true;
      }
    });
    window.addEventListener('keyup', function(event){
      if(event.keyCode === 16){
        that.isShift = false;
      }
    });
  }

  // rotation behavior
  var rotInv = new THREE.Quaternion();
  var posInv = this.parent.position.clone().multiplyScalar(-1);
  switch (message.orientation_mode) {
    case ROS3D.INTERACTIVE_MARKER_INHERIT:
      rotInv = this.parent.quaternion.clone().inverse();
      break;
    case ROS3D.INTERACTIVE_MARKER_FIXED:
      break;
    case ROS3D.INTERACTIVE_MARKER_VIEW_FACING:
      break;
    default:
      console.error('Unkown orientation mode: ' + message.orientation_mode);
      break;
  }

  // temporary TFClient to get transformations from InteractiveMarker
  // frame to potential child Marker frames
  var localTfClient = new ROSLIB.TFClient({
    ros : handle.tfClient.ros,
    fixedFrame : handle.message.header.frame_id,
    serverName : handle.tfClient.serverName
  });

  // create visuals (markers)
  message.markers.forEach(function(markerMsg) {
    var addMarker = function(transformMsg) {
      var markerHelper = new ROS3D.Marker({
        message : markerMsg,
        path : that.path,
        loader : that.loader
      });

      // if transformMsg isn't null, this was called by TFClient
      if (transformMsg !== null) {
        // get the current pose as a ROSLIB.Pose...
        var newPose = new ROSLIB.Pose({
          position : markerHelper.position,
          orientation : markerHelper.quaternion
        });
        // so we can apply the transform provided by the TFClient
        newPose.applyTransform(new ROSLIB.Transform(transformMsg));

        // get transform between parent marker's location and its frame
        // apply it to sub-marker position to get sub-marker position
        // relative to parent marker
        var transformMarker = new ROS3D.Marker({
          message : markerMsg,
          path : that.path,
          loader : that.loader
        });
        transformMarker.position.add(posInv);
        transformMarker.position.applyQuaternion(rotInv);
        transformMarker.quaternion.multiplyQuaternions(rotInv, transformMarker.quaternion);
        var translation = new THREE.Vector3(transformMarker.position.x, transformMarker.position.y, transformMarker.position.z);
        var transform = new ROSLIB.Transform({
          translation : translation,
          orientation : transformMarker.quaternion
        });

        // apply that transform too
        newPose.applyTransform(transform);

        markerHelper.setPose(newPose);

        markerHelper.updateMatrixWorld();
        // we only need to set the pose once - at least, this is what RViz seems to be doing, might change in the future
        localTfClient.unsubscribe(markerMsg.header.frame_id);
      }

      // add the marker
      that.add(markerHelper);
    };

    // If the marker is not relative to the parent marker's position,
    // ask the *local* TFClient for the transformation from the
    // InteractiveMarker frame to the sub-Marker frame
    if (markerMsg.header.frame_id !== '') {
      localTfClient.subscribe(markerMsg.header.frame_id, addMarker);
    }
    // If not, just add the marker without changing its pose
    else {
      addMarker(null);
    }
  });
};
ROS3D.InteractiveMarkerControl.prototype.__proto__ = THREE.Object3D.prototype;

ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld = function (force) {
  var that = this;
  var message = this.message;
  switch (message.orientation_mode) {
    case ROS3D.INTERACTIVE_MARKER_INHERIT:
      ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
      that.currentControlOri.copy(that.quaternion);
      that.currentControlOri.normalize();
      break;
    case ROS3D.INTERACTIVE_MARKER_FIXED:
      that.quaternion.copy(that.parent.quaternion.clone().inverse());
      that.updateMatrix();
      that.matrixWorldNeedsUpdate = true;
      ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
      that.currentControlOri.copy(that.quaternion);
      break;
    case ROS3D.INTERACTIVE_MARKER_VIEW_FACING:
      that.camera.updateMatrixWorld();
      var cameraRot = new THREE.Matrix4().extractRotation(that.camera.matrixWorld);

      var ros2Gl = new THREE.Matrix4();
      var r90 = Math.PI * 0.5;
      var rv = new THREE.Euler(-r90, 0, r90);
      ros2Gl.makeRotationFromEuler(rv);

      var worldToLocal = new THREE.Matrix4();
      worldToLocal.getInverse(that.parent.matrixWorld);

      cameraRot.multiplyMatrices(cameraRot, ros2Gl);
      cameraRot.multiplyMatrices(worldToLocal, cameraRot);

      that.currentControlOri.setFromRotationMatrix(cameraRot);

      // check the orientation
      if (!message.independent_marker_orientation) {
        that.quaternion.copy(that.currentControlOri);
        that.updateMatrix();
        that.matrixWorldNeedsUpdate = true;
      }
      ROS3D.InteractiveMarkerControl.prototype.updateMatrixWorld.call(that, force);
      break;
    default:
      console.error('Unkown orientation mode: ' + message.orientation_mode);
      break;
  }
};
