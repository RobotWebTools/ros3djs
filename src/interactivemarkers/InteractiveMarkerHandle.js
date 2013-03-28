/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * Handle with signals for a single interactive marker.
 *
 * @constructor
 * @param options - object with following keys:
 *  * ros - a handle to the ROS connection
 *  * tfClient - a handle to the TF client
 *  * topic (optional) - the topic to subscribe to, like '/basic_controls'
 *  * viewer - the main 3D viewer that is being rendered to
 *  * path (optional) - the base path to any meshes that will be loaded
 *  * rootObject (optional) - the root THREE 3D object to render to
 */
ROS3D.InteractiveMarkerHandle = function(imMsg, feedbackTopic, tfClient) {
  this.feedbackTopic = feedbackTopic;
  this.tfClient = tfClient;
  this.name = imMsg.name;
  this.header = imMsg.header;
  this.controls = imMsg.controls;
  this.menuEntries = imMsg.menu_entries;
  this.dragging = false;
  this.timeoutHandle = null;
  this.tfTransform = new ROSLIB.Transform();
  this.pose = new ROSLIB.Pose();
  this.setPoseFromServer(imMsg.pose);
};
ROS3D.InteractiveMarkerHandle.prototype.__proto__ = EventEmitter2.prototype;

ROS3D.InteractiveMarkerHandle.prototype.subscribeTf = function(imMsg) {
  // subscribe to tf updates if frame-fixed
  if (imMsg.header.stamp.secs === 0.0 && imMsg.header.stamp.nsecs === 0.0) {
    this.tfClient.subscribe(imMsg.header.frame_id, this.tfUpdate.bind(this));
  }
};

ROS3D.InteractiveMarkerHandle.prototype.emitServerPoseUpdate = function() {
  var poseTransformed = new ROSLIB.Pose({
    position : this.pose.position,
    orientation : this.pose.orientation
  });
  poseTransformed.applyTransform(this.tfTransform);
  this.emit('pose', poseTransformed);
};

ROS3D.InteractiveMarkerHandle.prototype.setPoseFromServer = function(poseMsg) {
  this.pose = new ROSLIB.Pose(poseMsg);
  this.emitServerPoseUpdate();
};

ROS3D.InteractiveMarkerHandle.prototype.tfUpdate = function(transformMsg) {
  this.tfTransform = new ROSLIB.Transform(transformMsg);
  this.emitServerPoseUpdate();
};

ROS3D.InteractiveMarkerHandle.prototype.setPoseFromClient = function(event) {
  this.pose = new ROSLIB.Pose(event);
  var inv = this.tfTransform.clone();
  inv.rotation.invert();
  this.pose.applyTransform(inv);

  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_POSE_UPDATE, undefined, 0, event.controlName);

  // keep sending pose feedback until the mouse goes up
  if (this.dragging) {
    if (this.timeoutHandle) {
      clearTimeout(this.timeoutHandle);
    }
    this.timeoutHandle = setTimeout(this.setPoseFromClient.bind(this, event), 250);
  }
};

ROS3D.InteractiveMarkerHandle.prototype.onButtonClick = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_BUTTON_CLICK, event.clickPosition, 0,
      event.controlName);
};

ROS3D.InteractiveMarkerHandle.prototype.onMouseDown = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_MOUSE_DOWN, event.clickPosition, 0, event.controlName);
  this.dragging = true;
};

ROS3D.InteractiveMarkerHandle.prototype.onMouseUp = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_MOUSE_UP, event.clickPosition, 0, event.controlName);
  this.dragging = false;
  if (this.timeoutHandle) {
    clearTimeout(this.timeoutHandle);
  }
};

ROS3D.InteractiveMarkerHandle.prototype.onMenuSelect = function(event) {
  this.sendFeedback(ROS3D.INTERACTIVE_MARKER_MENU_SELECT, undefined, event.id, event.controlName);
};

ROS3D.InteractiveMarkerHandle.prototype.sendFeedback = function(eventType, clickPosition,
    menu_entry_id, controlName) {

  var mouse_point_valid = clickPosition !== undefined;
  var clickPosition = clickPosition || {
    x : 0,
    y : 0,
    z : 0
  };

  var feedback = {
    header : this.header,
    client_id : this.client_id,
    marker_name : this.name,
    control_name : controlName,
    event_type : eventType,
    pose : this.pose,
    mouse_point : clickPosition,
    mouse_point_valid : mouse_point_valid,
    menu_entry_id : menu_entry_id
  };
  this.feedbackTopic.publish(feedback);
};
