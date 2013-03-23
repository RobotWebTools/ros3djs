var ImProxy = {};

ROS3D.InteractiveMarkerClient = function(options) {
  var that = this;
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.topic = options.topic;
  this.root = new THREE.Object3D();
  this.interactiveMarkers = {};
  this.updateTopic = null;
  this.feedbackTopic = null;
  this.camera = options.camera;
  this.meshBaseUrl = '';

  // check for an initial topic
  if (this.topic) {
    this.subscribe(this.topic);
  }

  this.on('created_marker', function(marker) {
    var intMarker = new InteractiveMarker(marker, that.camera, that.meshBaseUrl);
    that.root.add(intMarker);

    marker.on('server_updated_pose', function(pose) {
      intMarker.onServerSetPose({
        pose : pose
      });
    });

    intMarker.addEventListener('user_changed_pose', marker.setPoseFromClient.bind(marker));
    intMarker.addEventListener('user_mouse_down', marker.onMouseDown.bind(marker));
    intMarker.addEventListener('user_mouse_up', marker.onMouseUp.bind(marker));
    intMarker.addEventListener('user_button_click', marker.onButtonClick.bind(marker));
    intMarker.addEventListener('menu_select', marker.onMenuSelect.bind(marker));
  });
};
ROS3D.InteractiveMarkerClient.prototype.__proto__ = EventEmitter2.prototype;

ROS3D.InteractiveMarkerClient.prototype.subscribe = function(topic) {
  // unsubscribe to the other topics
  this.unsubscribe();

  this.updateTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : topic + '/tunneled/update',
    messageType : 'visualization_msgs/InteractiveMarkerUpdate'
  });
  this.updateTopic.subscribe(this.processUpdate.bind(this));

  this.feedbackTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : topic + '/feedback',
    messageType : 'visualization_msgs/InteractiveMarkerFeedback'
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

ROS3D.InteractiveMarkerClient.prototype.eraseIntMarker = function(intMarkerName) {
  if (this.interactiveMarkers[intMarkerName]) {
    this.emit('deleted_marker', intMarkerName);
    delete this.interactiveMarkers[intMarkerName];
  }
};

ROS3D.InteractiveMarkerClient.prototype.processInit = function(initMessage) {
  var message = initMessage.msg;
  var client = this;
  message.erases = [];
  for (intMarkerName in this.interactiveMarkers) {
    message.erases.push(intMarkerName);
  };
  message.poses = [];
  this.processUpdate(message);
};

ROS3D.InteractiveMarkerClient.prototype.processUpdate = function(message) {
  var that = this;

  // Deletes markers
  message.erases.forEach(function(name) {
    var marker = that.interactiveMarkers[name];
    that.eraseIntMarker(name);
  });

  // Updates marker poses
  message.poses.forEach(function(poseMessage) {
    var marker = that.interactiveMarkers[poseMessage.name];
    if (marker) {
      marker.setPoseFromServer(poseMessage.pose);
    }
  });

  // Adds new markers
  message.markers.forEach(function(imMsg) {
    var oldIntMarkerHandle = that.interactiveMarkers[imMsg.name];
    if (oldIntMarkerHandle) {
      that.eraseIntMarker(oldIntMarkerHandle.name);
    }

    var intMarkerHandle = new ImProxy.IntMarkerHandle(imMsg, that.feedbackTopic, that.tfClient);
    that.interactiveMarkers[imMsg.name] = intMarkerHandle;
    that.emit('created_marker', intMarkerHandle);
    // this might trigger a transform update event immediately,
    // so we need to call it after emitting a created_marker event.
    intMarkerHandle.subscribeTf(imMsg);
  });
};

/* Handle with signals for a single interactive marker */

var IntMarkerHandle = ImProxy.IntMarkerHandle = function(imMsg, feedbackTopic, tfClient) {
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
IntMarkerHandle.prototype.__proto__ = EventEmitter2.prototype;

var KEEP_ALIVE = 0;
var POSE_UPDATE = 1;
var MENU_SELECT = 2;
var BUTTON_CLICK = 3;
var MOUSE_DOWN = 4;
var MOUSE_UP = 5;

IntMarkerHandle.prototype.subscribeTf = function(imMsg) {
  // subscribe to tf updates if frame-fixed
  if (imMsg.header.stamp.secs === 0.0 && imMsg.header.stamp.nsecs === 0.0) {
    this.tfClient.subscribe(imMsg.header.frame_id, this.tfUpdate.bind(this));
  }
};

IntMarkerHandle.prototype.emitServerPoseUpdate = function() {
  var poseTransformed = new ROSLIB.Pose({
    position : this.pose.position,
    orientation : this.pose.orientation
  });
  poseTransformed.applyTransform(this.tfTransform);
  this.emit('server_updated_pose', poseTransformed);
};

IntMarkerHandle.prototype.setPoseFromServer = function(poseMsg) {
  this.pose = new ROSLIB.Pose(poseMsg);
  this.emitServerPoseUpdate();
};

IntMarkerHandle.prototype.tfUpdate = function(transformMsg) {
  this.tfTransform = new ROSLIB.Transform(transformMsg);
  this.emitServerPoseUpdate();
};

IntMarkerHandle.prototype.setPoseFromClient = function(event) {
  this.pose = new ROSLIB.Pose(event);
  this.pose.applyInverseTransform(this.tfTransform);
  this.emit('client_updated_pose', event);
  this.sendFeedback(POSE_UPDATE, undefined, 0, event.controlName);

  // keep sending pose feedback until the mouse goes up
  if (this.dragging) {
    if (this.timeoutHandle) {
      clearTimeout(this.timeoutHandle);
    }
    this.timeoutHandle = setTimeout(this.setPoseFromClient.bind(this, event), 250);
  }
};

IntMarkerHandle.prototype.onButtonClick = function(event) {
  this.sendFeedback(BUTTON_CLICK, event.clickPosition, 0, event.controlName);
};

IntMarkerHandle.prototype.onMouseDown = function(event) {
  this.sendFeedback(MOUSE_DOWN, event.clickPosition, 0, event.controlName);
  this.dragging = true;
};

IntMarkerHandle.prototype.onMouseUp = function(event) {
  this.sendFeedback(MOUSE_UP, event.clickPosition, 0, event.controlName);
  this.dragging = false;
  if (this.timeoutHandle) {
    clearTimeout(this.timeoutHandle);
  }
};

IntMarkerHandle.prototype.onMenuSelect = function(event) {
  this.sendFeedback(MENU_SELECT, undefined, event.id, event.controlName);
};

IntMarkerHandle.prototype.sendFeedback = function(eventType, clickPosition, menu_entry_id,
    controlName) {

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
