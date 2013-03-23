ROS3D.InteractiveMarkerClient = function(options) {
  var that = this;
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.topic = options.topic;
  this.root = new THREE.Object3D();
  options.viewer.selectableObjs.add(this.root);
  this.interactiveMarkers = {};
  this.updateTopic = null;
  this.feedbackTopic = null;
  this.camera = options.viewer.camera;
  this.meshBaseUrl = '';

  // check for an initial topic
  if (this.topic) {
    this.subscribe(this.topic);
  }

  this.on('created_marker', function(marker) {
    var intMarker = new ROS3D.InteractiveMarker(marker, that.camera, that.meshBaseUrl);
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

    var intMarkerHandle = new ROS3D.IntMarkerHandle(imMsg, that.feedbackTopic, that.tfClient);
    that.interactiveMarkers[imMsg.name] = intMarkerHandle;
    that.emit('created_marker', intMarkerHandle);
    // this might trigger a transform update event immediately,
    // so we need to call it after emitting a created_marker event.
    intMarkerHandle.subscribeTf(imMsg);
  });
};
