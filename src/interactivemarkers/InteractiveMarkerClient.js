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
 *  * viewer - the main 3D viewer that is being rendered to
 *  * path (optional) - the base path to any meshes that will be loaded
 *  * rootObject (optional) - the root THREE 3D object to render to
 */
ROS3D.InteractiveMarkerClient = function(options) {
  var that = this;
  var options = options || {};
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.topic = options.topic;
  this.viewer = options.viewer;
  this.path = '';
  this.rootObject = options.rootObject || new THREE.Object3D();

  this.interactiveMarkers = {};
  this.updateTopic = null;
  this.feedbackTopic = null;

  // check for an initial topic
  if (this.topic) {
    this.subscribe(this.topic);
  }

  // add the markers
  this.viewer.addSelectableObject(this.rootObject);
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
    var handle = new ROS3D.InteractiveMarkerHandle(msg, that.feedbackTopic, that.tfClient);
    that.interactiveMarkers[msg.name] = handle;

    // create the actual marker
    var intMarker = new ROS3D.InteractiveMarker(handle, that.viewer.camera, that.path);
    // add it to the scene
    that.rootObject.add(intMarker);

    // listen for any pose updates from the server
    handle.on('pose', function(pose) {
      intMarker.onServerSetPose({
        pose : pose
      });
    });

    intMarker.addEventListener('user_changed_pose', handle.setPoseFromClient.bind(handle));
    intMarker.addEventListener('user_mouse_down', handle.onMouseDown.bind(handle));
    intMarker.addEventListener('user_mouse_up', handle.onMouseUp.bind(handle));
    intMarker.addEventListener('user_button_click', handle.onButtonClick.bind(handle));
    intMarker.addEventListener('menu_select', handle.onMenuSelect.bind(handle));

    // now list for any TF changes
    handle.subscribeTf(msg);
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
