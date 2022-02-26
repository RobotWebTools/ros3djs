/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A client for an interactive marker topic.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - a handle to the ROS connection
 *  * tfClient - a handle to the TF client
 *  * topic (optional) - the topic to subscribe to, like '/basic_controls', if not provided use subscribe() to start message receiving
 *  * path (optional) - the base path to any meshes that will be loaded
 *  * camera - the main camera associated with the viewer for this marker client
 *  * rootObject (optional) - the root THREE 3D object to render to
 *  * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER)
 *  * menuFontSize (optional) - the menu font size
 */
ROS3D.InteractiveMarkerClient = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.topicName = options.topic;
  this.path = options.path || '/';
  this.camera = options.camera;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.loader = options.loader;
  this.menuFontSize = options.menuFontSize || '0.8em';

  this.interactiveMarkers = {};
  this.updateTopic = null;
  this.feedbackTopic = null;

  // check for an initial topic
  if (this.topicName) {
    this.subscribe(this.topicName);
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
    this.updateTopic.unsubscribe(this.processUpdate);
  }
  if (this.feedbackTopic) {
    this.feedbackTopic.unadvertise();
  }
  // erase all markers
  for (var intMarkerName in this.interactiveMarkers) {
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
  for (var intMarkerName in this.interactiveMarkers) {
    message.erases.push(intMarkerName);
  }
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
      tfClient : that.tfClient,
      menuFontSize : that.menuFontSize
    });
    that.interactiveMarkers[msg.name] = handle;

    // create the actual marker
    var intMarker = new ROS3D.InteractiveMarker({
      handle : handle,
      camera : that.camera,
      path : that.path,
      loader : that.loader
    });
    // add it to the scene
    intMarker.name = msg.name;
    that.rootObject.add(intMarker);

    // listen for any pose updates from the server
    handle.on('pose', function(pose) {
      intMarker.onServerSetPose({
        pose : pose
      });
    });

    // add bound versions of UI handlers
    intMarker.addEventListener('user-pose-change', handle.setPoseFromClientBound);
    intMarker.addEventListener('user-mousedown', handle.onMouseDownBound);
    intMarker.addEventListener('user-mouseup', handle.onMouseUpBound);
    intMarker.addEventListener('user-button-click', handle.onButtonClickBound);
    intMarker.addEventListener('menu-select', handle.onMenuSelectBound);

    // now listen for any TF changes
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
    // remove the object
    var targetIntMarker = this.rootObject.getObjectByName(intMarkerName);
    this.rootObject.remove(targetIntMarker);
    // unsubscribe from TF topic!
    var handle = this.interactiveMarkers[intMarkerName];
    handle.unsubscribeTf();

    // remove all other listeners

    targetIntMarker.removeEventListener('user-pose-change', handle.setPoseFromClientBound);
    targetIntMarker.removeEventListener('user-mousedown', handle.onMouseDownBound);
    targetIntMarker.removeEventListener('user-mouseup', handle.onMouseUpBound);
    targetIntMarker.removeEventListener('user-button-click', handle.onButtonClickBound);
    targetIntMarker.removeEventListener('menu-select', handle.onMenuSelectBound);

    // remove the handle from the map - after leaving this function's scope, there should be no references to the handle
    delete this.interactiveMarkers[intMarkerName];
    targetIntMarker.dispose();
  }
};
