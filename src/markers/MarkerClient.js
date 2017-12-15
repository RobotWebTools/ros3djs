/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A marker client that listens to a given marker topic.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the marker
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic - the marker topic to listen to
 *   * tfClient - the TF client handle to use
 *   * rootObject (optional) - the root object to add this marker to
 *   * path (optional) - the base path to any meshes that will be loaded
 */
ROS3D.MarkerClient = function(options) {
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.path = options.path || '/';

  // Markers that are displayed (Map ns+id--Marker)
  this.markers = {};
  this.rosTopic = undefined;

  this.subscribe();
};
ROS3D.MarkerClient.prototype.__proto__ = EventEmitter2.prototype;

ROS3D.MarkerClient.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe();
  }
};

ROS3D.MarkerClient.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : this.topicName,
    messageType : 'visualization_msgs/Marker',
    compression : 'png'
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.MarkerClient.prototype.processMessage = function(message){
  var newMarker = new ROS3D.Marker({
    message : message,
    path : this.path,
  });

  // remove old marker from Three.Object3D children buffer
  var oldNode = this.markers[message.ns + message.id];
  if (oldNode) {
    oldNode.unsubscribeTf();
    this.rootObject.remove(oldNode);
  }

  this.markers[message.ns + message.id] = new ROS3D.SceneNode({
    frameID : message.header.frame_id,
    tfClient : this.tfClient,
    object : newMarker
  });
  this.rootObject.add(this.markers[message.ns + message.id]);

  this.emit('change');
};
