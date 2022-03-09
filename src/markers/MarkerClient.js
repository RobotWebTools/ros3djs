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
 *   * lifetime - the lifetime of marker
 */
ROS3D.MarkerClient = function(options) {
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.path = options.path || '/';
  this.lifetime = options.lifetime || 0;

  // Markers that are displayed (Map ns+id--Marker)
  this.markers = {};
  this.rosTopic = undefined;
  this.updatedTime = {};

  this.subscribe();
};
ROS3D.MarkerClient.prototype.__proto__ = EventEmitter2.prototype;

ROS3D.MarkerClient.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.MarkerClient.prototype.checkTime = function(name){
    var curTime = new Date().getTime();
    if (curTime - this.updatedTime[name] > this.lifetime) {
        this.removeMarker(name);
        this.emit('change');
    } else {
        var that = this;
        setTimeout(function() {that.checkTime(name);},
                   100);
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
  // remove old marker from Three.Object3D children buffer
  var key = message.ns + message.id;
  var oldNode = this.markers[key];
  this.updatedTime[key] = new Date().getTime();
  if (oldNode) {
    this.removeMarker(key);

  } else if (this.lifetime) {
    this.checkTime(message.ns + message.id);
  }

  if (message.action === 0) {  // "ADD" or "MODIFY"
    var newMarker = new ROS3D.Marker({
      message : message,
      path : this.path,
    });

    this.markers[key] = new ROS3D.SceneNode({
      frameID : message.header.frame_id,
      tfClient : this.tfClient,
      object : newMarker
    });
    this.rootObject.add(this.markers[key]);
  }

  this.emit('change');
};

ROS3D.MarkerClient.prototype.removeMarker = function(key) {
  var oldNode = this.markers[key];
  if(!oldNode) {
    return;
  }
  oldNode.unsubscribeTf();
  this.rootObject.remove(oldNode);
  oldNode.children.forEach(child => {
    child.dispose();
  });
  delete(this.markers[key]);
};
