/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A marker client that listens to a given marker topic.
 *
 *  @constructor
 *  @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic - the marker topic to listen to
 */
ROS3D.MarkerClient = function(options) {
  var that = this;
  var options = options || {};
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.topic = options.topic;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.previousMarker = null;

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : this.topic,
    messageType : 'visualization_msgs/Marker',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {
    var newMarker = new ROS3D.Marker({
      message : message
    });
    // check for an old marker
    if (that.previousMarker) {
      that.rootObject.remove(that.previousMarker);
    }
    var sceneNode = new ROS3D.SceneNode({
      frameID : message.header.frame_id,
      tfClient : that.tfClient,
      model : newMarker
    });

    that.previousMarker = sceneNode;
    that.rootObject.add(sceneNode);
    that.emit('update');
  });
};
ROS3D.MarkerClient.prototype.__proto__ = EventEmitter2.prototype;
