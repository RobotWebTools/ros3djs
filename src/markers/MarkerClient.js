/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A marker client that listens to a given marker topic.
 * 
 * Emits the following events:
 *  * 'change' - there was an update or change in the marker
 *  
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic - the marker topic to listen to
 *   * tfClient - the TF client handle to use
 *   * rootObject (optional) - the root object to add this marker to
 */
ROS3D.MarkerClient = function(options) {
  var that = this;
  var options = options || {};
  var ros = options.ros;
  var topic = options.topic;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();

  // current marker that is displayed
  this.currentMarker = null;

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'visualization_msgs/Marker',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {
    var newMarker = new ROS3D.Marker({
      message : message
    });
    // check for an old marker
    if (that.currentMarker) {
      that.rootObject.remove(that.currentMarker);
    }

    that.currentMarker = new ROS3D.SceneNode({
      frameID : message.header.frame_id,
      tfClient : that.tfClient,
      object : newMarker
    });
    that.rootObject.add(that.currentMarker);
    
    that.emit('change');
  });
};
ROS3D.MarkerClient.prototype.__proto__ = EventEmitter2.prototype;
