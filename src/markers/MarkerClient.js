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
 *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                         ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.MarkerClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.path = options.path || '/';
  this.loader = options.loader || ROS3D.COLLADA_LOADER_2;

  // Markers that are displayed (Map ns+id--Marker)
  this.markers = {};

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'visualization_msgs/Marker',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {

    var newMarker = new ROS3D.Marker({
      message : message,
      path : that.path,
      loader : that.loader
    });

    // remove old marker from Three.Object3D children buffer
    that.rootObject.remove(that.markers[message.ns + message.id]);

    that.markers[message.ns + message.id] = new ROS3D.SceneNode({
      frameID : message.header.frame_id,
      tfClient : that.tfClient,
      object : newMarker
    });
    that.rootObject.add(that.markers[message.ns + message.id]);

    that.emit('change');
  });
};
ROS3D.MarkerClient.prototype.__proto__ = EventEmitter2.prototype;
