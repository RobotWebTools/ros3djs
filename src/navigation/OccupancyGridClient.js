/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An occupancy grid client that listens to a given map topic.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the marker
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * tfClient (optional) - the TF client handle to use for a scene node
 *   * rootObject (optional) - the root object to add this marker to
 *   * offsetPose (optional) - offset pose of the grid visualization, e.g. for z-offset (ROSLIB.Pose type)
 *   * color (optional) - color of the visualized grid
 *   * opacity (optional) - opacity of the visualized grid (0.0 == fully transparent, 1.0 == opaque)
 */
ROS3D.OccupancyGridClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/map';
  this.continuous = options.continuous;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.offsetPose = options.offsetPose || new ROSLIB.Pose();
  this.color = options.color || {r:255,g:255,b:255};
  this.opacity = options.opacity || 1.0;

  // current grid that is displayed
  this.currentGrid = null;

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'nav_msgs/OccupancyGrid',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {
    // check for an old map
    if (that.currentGrid) {
      // check if it there is a tf client
      if (that.currentGrid.tfClient) {
        // grid is of type ROS3D.SceneNode
        that.currentGrid.unsubscribeTf();
      }
      that.rootObject.remove(that.currentGrid);
    }

    var newGrid = new ROS3D.OccupancyGrid({
      message : message,
      color : that.color,
      opacity : that.opacity
    });

    // check if we care about the scene
    if (that.tfClient) {
      that.currentGrid = new ROS3D.SceneNode({
        frameID : message.header.frame_id,
        tfClient : that.tfClient,
        object : newGrid,
        pose : that.offsetPose
      });
    } else {
      that.currentGrid = newGrid;
    }

    that.rootObject.add(that.currentGrid);

    that.emit('change');

    // check if we should unsubscribe
    if (!that.continuous) {
      rosTopic.unsubscribe();
    }
  });
};
ROS3D.OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;
