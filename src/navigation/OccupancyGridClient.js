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
 *   * compression (optional) - message compression (default: 'cbor')
 *   * rootObject (optional) - the root object to add this marker to
 *   * offsetPose (optional) - offset pose of the grid visualization, e.g. for z-offset (ROSLIB.Pose type)
 *   * color (optional) - color of the visualized grid
 *   * opacity (optional) - opacity of the visualized grid (0.0 == fully transparent, 1.0 == opaque)
 */
ROS3D.OccupancyGridClient = function(options) {
  EventEmitter2.call(this);
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/map';
  this.compression = options.compression || 'cbor';
  this.continuous = options.continuous;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.offsetPose = options.offsetPose || new ROSLIB.Pose();
  this.color = options.color || {r:255,g:255,b:255};
  this.opacity = options.opacity || 1.0;

  // current grid that is displayed
  this.currentGrid = null;

  // subscribe to the topic
  this.rosTopic = undefined;
  this.subscribe();
};
ROS3D.OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;

ROS3D.OccupancyGridClient.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.OccupancyGridClient.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : this.topicName,
    messageType : 'nav_msgs/OccupancyGrid',
    queue_length : 1,
    compression : this.compression
  });
  this.sceneNode = null;
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.OccupancyGridClient.prototype.processMessage = function(message){
  // check for an old map
  if (this.currentGrid) {
    // check if it there is a tf client
    if (this.tfClient) {
      // grid is of type ROS3D.SceneNode
      this.sceneNode.unsubscribeTf();
      this.sceneNode.remove(this.currentGrid);
    } else {
      this.rootObject.remove(this.currentGrid);
    }
    this.currentGrid.dispose();
  }

  var newGrid = new ROS3D.OccupancyGrid({
    message : message,
    color : this.color,
    opacity : this.opacity
  });

  // check if we care about the scene
  if (this.tfClient) {
    this.currentGrid = newGrid;
    if (this.sceneNode === null) {
      this.sceneNode = new ROS3D.SceneNode({
        frameID : message.header.frame_id,
        tfClient : this.tfClient,
        object : newGrid,
        pose : this.offsetPose
      });
      this.rootObject.add(this.sceneNode);
    } else {
      this.sceneNode.add(this.currentGrid);
    }
  } else {
    this.sceneNode = this.currentGrid = newGrid;
    this.rootObject.add(this.currentGrid);
  }

  this.emit('change');

  // check if we should unsubscribe
  if (!this.continuous) {
    this.rosTopic.unsubscribe(this.processMessage);
  }
};
