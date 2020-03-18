/**
 * @author Peter Sari - sari@photoneo.com
 */

/**
 * A 3D Occupancy map ...
 * TODO: TBD
 */

// ...

ROS3D.OccupancyMapClient = function(options) {
  EventEmitter2.call(this);
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/map';
  this.compression = options.compression || 'cbor';
  this.continuous = options.continuous;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.offsetPose = options.offsetPose || new ROSLIB.Pose();

  // current grid that is displayed
  this.currentGrid = null;

  // subscribe to the topic
  this.rosTopic = undefined;
  this.subscribe();
};

ROS3D.OccupancyMapClient.prototype.unsubscribe = function() {
  if (this.rosTopic) {
    this.rosTopic.unsubscribe();
  }
};

ROS3D.OccupancyMapClient.prototype.subscribe = function() {
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
    ros: this.ros,
    name: this.topicName,
    messageType: 'octomap_msgs/Octomap',
    queue_length: 1,
    compression: this.compression
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.OccupancyMapClient.prototype.processMessage = function(message) {
  // check for an old map
  if (this.currentGrid) {
    // check if it there is a tf client
    if (this.currentGrid.tfClient) {
      // grid is of type ROS3D.SceneNode
      this.currentGrid.unsubscribeTf();
    }
    this.rootObject.remove(this.currentGrid);
  }

  var newGrid = new ROS3D.Octomap({
    message: message
    //   color : this.color,
    //   opacity : this.opacity
  });

  // console.log("Message: ", { message });

  // // check if we care about the scene
  // if (this.tfClient) {
  //   this.currentGrid = newGrid;
  //   this.sceneNode = new ROS3D.SceneNode({
  //     frameID : message.header.frame_id,
  //     tfClient : this.tfClient,
  //     object : newGrid,
  //     pose : this.offsetPose
  //   });
  // } else {
  //   this.sceneNode = this.currentGrid = newGrid;
  // }

  this.rootObject.add(this.sceneNode);

  this.emit('change');

  // check if we should unsubscribe
  if (!this.continuous) {
    this.rosTopic.unsubscribe();
  }
};
