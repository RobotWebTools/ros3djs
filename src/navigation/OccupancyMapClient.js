/**
 * @author Peter Sari - sari@photoneo.com
 */

/**
 * A 3D Occupancy map ...
 * TODO: TBD
 */


// ...

ROS3D.OccupancyMapClient = function (options) {
  EventEmitter2.call(this);
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/octomap';
  this.compression = options.compression || 'cbor';
  this.continuous = options.continuous;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.offsetPose = options.offsetPose || new ROSLIB.Pose();

  // current grid that is displayed
  this.currentMap = null;

  // subscribe to the topic
  this.rosTopic = undefined;
  this.subscribe();
};

ROS3D.OccupancyMapClient.prototype.unsubscribe = function () {
  if (this.rosTopic) {
    this.rosTopic.unsubscribe();
  }
};

ROS3D.OccupancyMapClient.prototype.subscribe = function () {
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

ROS3D.OccupancyMapClient.prototype.processMessage = function (message) {
  // check for an old map
  if (this.currentMap) {
    // check if it there is a tf client
    if (this.currentMap.tfClient) {
      // grid is of type ROS3D.SceneNode
      this.currentMap.unsubscribeTf();
    }
    // this.rootObject.remove(this.currentMap);
  }

  console.log(message);

  new Promise(
    // 1. Create the corresponding octree object from message
    function (resolve, reject) {
      let newOcTree = null;
      if (message.binary) {
        newOcTree = new ROS3D.OcTreeBase({
          resolution: message.resolution
        });
        newOcTree.readBinary(message.data);
      } else {

        const ctorTable = {
          'OcTree': ROS3D.OcTree,
          'ColorOcTree': ROS3D.ColorOcTree,
        };

        if (message.id in ctorTable) {
          console.log(message.id);

          newOcTree = new ctorTable[message.id]({
            resolution: message.resolution
          });

          newOcTree.read(message.data);
        }

      }

      resolve(newOcTree);
    }
  ).then(
    // 2. Build geometry from octree
    function (newOcTree) {
      newOcTree.buildGeometry();
      return newOcTree;
    }
  ).then(
    // 3. Replace geometry
    function (newOcTree) {
      // check if we care about the scene
      const oldNode = this.sceneNode;
      if (this.tfClient) {
        this.currentMap = newOcTree;
        this.sceneNode = new ROS3D.SceneNode({
          frameID: message.header.frame_id,
          tfClient: this.tfClient,
          object: newOcTree.object,
          pose: this.offsetPose
        });
      } else {
        this.sceneNode = newOcTree.object;
        this.currentMap = newOcTree;
      }
      this.rootObject.remove(oldNode);
      this.rootObject.add(this.sceneNode);

      // console.log({ oldNode, sceneNode: this.sceneNode });

      // this.emit('change');
    }.bind(this)
  ).then(function () {
    // check if we should unsubscribe
    if (!this.continuous) {
      this.rosTopic.unsubscribe();
    }

  }.bind(this)
  );

};
