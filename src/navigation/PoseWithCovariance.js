/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A PoseWithCovarianceStamped client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 */
ROS3D.PoseWithCovariance = function(options) {
  THREE.Object3D.call(this);
  this.options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/PoseWithCovariance';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();

  this.sn = null;

  this.rosTopic = undefined;
  this.subscribe();
};
ROS3D.PoseWithCovariance.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.PoseWithCovariance.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.PoseWithCovariance.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : this.topicName,
      queue_length : 1,
      messageType : 'geometry_msgs/PoseWithCovarianceStamped'
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.PoseWithCovariance.prototype.processMessage = function(message){
  if(this.sn!==null){
      this.sn.unsubscribeTf();
      this.rootObject.remove(this.sn);
  }

  this.options.origin = new THREE.Vector3( message.pose.pose.position.x, message.pose.pose.position.y,
                                           message.pose.pose.position.z);

  var rot = new THREE.Quaternion(message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                                 message.pose.pose.orientation.z, message.pose.pose.orientation.w);
  this.options.direction = new THREE.Vector3(1,0,0);
  this.options.direction.applyQuaternion(rot);
  this.options.material = new THREE.MeshBasicMaterial({color: this.color});
  var arrow = new ROS3D.Arrow(this.options);

  this.sn = new ROS3D.SceneNode({
      frameID : message.header.frame_id,
      tfClient : this.tfClient,
      object : arrow
  });

  this.rootObject.add(this.sn);
};
