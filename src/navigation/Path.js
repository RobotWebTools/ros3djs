/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A Path client that listens to a given topic and displays a line connecting the poses.
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
ROS3D.Path = function(options) {
  THREE.Object3D.call(this);
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/path';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();

  this.sn = null;
  this.line = null;

  this.rosTopic = undefined;
  this.subscribe();
};
ROS3D.Path.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.Path.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.Path.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : this.topicName,
      queue_length : 1,
      messageType : 'nav_msgs/Path'
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.Path.prototype.processMessage = function(message){
  if(this.sn!==null){
      this.sn.unsubscribeTf();
      this.rootObject.remove(this.sn);
  }

  var lineGeometry = new THREE.Geometry();
  for(var i=0; i<message.poses.length;i++){
      var v3 = new THREE.Vector3( message.poses[i].pose.position.x, message.poses[i].pose.position.y,
                                  message.poses[i].pose.position.z);
      lineGeometry.vertices.push(v3);
  }

  lineGeometry.computeLineDistances();
  var lineMaterial = new THREE.LineBasicMaterial( { color: this.color } );
  var line = new THREE.Line( lineGeometry, lineMaterial );

  this.sn = new ROS3D.SceneNode({
      frameID : message.header.frame_id,
      tfClient : this.tfClient,
      object : line
  });

  this.rootObject.add(this.sn);
};
