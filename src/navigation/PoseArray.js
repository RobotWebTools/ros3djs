/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A PoseArray client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * length (optional) - the length of the arrow (default: 1.0)
 */
ROS3D.PoseArray = function(options) {
  THREE.Object3D.call(this);
  this.options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/particlecloud';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.length = options.length || 1.0;
  this.rootObject = options.rootObject || new THREE.Object3D();

  this.sn = null;

  this.rosTopic = undefined;
  this.subscribe();
};
ROS3D.PoseArray.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.PoseArray.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.PoseArray.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
     ros : this.ros,
     name : this.topicName,
     queue_length : 1,
     messageType : 'geometry_msgs/PoseArray'
 });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.PoseArray.prototype.processMessage = function(message){
  if(this.sn!==null){
      this.sn.unsubscribeTf();
      this.rootObject.remove(this.sn);
  }

  var group = new THREE.Object3D();
  var line;

  for(var i=0;i<message.poses.length;i++){
      var lineGeometry = new THREE.Geometry();

      var v3 = new THREE.Vector3( message.poses[i].position.x, message.poses[i].position.y,
                                  message.poses[i].position.z);
      lineGeometry.vertices.push(v3);

      var rot = new THREE.Quaternion(message.poses[i].orientation.x, message.poses[i].orientation.y,
                                     message.poses[i].orientation.z, message.poses[i].orientation.w);

      var tip = new THREE.Vector3(this.length,0,0);
      var side1 = new THREE.Vector3(this.length*0.8, this.length*0.2, 0);
      var side2 = new THREE.Vector3(this.length*0.8, -this.length*0.2, 0);
      tip.applyQuaternion(rot);
      side1.applyQuaternion(rot);
      side2.applyQuaternion(rot);

      lineGeometry.vertices.push(tip.add(v3));
      lineGeometry.vertices.push(side1.add(v3));
      lineGeometry.vertices.push(side2.add(v3));
      lineGeometry.vertices.push(tip);

      lineGeometry.computeLineDistances();
      var lineMaterial = new THREE.LineBasicMaterial( { color: this.color } );
      line = new THREE.Line( lineGeometry, lineMaterial );

      group.add(line);
  }

  this.sn = new ROS3D.SceneNode({
      frameID : message.header.frame_id,
      tfClient : this.tfClient,
      object : group
  });

  this.rootObject.add(this.sn);
};
