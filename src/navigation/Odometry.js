/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * An Odometry client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * keep (optional) - number of markers to keep around (default: 1)
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * length (optional) - the length of the arrow (default: 1.0)
 *  * headLength (optional) - the head length of the arrow (default: 0.2)
 *  * shaftDiameter (optional) - the shaft diameter of the arrow (default: 0.05)
 *  * headDiameter (optional) - the head diameter of the arrow (default: 0.1)
 */
ROS3D.Odometry = function(options) {
  this.options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/particlecloud';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.length = options.length || 1.0;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.keep = options.keep || 1;
  var that = this;
  THREE.Object3D.call(this);

  this.sns = [];

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'nav_msgs/Odometry'
  });

  rosTopic.subscribe(function(message) {
      if(that.sns.length >= that.keep) {
          that.rootObject.remove(that.sns[0]);
          that.sns.shift();
      }

      that.options.origin = new THREE.Vector3( message.pose.pose.position.x, message.pose.pose.position.y,
                                               message.pose.pose.position.z);

      var rot = new THREE.Quaternion(message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                                     message.pose.pose.orientation.z, message.pose.pose.orientation.w);
      that.options.direction = new THREE.Vector3(1,0,0);
      that.options.direction.applyQuaternion(rot);
      that.options.material = new THREE.MeshBasicMaterial({color: that.color});
      var arrow = new ROS3D.Arrow(that.options);

      that.sns.push(new ROS3D.SceneNode({
            frameID : message.header.frame_id,
            tfClient : that.tfClient,
            object : arrow
      }));

      that.rootObject.add(that.sns[ that.sns.length - 1]);
  });
};
ROS3D.Odometry.prototype.__proto__ = THREE.Object3D.prototype;
