/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A PoseStamped client
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
 *  * headLength (optional) - the head length of the arrow (default: 0.2)
 *  * shaftDiameter (optional) - the shaft diameter of the arrow (default: 0.05)
 *  * headDiameter (optional) - the head diameter of the arrow (default: 0.1)
 */
ROS3D.Pose = function(options) {
  this.options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/pose';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);

  this.sn = null;

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'geometry_msgs/PoseStamped'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
      }

      that.options.origin = new THREE.Vector3( message.pose.position.x, message.pose.position.y,
                                               message.pose.position.z);

      var rot = new THREE.Quaternion(message.pose.orientation.x, message.pose.orientation.y,
                                     message.pose.orientation.z, message.pose.orientation.w);
      that.options.direction = new THREE.Vector3(1,0,0);
      that.options.direction.applyQuaternion(rot);
      that.options.material = new THREE.MeshBasicMaterial({color: that.color});
      var arrow = new ROS3D.Arrow(that.options);

      that.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : that.tfClient,
          object : arrow
      });

      that.rootObject.add(that.sn);
  });
};
ROS3D.Pose.prototype.__proto__ = THREE.Object3D.prototype;
