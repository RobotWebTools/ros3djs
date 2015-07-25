/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A PointStamped client
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * radius (optional) - radius of the point (default: 0.2)
 */
ROS3D.Point = function(options) {
  this.options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/point';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.radius = options.radius || 0.2;
  var that = this;
  THREE.Object3D.call(this);

  this.sn = null;

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'geometry_msgs/PointStamped'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
      }

      var sphereGeometry = new THREE.SphereGeometry( that.radius );
      var sphereMaterial = new THREE.MeshBasicMaterial( {color: that.color} );
      var sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
      sphere.position.set(message.point.x, message.point.y, message.point.z);

      that.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : that.tfClient,
          object : sphere
      });

      that.rootObject.add(that.sn);
  });
};
ROS3D.Point.prototype.__proto__ = THREE.Object3D.prototype;
