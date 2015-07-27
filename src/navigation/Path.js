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
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/path';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);

  this.sn = null;
  this.line = null;

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'nav_msgs/Path'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
      }

      var lineGeometry = new THREE.Geometry();
      for(var i=0; i<message.poses.length;i++){
          var v3 = new THREE.Vector3( message.poses[i].pose.position.x, message.poses[i].pose.position.y,
                                      message.poses[i].pose.position.z);
          lineGeometry.vertices.push(v3);
      }

      lineGeometry.computeLineDistances();
      var lineMaterial = new THREE.LineBasicMaterial( { color: that.color } );
      var line = new THREE.Line( lineGeometry, lineMaterial );

      that.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : that.tfClient,
          object : line
      });

      that.rootObject.add(that.sn);
  });
};
ROS3D.Path.prototype.__proto__ = THREE.Object3D.prototype;
