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
  this.options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/particlecloud';
  this.tfClient = options.tfClient;
  this.color = options.color || 0xcc00ff;
  this.length = options.length || 1.0;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);

  this.sn = null;

  var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'geometry_msgs/PoseArray'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
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

          var tip = new THREE.Vector3(that.length,0,0);
          var side1 = new THREE.Vector3(that.length*0.8, that.length*0.2, 0);
          var side2 = new THREE.Vector3(that.length*0.8, -that.length*0.2, 0);
          tip.applyQuaternion(rot);
          side1.applyQuaternion(rot);
          side2.applyQuaternion(rot);

          lineGeometry.vertices.push(tip.add(v3));
          lineGeometry.vertices.push(side1.add(v3));
          lineGeometry.vertices.push(side2.add(v3));
          lineGeometry.vertices.push(tip);

          lineGeometry.computeLineDistances();
          var lineMaterial = new THREE.LineBasicMaterial( { color: that.color } );
          line = new THREE.Line( lineGeometry, lineMaterial );

          group.add(line);
      }

      that.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : that.tfClient,
          object : group
      });

      that.rootObject.add(that.sn);
  });
};
ROS3D.PoseArray.prototype.__proto__ = THREE.Object3D.prototype;
