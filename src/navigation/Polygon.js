/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A PolygonStamped client that listens to a given topic and displays the polygon
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
ROS3D.Polygon = function(options) {
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
      messageType : 'geometry_msgs/PolygonStamped'
  });

  rosTopic.subscribe(function(message) {
      if(that.sn!==null){
          that.rootObject.remove(that.sn);
      }

      var lineGeometry = new THREE.Geometry();
      var v3;
      for(var i=0; i<message.polygon.points.length;i++){
          v3 = new THREE.Vector3( message.polygon.points[i].x, message.polygon.points[i].y,
                                  message.polygon.points[i].z);
          lineGeometry.vertices.push(v3);
      }
      v3 = new THREE.Vector3( message.polygon.points[0].x, message.polygon.points[0].y,
                              message.polygon.points[0].z);
      lineGeometry.vertices.push(v3);
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
ROS3D.Polygon.prototype.__proto__ = THREE.Object3D.prototype;
