/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A LaserScan client that listens to a given topic and displays the points.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * color - (optional) color of the points (default 0xFFA500)
 *  * texture - (optional) Image url for a texture to use for the points. Defaults to a single white pixel.
 *  * rootObject (optional) - the root object to add this marker to
 *  * size (optional) - size to draw each point (default 0.05)
 *  * max_pts (optional) - number of points to draw (default 100)
 */
ROS3D.LaserScan = function(options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/scan';
  this.color = options.color || 0xFFA500;
  var that = this;

  this.particles = new ROS3D.Particles(options);

  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'sensor_msgs/LaserScan'
  });


  rosTopic.subscribe(function(message) {
    setFrame(that.particles, message.header.frame_id);

    var n = message.ranges.length;
    for(var i=0;i<n;i++){
      var range = message.ranges[i];
      if(range < message.range_min || range > message.range_max){
        that.particles.alpha[i] = 0.0;
      }else{
          var angle = message.angle_min + i * message.angle_increment;
          that.particles.points[i] = new THREE.Vector3( range * Math.cos(angle), range * Math.sin(angle), 0.0 );
          that.particles.alpha[i] = 1.0;
      }
      that.particles.colors[ i ] = new THREE.Color( that.color );
    }

    finishedUpdate(that.particles, n);
  });
};
ROS3D.LaserScan.prototype.__proto__ = THREE.Object3D.prototype;
