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
  this.ros = options.ros;
  this.topicName = options.topic || '/scan';
  this.color = options.color || 0xFFA500;

  this.particles = new ROS3D.Particles(options);

  this.rosTopic = undefined;
  this.subscribe();

};
ROS3D.LaserScan.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.LaserScan.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe();
  }
};

ROS3D.LaserScan.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : this.topicName,
    messageType : 'sensor_msgs/LaserScan'
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.LaserScan.prototype.processMessage = function(message){
  setFrame(this.particles, message.header.frame_id);

  var n = message.ranges.length;
  for(var i=0;i<n;i++){
    var range = message.ranges[i];
    if(range < message.range_min || range > message.range_max){
      this.particles.alpha[i] = 0.0;
    }else{
        var angle = message.angle_min + i * message.angle_increment;
        this.particles.points[i] = new THREE.Vector3( range * Math.cos(angle), range * Math.sin(angle), 0.0 );
        this.particles.alpha[i] = 1.0;
    }
    this.particles.colors[ i ] = new THREE.Color( this.color );
  }

  finishedUpdate(this.particles, n);
};
