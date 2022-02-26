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
 *  * topic - the marker topic to listen to (default '/scan')
 *  * tfClient - the TF client handle to use
 *  * compression (optional) - message compression (default: 'cbor')
 *  * rootObject (optional) - the root object to add this marker to use for the points.
 *  * max_pts (optional) - number of points to draw (default: 10000)
 *  * pointRatio (optional) - point subsampling ratio (default: 1, no subsampling)
 *  * messageRatio (optional) - message subsampling ratio (default: 1, no subsampling)
 *  * material (optional) - a material object or an option to construct a PointsMaterial.
 */
ROS3D.LaserScan = function(options) {
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/scan';
  this.compression = options.compression || 'cbor';
  this.points = new ROS3D.Points(options);
  this.rosTopic = undefined;
  this.subscribe();

};
ROS3D.LaserScan.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.LaserScan.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.LaserScan.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : this.topicName,
    compression : this.compression,
    queue_length : 1,
    messageType : 'sensor_msgs/LaserScan'
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.LaserScan.prototype.processMessage = function(message){
  if(!this.points.setup(message.header.frame_id)) {
      return;
  }
  var n = message.ranges.length;
  var j = 0;
  for(var i=0;i<n;i+=this.points.pointRatio){
    var range = message.ranges[i];
    if(range >= message.range_min && range <= message.range_max){
        var angle = message.angle_min + i * message.angle_increment;
        this.points.positions.array[j++] = range * Math.cos(angle);
        this.points.positions.array[j++] = range * Math.sin(angle);
        this.points.positions.array[j++] = 0.0;
    }
  }
  this.points.update(j/3);
};
