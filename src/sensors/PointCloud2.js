/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

function read_point(msg, index, data_view){
    var pt = [];
    var base = msg.point_step * index;
    var n = 4;
    for(var fi=0; fi<msg.fields.length; fi++){
        var si = base + msg.fields[fi].offset;

        if( msg.fields[fi].name === 'rgb' ){
            pt[ 'rgb' ] = data_view.getInt32(si, 1);
        }else{
            pt[ msg.fields[fi].name ] = data_view.getFloat32(si, 1);
        }
    }
    return pt;
}

var BASE64 = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=';
function decode64(x) {
    var a = [], z = 0, bits = 0;

    for (var i = 0, len = x.length; i < len; i++) {
      z += BASE64.indexOf( x[i] );
      bits += 6;
      if(bits>=8){
          bits -= 8;
          a.push(z >> bits);
          z = z & (Math.pow(2, bits)-1);
      }
      z = z << 6;
    }
    return a;
}

/**
 * A PointCloud2 client that listens to a given topic and displays the points.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * texture - (optional) Image url for a texture to use for the points. Defaults to a single white pixel.
 *  * rootObject (optional) - the root object to add this marker to
 *  * size (optional) - size to draw each point (default 0.05)
 *  * max_pts (optional) - number of points to draw (default 100)
 *  * color (optional) - point color (otherwise taken from the topic)
 */
ROS3D.PointCloud2 = function(options) {
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/points';
  this.color = options.color;

  this.particles = new ROS3D.Particles(options);
  this.rosTopic = undefined;
  this.subscribe();
};
ROS3D.PointCloud2.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.PointCloud2.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe();
  }
};

ROS3D.PointCloud2.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : this.topicName,
    messageType : 'sensor_msgs/PointCloud2'
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.PointCloud2.prototype.processMessage = function(message){
  setFrame(this.particles, message.header.frame_id);

  var n = message.height*message.width;
  var buffer;
  if(message.data.buffer){
    buffer = message.data.buffer.buffer;
  }else{
    buffer = Uint8Array.from(decode64(message.data)).buffer;
  }
  var dv = new DataView(buffer);
  var color;
  if(this.color !== undefined){
    color = new THREE.Color(this.color);
  }
  for(var i=0;i<n;i++){
    var pt = read_point(message, i, dv);
    this.particles.points[i] = new THREE.Vector3( pt['x'], pt['y'], pt['z'] );
    this.particles.colors[ i ] = color || new THREE.Color( pt['rgb'] );
    this.particles.alpha[i] = 1.0;
  }

  finishedUpdate(this.particles, n);
};
