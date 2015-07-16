/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

function read_point(msg, index, buffer){
    var pt = [];
    var base = msg.point_step * index;
    var n = 4;
    var ar = new Uint8Array(n);
    for(var fi=0; fi<msg.fields.length; fi++){
        var si = base + msg.fields[fi].offset;
        for(var i=0; i<n; i++){
            ar[i] = buffer[si + i];
        }

        var dv = new DataView(ar.buffer);

        if( msg.fields[fi].name === 'rgb' ){
            pt[ 'rgb' ] =dv.getInt32(0, 1);
        }else{
            pt[ msg.fields[fi].name ] = dv.getFloat32(0, 1);
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
 */
ROS3D.PointCloud2 = function(options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/points';
  var that = this;

  this.particles = new ROS3D.Particles(options);

  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'sensor_msgs/PointCloud2'
  });

  rosTopic.subscribe(function(message) {
    setFrame(that.particles, message.header.frame_id);

    var n = message.height*message.width;
    var buffer;
    if(message.data.buffer){
      buffer = message.data.buffer;
    }else{
      buffer = decode64(message.data);
    }
    for(var i=0;i<n;i++){
      var pt = read_point(message, i, buffer);
      that.particles.points[i] = new THREE.Vector3( pt['x'], pt['y'], pt['z'] );
      that.particles.colors[ i ] = new THREE.Color( pt['rgb'] );
      that.particles.alpha[i] = 1.0;
    }

    finishedUpdate(that.particles, n);
  });
};
ROS3D.PointCloud2.prototype.__proto__ = THREE.Object3D.prototype;
