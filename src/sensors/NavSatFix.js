/**
 * @author Mathieu Bredif - mathieu.bredif@ign.fr
 */

/**
 * A NavSatFix client that listens to a given topic and displays a line connecting the gps fixes.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the NavSatFix topic to listen to
 *  * rootObject (optional) - the root object to add the trajectory line and the gps marker to
 *  * object3d (optional) - the object3d to be translated by the gps position
 *  * material (optional) - THREE.js material or options passed to a THREE.LineBasicMaterial, such as :
 *    * material.color (optional) - color for line
 *    * material.linewidth (optional) - line width
 *  * altitudeNaN (optional) - default altitude when the message altitude is NaN (default: 0)
 *  * keep (optional) - number of gps fix points to keep (default: 100)
 *  * convert (optional) - conversion function from lon/lat/alt to THREE.Vector3 (default: passthrough)
 */

ROS3D.NavSatFix = function(options) {
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/gps/fix';
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.object3d = options.object3d || new THREE.Object3D();
  var material = options.material || {};
  this.altitudeNaN = options.altitudeNaN || 0;
  this.keep = options.keep || 100;
  this.convert = options.convert || function(lon,lat,alt) { return new THREE.Vector3(lon,lat,alt); };
  this.count = 0;
  this.next1 = 0;
  this.next2 = this.keep;

  this.geom = new THREE.BufferGeometry();
  this.vertices = new THREE.BufferAttribute(new Float32Array( 6 * this.keep ), 3 );
  this.geom.addAttribute( 'position',  this.vertices);
  this.material = material.isMaterial ? material : new THREE.LineBasicMaterial( material );
  this.line = new THREE.Line( this.geom, this.material );
  this.rootObject.add(this.object3d);
  this.rootObject.add(this.line);

  this.rosTopic = undefined;
  this.subscribe();
};
ROS3D.NavSatFix.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.NavSatFix.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.NavSatFix.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : this.topicName,
      queue_length : 1,
      messageType : 'sensor_msgs/NavSatFix'
  });

  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.NavSatFix.prototype.processMessage = function(message){
  var altitude = isNaN(message.altitude) ? this.altitudeNaN : message.altitude;
  var p = this.convert(message.longitude, message.latitude, altitude);

  // move the object3d to the gps position
  this.object3d.position.copy(p);
  this.object3d.updateMatrixWorld(true);

  // copy the position twice in the circular buffer
  // the second half replicates the first to allow a single drawRange
  this.vertices.array[3*this.next1  ] = p.x;
  this.vertices.array[3*this.next1+1] = p.y;
  this.vertices.array[3*this.next1+2] = p.z;
  this.vertices.array[3*this.next2  ] = p.x;
  this.vertices.array[3*this.next2+1] = p.y;
  this.vertices.array[3*this.next2+2] = p.z;
  this.vertices.needsUpdate = true;

  this.next1 = (this.next1+1) % this.keep;
  this.next2 = this.next1 + this.keep;
  this.count = Math.min(this.count+1, this.keep);
  this.geom.setDrawRange(this.next2-this.count, this.count );
};
