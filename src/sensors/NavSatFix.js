/**
 * @author Mathieu Bredif!! - mathieu.bredif@ign.fr
 */

/**
 * A NavSatFix client that listens to a given topic and displays a line connecting the gps fixes.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the NavSatFix topic to listen to
 *  * rootObject (optional) - the root object to add this marker to
 *  * color (optional) - color for line (default: 0xcc00ff)
 *  * keep (optional) - number of gps fix points to keep (default: 100)
 *  * convert (optional) - conversion from lat/lon/alt to xyz (default: passthrough)
 */
ROS3D.NavSatFix = function(options) {
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic || '/gps/fix';
  this.color = options.color || 0xcc00ff;
  this.convert = options.convert || function(lat, lon, alt) { return new THREE.Vector3(lat, lon, alt); };
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.keep = options.keep || 100;
  this.count = 0;
  this.next1 = 0;
  this.next2 = this.keep;
  THREE.Object3D.call(this);

  this.geom = new THREE.BufferGeometry();
  this.vertices = new THREE.BufferAttribute(new Float32Array( 6 * this.keep ), 3 );
  this.geom.addAttribute( 'position',  this.vertices);
  this.material = new THREE.LineBasicMaterial( { color: this.color } );
  this.object = new THREE.Line( this.geom, this.material );
  this.group = new THREE.Group();
  this.add(this.group);
  this.add(this.object);
  this.rootObject.add(this);

  this.rosTopic = undefined;
  this.subscribe();
};
ROS3D.NavSatFix.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.NavSatFix.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe();
  }
};

ROS3D.NavSatFix.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to the topic
  this.rosTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : this.topicName,
      messageType : 'sensor_msgs/NavSatFix'
  });

  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.NavSatFix.prototype.processMessage = function(message){
  var pt = this.convert(message.longitude, message.latitude, message.altitude);

  // move the group to the gps position
  this.group.position.set(pt.x, pt.y, pt.z);
  this.group.updateMatrix();

  // copy the position twice in the circular buffer
  // the second half replicates the first to allow a single drawRange
  this.vertices.array[3*this.next1  ] = pt.x;
  this.vertices.array[3*this.next1+1] = pt.y;
  this.vertices.array[3*this.next1+2] = pt.z;
  this.vertices.array[3*this.next2  ] = pt.x;
  this.vertices.array[3*this.next2+1] = pt.y;
  this.vertices.array[3*this.next2+2] = pt.z;
  this.vertices.needsUpdate = true;

  this.next1 = (this.next1+1) % this.keep;
  this.next2 = this.next1 + this.keep;
  this.count = Math.min(this.count+1, this.keep);
  this.geom.setDrawRange(this.next2-this.count, this.count );
};
