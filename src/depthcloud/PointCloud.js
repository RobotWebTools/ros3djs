/**
 * @author Julius Kammerl - jkammerl@willowgarage.com
 * @author Peter Soetens - peter@thesourceworks.com
 */

/**
 * The PointCloud object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * topic - the topic to subscribe to for PointCloud2 messages
 *   * pointSize (optional) - point size (pixels) for rendered point cloud
 *   * width (optional) - width of the video stream
 *   * height (optional) - height of the video stream
 *   * whiteness (optional) - blends rgb values to white (0..100)
 */
ROS3D.PointCloud = function(options) {
  options = options || {};
  //THREE.Object3D.call(this);

  
  this.topic = options.topic;
  this.pointSize = options.pointSize || 3;
  this.width = options.width || 640;
  this.height = options.height || 480;
  this.whiteness = options.whiteness || 0;

  var ros = options.ros;
  var topic = options.topic;

  
    this.geometry = new THREE.Geometry();
    
    for (var i = 0, l = this.width * this.height; i < l; i++) {
      var vertex = new THREE.Vector3();
      this.geometry.vertices.push(vertex);
      this.geometry.colors.push( new THREE.Color() );
    }
    this.mesh = new THREE.PointCloud( this.geometry, new THREE.PointCloudMaterial( { size: this.pointSize / 1000.0 } ) );
    this.mesh.material.vertexColors = THREE.VertexColors;

    // if we don't clear this, the PC gets undrawn when we get too close with the camera, even if it doesn't seem close.
    this.mesh.frustumCulled = false;
    //this.add(this.mesh);

    // subscribe to the topic
    this.rosTopic = new ROSLIB.Topic({
	ros : ros,
	name : topic,
	messageType : 'sensor_msgs/PointCloud2',
	compression : 'png',
	queue_length  : 0,
	throttle_rate : 500
    });

};
//ROS3D.PointCloud.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Start video playback
 * @todo: move this code + coloring of the point cloud into a vertex+fragment shader.
 * The message.data is a base64 encoded string, but also internally an Image object.
 * Maybe the image can be passed to the vertex shader and decoded there, which would
 * help in speeding up : receiving as png and no longer decompressing in javascript.
 */
ROS3D.PointCloud.prototype.startStream = function() {
    var that = this;
    this.rosTopic.subscribe(function(message) {
	console.log('pc in!');
	var floatView = dcodeIO.ByteBuffer.wrap(message.data, 'base64', dcodeIO.ByteBuffer.LITTLE_ENDIAN);
	for (var i = 0, l = message.width * message.height; i < that.geometry.vertices.length; i++) {
            if ( i < l ) {
		that.geometry.vertices[i].x = floatView.readFloat32();
		that.geometry.vertices[i].y = floatView.readFloat32();
		that.geometry.vertices[i].z = floatView.readFloat32();
		floatView.offset += 4; // skip empty channel
		that.geometry.colors[i].b   = floatView.readUint8() / 255.0;
		that.geometry.colors[i].g   = floatView.readUint8() / 255.0;
		that.geometry.colors[i].r   = floatView.readUint8() / 255.0;
		floatView.offset += message.point_step - (3 + 4 + 3*4); //  - (rgb + skip empty + 3 floats(xyz) )
            } else {
		that.geometry.vertices[i].x = that.geometry.vertices[i-1].x;
		that.geometry.vertices[i].y = that.geometry.vertices[i-1].y;
		that.geometry.vertices[i].z = that.geometry.vertices[i-1].z;
            }
	}
	console.log('pc done! ' + message.width + ' ' + message.height + ' ' + that.geometry.vertices.length);
	console.log('colors : ' + that.geometry.colors[10000].r + ' ' + that.geometry.colors[10000].g + ' ' + that.geometry.colors[10000].b + ' ' );
	that.geometry.verticesNeedUpdate = true;
	that.geometry.colorsNeedUpdate = true;
	//that.rosTopic.unsubscribe();
    });
};

/**
 * Stop video playback
 */
ROS3D.PointCloud.prototype.stopStream = function() {
    this.rosTopic.unsubscribe();
};
