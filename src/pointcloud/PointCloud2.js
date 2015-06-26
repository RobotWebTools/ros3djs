/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

function read_point(msg, index){
    pt = [];
    var base = msg.point_step * index;
    for(var fi=0; fi<msg.fields.length; fi++){
        var si = base + msg.fields[fi].offset;
        var ar = new Uint8Array(4);
        for(var i=0; i<ar.length; i++){
            ar[i] = msg.data.buffer[si + i];
        }
        
        var dv = new DataView(ar.buffer);
        
        if( msg.fields[fi].name == 'rgb' ){
            pt[ 'rgb' ] =dv.getInt32(0, 1);
        }else{
            pt[ msg.fields[fi].name ] = dv.getFloat32(0, 1);
        }    
    }
    return pt;
}

/**
 * A point cloud client that listens to a given topic and displays the points.
 * 
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - the ROSLIB.Ros connection handle
 *  * topic - the marker topic to listen to
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to
 *  * size (optional) - size to draw each point (default 0.05)
 *  * max_pts (optional) - number of points to draw (default 100)
 */
ROS3D.PointCloud = function(options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/points';
  var size = options.size || 0.05;
  var max_pts = options.max_pts || 100;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);
  
  this.vertex_shader = [
    'attribute vec3 customColor;',
    'varying vec3 vColor;',
    'void main() ',
    '{',
    '    vColor = customColor; // set color associated to vertex; use later in fragment shader',
    '    vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );',
    '',
    '    // option (1): draw particles at constant size on screen',
    '    // gl_PointSize = size;',
    '    // option (2): scale particles as objects in 3D space',
    '    gl_PointSize = ', size, '* ( 300.0 / length( mvPosition.xyz ) );',
    '    gl_Position = projectionMatrix * mvPosition;',
    '}'
    ].join('\n');

  this.fragment_shader = [
    'uniform sampler2D texture;',
    'varying vec3 vColor; // colors associated to vertices; assigned by vertex shader',
    'void main() ',
    '{',
    '    // calculates a color for the particle',
    '    gl_FragColor = vec4( vColor, 1.0 );',
    '    // sets particle texture to desired color',
    '    gl_FragColor = gl_FragColor * texture2D( texture, gl_PointCoord );',
    '}'
    ].join('\n');
    
    this.geom = new THREE.Geometry();
    for(var i=0;i<max_pts;i++){
        this.geom.vertices.push(new THREE.Vector3( ));
    }    
    
    var customUniforms = 
    {
        texture:   { type: "t", value: THREE.ImageUtils.loadTexture( 'pixel.png' ) },
    };
    
    this.attribs = 
    {
        customColor:   { type: "c", value: [] },
    };
    
    this.shaderMaterial = new THREE.ShaderMaterial( 
    {
        uniforms:         customUniforms,
        attributes:        this.attribs,
        vertexShader : that.vertex_shader,
        fragmentShader : that.fragment_shader,
        transparent: true, alphaTest: 0.5
    });
    
    this.ps = new THREE.ParticleSystem( this.geom, this.shaderMaterial );
    this.rootObject.add(this.ps);

    var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'sensor_msgs/PointCloud2'
    });
    
    rosTopic.subscribe(function(message) {
        for(var i=0;i<message.height*message.width;i++){
            var pt = read_point(message, i);
            that.geom.vertices[i] = new THREE.Vector3( pt['x'], pt['y'], pt['z'] );
            that.attribs.customColor.value[ i ] = new THREE.Color( pt['rgb'] );
        }
        that.geom.verticesNeedUpdate = true;
        that.attribs.customColor.needsUpdate = true;
    });


};
ROS3D.PointCloud.prototype.__proto__ = THREE.Object3D.prototype;
