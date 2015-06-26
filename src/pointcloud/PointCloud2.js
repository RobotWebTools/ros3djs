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
 * @constructor
 * @param options - object with following keys:
 *
 *  * color (optional) - the line color of the grid, like '#cccccc'
 *  * lineWidth (optional) - the width of the lines in the grid
 *  * cellSize (optional) - The length, in meters, of the side of each cell
 */
ROS3D.PointCloud = function(options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/points';
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.viewer = options.viewer;
  var that = this;
  this.cloud = null;
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
    '    gl_PointSize = 0.01 * ( 300.0 / length( mvPosition.xyz ) );',
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

    var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'sensor_msgs/PointCloud2'
    });
    
    rosTopic.subscribe(function(message) {
        //that.ps.geometry.__dirtyVertices = true;
        for(var i=0;i<message.height*message.width;i++){
            var pt = read_point(message, i);
            that.geom.vertices.push(new THREE.Vector3( pt['x'], pt['y'], pt['z'] ));
            that.attribs.customColor.value[ i ] = new THREE.Color( pt['rgb'] );
        }

    ps = new THREE.ParticleSystem( that.geom, that.shaderMaterial );
    that.rootObject.add(ps);
    });


};
ROS3D.PointCloud.prototype.__proto__ = THREE.Object3D.prototype;
