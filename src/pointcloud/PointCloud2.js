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
 *  * rootObject (optional) - the root object to add this marker to
 *  * size (optional) - size to draw each point (default 0.05)
 *  * max_pts (optional) - number of points to draw (default 100)
 */
ROS3D.PointCloud2 = function(options) {
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/points';
  this.tfClient = options.tfClient;
  var size = options.size || 0.05;
  var max_pts = options.max_pts || 100;
  this.prev_pts = 0;
  this.rootObject = options.rootObject || new THREE.Object3D();
  var that = this;
  THREE.Object3D.call(this);

  this.vertex_shader = [
    'attribute vec3 customColor;',
    'attribute float alpha;',
    'varying vec3 vColor;',
    'varying float falpha;',
    'void main() ',
    '{',
    '    vColor = customColor; // set color associated to vertex; use later in fragment shader',
    '    vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );',
    '    falpha = alpha; ',
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
    'varying float falpha;',
    'void main() ',
    '{',
    '    // calculates a color for the particle',
    '    gl_FragColor = vec4( vColor, falpha );',
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
        texture:   { type: 't', value: THREE.ImageUtils.loadTexture( 'pixel.png' ) },
    };

    this.attribs =
    {
        customColor:   { type: 'c', value: [] },
        alpha:         { type: 'f', value: [] }
    };

    this.shaderMaterial = new THREE.ShaderMaterial(
    {
        uniforms:          customUniforms,
        attributes:        this.attribs,
        vertexShader:      this.vertex_shader,
        fragmentShader:    this.fragment_shader,
        transparent: true, alphaTest: 0.5
    });

    this.ps = new THREE.ParticleSystem( this.geom, this.shaderMaterial );
    this.sn = null;

    var rosTopic = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'sensor_msgs/PointCloud2'
    });

    rosTopic.subscribe(function(message) {
        rosTopic.unsubscribe();

        if(that.sn===null){
            that.sn = new ROS3D.SceneNode({
                frameID : message.header.frame_id,
                tfClient : that.tfClient,
                object : that.ps
            });

            that.rootObject.add(that.sn);
        }

        var n = message.height*message.width;

        var buffer;
        if(message.data.buffer){
            buffer = message.data.buffer;
        }else{
            buffer = decode64(message.data);
        }
        for(var i=0;i<n;i++){
            var pt = read_point(message, i, buffer);
            that.geom.vertices[i] = new THREE.Vector3( pt['x'], pt['y'], pt['z'] );
            that.attribs.customColor.value[ i ] = new THREE.Color( pt['rgb'] );
            that.attribs.alpha.value[i] = 1.0;
        }
        for(i=n; i<that.prev_pts; i++){
            that.attribs.alpha.value[i] = 0.0;
        }
        that.prev_pts = n;

        that.geom.verticesNeedUpdate = true;
        that.attribs.customColor.needsUpdate = true;
        that.attribs.alpha.needsUpdate = true;
    });


};
ROS3D.PointCloud2.prototype.__proto__ = THREE.Object3D.prototype;
