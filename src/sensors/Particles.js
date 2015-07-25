/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/**
 * A set of particles. Used by PointCloud2.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * tfClient - the TF client handle to use
 *  * texture - (optional) Image url for a texture to use for the points. Defaults to a single white pixel.
 *  * rootObject (optional) - the root object to add this marker to
 *  * size (optional) - size to draw each point (default 0.05)
 *  * max_pts (optional) - number of points to draw (default 100)
 */
ROS3D.Particles = function(options) {
  options = options || {};
  this.tfClient = options.tfClient;
  var texture = options.texture || 'https://upload.wikimedia.org/wikipedia/commons/a/a2/Pixel-white.png';
  var size = options.size || 0.05;
  this.max_pts = options.max_pts || 10000;
  this.first_size = null;
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
    for(var i=0;i<this.max_pts;i++){
        this.geom.vertices.push(new THREE.Vector3( ));
    }

    var customUniforms =
    {
        texture:   { type: 't', value: THREE.ImageUtils.loadTexture( texture ) },
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

    this.points = this.geom.vertices;
    this.colors = this.attribs.customColor.value;
    this.alpha =  this.attribs.alpha.value;

};

function setFrame(particles, frame)
{
    if(particles.sn===null){
        particles.sn = new ROS3D.SceneNode({
            frameID : frame,
            tfClient : particles.tfClient,
            object : particles.ps
        });

        particles.rootObject.add(particles.sn);
    }
}

function finishedUpdate(particles, n)
{
    if(particles.first_size === null){
        particles.first_size = n;
        particles.max_pts = Math.max(particles.max_pts, n);
    }

    for(var i=n; i<particles.prev_pts; i++){
        particles.alpha[i] = 0.0;
    }
    particles.prev_pts = n;

    particles.geom.verticesNeedUpdate = true;
    particles.attribs.customColor.needsUpdate = true;
    particles.attribs.alpha.needsUpdate = true;

    if(n>particles.max_pts){
        throw 'Attempted to draw more points than max_pts allows';
    }
}
