/**
 * @author Julius Kammerl - jkammerl@willowgarage.com
 */

/**
 * The DepthCloud object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * url - the URL of the stream
 *   * f (optional) - the camera's focal length (defaults to standard Kinect calibration)
 *   * pointSize (optional) - point size (pixels) for rendered point cloud
 *   * width (optional) - width of the depthcloud encoded video stream
 *   * height (optional) - height of the depthcloud encoded video stream
 *   * whiteness (optional) - blends rgb values to white (0..100)
 *   * varianceThreshold (optional) - threshold for variance filter, used for compression artifact removal
 */
ROS3D.DepthCloud = function(options) {
  THREE.Object3D.call(this);
  this.type = 'DepthCloud';

  this.options = options || {};
  this.url = options.url;
  this.f = options.f || 526;
  this.pointSize = options.pointSize || 3;
  this.width = options.width || 1024;
  this.height = options.height || 1024;
  this.whiteness = options.whiteness || 0;
  this.varianceThreshold = options.varianceThreshold || 0.000016667;

  var metaLoaded = false;
  this.video = document.createElement('video');
  this.video.width = this.width;
  this.video.height = this.height;
  this.video.addEventListener('loadedmetadata', this.metaLoaded.bind(this), false);

  this.video.loop = true;
  this.video.src = this.url;
  this.video.crossOrigin = 'Anonymous';
  this.video.setAttribute('crossorigin', 'Anonymous');

  this.intervalCallback = null;
  this.stopCloud = function() {
  this.video.pause();
  this.video.src = undefined; // forcefully silence the video streaming url.
  clearInterval(this.intervalCallback);
  };

  // define custom shaders
  this.vertex_shader = [
    'uniform sampler2D map;',
    '',
    'uniform float width;',
    'uniform float height;',
    '',
    'uniform float pointSize;',
    'uniform float zOffset;',
    '',
    'uniform float focallength;',
    '',
    'varying vec2 vUvP;',
    'varying vec2 colorP;',
    '',
    'varying float depthVariance;',
    'varying float maskVal;',
    '',
    'float sampleDepth(vec2 pos)',
    '  {',
    '    float depth;',
    '    ',
    '    vec2 vUv = vec2( pos.x / (width*2.0), pos.y / (height*2.0)+0.5 );',
    '    vec2 vUv2 = vec2( pos.x / (width*2.0)+0.5, pos.y / (height*2.0)+0.5 );',
    '    ',
    '    vec4 depthColor = texture2D( map, vUv );',
    '    ',
    '    depth = ( depthColor.r + depthColor.g + depthColor.b ) / 3.0;',
    '    ',
    '    if (depth > (1.0 - 3.0/255.0) )', // If we're closer than 3 values from saturation, check the next depth image
    '    {',
    '      vec4 depthColor2 = texture2D( map, vUv2 );',
    '      float depth2 = ( depthColor2.r + depthColor2.g + depthColor2.b ) / 3.0 ;',
    '      depth += depth2;',
    '    }',
    '    ',
    '    return depth;',
    '  }',
    '',
    'float median(float a, float b, float c)',
    '  {',
    '    float r=a;',
    '    ',
    '    if ( (a<b) && (b<c) )',
    '    {',
    '      r = b;',
    '    }',
    '    if ( (a<c) && (c<b) )',
    '    {',
    '      r = c;',
    '    }',
    '    return r;',
    '  }',
    '',
    'float variance(float d1, float d2, float d3, float d4, float d5, float d6, float d7, float d8, float d9)',
    '  {',
    '    float mean = (d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8 + d9) / 9.0;',
    '    float t1 = (d1-mean);',
    '    float t2 = (d2-mean);',
    '    float t3 = (d3-mean);',
    '    float t4 = (d4-mean);',
    '    float t5 = (d5-mean);',
    '    float t6 = (d6-mean);',
    '    float t7 = (d7-mean);',
    '    float t8 = (d8-mean);',
    '    float t9 = (d9-mean);',
    '    float v = (t1*t1+t2*t2+t3*t3+t4*t4+t5*t5+t6*t6+t7*t7+t8*t8+t9*t9)/9.0;',
    '    return v;',
    '  }',
    '',
    'vec2 decodeDepth(vec2 pos)',
    '  {',
    '    vec2 ret;',
    '    ',
    '    ',
    '    float depth1 = sampleDepth(vec2(position.x-1.0, position.y-1.0));',
    '    float depth2 = sampleDepth(vec2(position.x, position.y-1.0));',
    '    float depth3 = sampleDepth(vec2(position.x+1.0, position.y-1.0));',
    '    float depth4 = sampleDepth(vec2(position.x-1.0, position.y));',
    '    float depth5 = sampleDepth(vec2(position.x, position.y));',
    '    float depth6 = sampleDepth(vec2(position.x+1.0, position.y));',
    '    float depth7 = sampleDepth(vec2(position.x-1.0, position.y+1.0));',
    '    float depth8 = sampleDepth(vec2(position.x, position.y+1.0));',
    '    float depth9 = sampleDepth(vec2(position.x+1.0, position.y+1.0));',
    '    ',
    '    float median1 = median(depth1, depth2, depth3);',
    '    float median2 = median(depth4, depth5, depth6);',
    '    float median3 = median(depth7, depth8, depth9);',
    '    ',
    '    ret.x = median(median1, median2, median3);',
    '    ret.y = variance(depth1, depth2, depth3, depth4, depth5, depth6, depth7, depth8, depth9);',
    '    ',
    '    return ret;',
    '    ',
    '  }',
    '',
    '',
    'void main() {',
    '  ',
    '  vUvP = vec2( position.x / (width*2.0), position.y / (height*2.0)+0.5 );',
    '  colorP = vec2( position.x / (width*2.0)+0.5 , position.y / (height*2.0)  );',
    '  ',
    '  vec4 pos = vec4(0.0,0.0,0.0,0.0);',
    '  depthVariance = 0.0;',
    '  ',
    '  if ( (vUvP.x<0.0)|| (vUvP.x>0.5) || (vUvP.y<0.5) || (vUvP.y>0.0))',
    '  {',
    '    vec2 smp = decodeDepth(vec2(position.x, position.y));',
    '    float depth = smp.x;',
    '    depthVariance = smp.y;',
    '    ',
    '    float z = -depth;',
    '    ',
    '    pos = vec4(',
    '      ( position.x / width - 0.5 ) * z *0.5 * (1000.0/focallength) * -1.0,',
    '      ( position.y / height - 0.5 ) * z *0.5 * (1000.0/focallength),',
    '      (- z + zOffset / 1000.0) * 1.0,',
    '      1.0);',
    '    ',
    '    vec2 maskP = vec2( position.x / (width*2.0), position.y / (height*2.0)  );',
    '    vec4 maskColor = texture2D( map, maskP );',
    '    maskVal = ( maskColor.r + maskColor.g + maskColor.b ) / 3.0 ;',
    '  }',
    '  ',
    '  gl_PointSize = pointSize;',
    '  gl_Position = projectionMatrix * modelViewMatrix * pos;',
    '  ',
    '}'
    ].join('\n');

  this.fragment_shader = [
    'uniform sampler2D map;',
    'uniform float varianceThreshold;',
    'uniform float whiteness;',
    '',
    'varying vec2 vUvP;',
    'varying vec2 colorP;',
    '',
    'varying float depthVariance;',
    'varying float maskVal;',
    '',
    '',
    'void main() {',
    '  ',
    '  vec4 color;',
    '  ',
    '  if ( (depthVariance>varianceThreshold) || (maskVal>0.5) ||(vUvP.x<0.0)|| (vUvP.x>0.5) || (vUvP.y<0.5) || (vUvP.y>1.0))',
    '  {  ',
    '    discard;',
    '  }',
    '  else ',
    '  {',
    '    color = texture2D( map, colorP );',
    '    ',
    '    float fader = whiteness /100.0;',
    '    ',
    '    color.r = color.r * (1.0-fader)+ fader;',
    '    ',
    '    color.g = color.g * (1.0-fader)+ fader;',
    '    ',
    '    color.b = color.b * (1.0-fader)+ fader;',
    '    ',
    '    color.a = 1.0;//smoothstep( 20000.0, -20000.0, gl_FragCoord.z / gl_FragCoord.w );',
    '  }',
    '  ',
    '  gl_FragColor = vec4( color.r, color.g, color.b, color.a );',
    '  ',
    '}'
    ].join('\n');
};
ROS3D.DepthCloud.prototype = Object.create( THREE.Object3D.prototype );
ROS3D.DepthCloud.prototype.constructor = ROS3D.DepthCloud;

ROS3D.DepthCloud.prototype.clone = function ( object, recursive ) {

    if ( object === undefined ) { object = new ROS3D.DepthCloud( this.options ); }

    THREE.Object3D.prototype.clone.call( this, object, recursive );

    return object;

};

/**
 * Callback called when video metadata is ready
 */
ROS3D.DepthCloud.prototype.metaLoaded = function() {
  this.metaLoaded = true;
  this.initStreamer();
};

/**
 * Callback called when video metadata is ready
 */
ROS3D.DepthCloud.prototype.initStreamer = function() {

  if (this.metaLoaded) {
    this.dctexture = new THREE.Texture(this.video);
    this.dcgeometry = new THREE.Geometry();

    var qwidth = this.width / 2.0;
    var qheight = this.height / 2.0;
    // the number of points is a forth of the total image size
    for (var i = 0, l = qwidth * qheight; i < l; i++) {

      var vertex = new THREE.Vector3();
      vertex.x = (i % qwidth);
      vertex.y = Math.floor(i / qwidth);

      this.dcgeometry.vertices.push(vertex);
    }

    this.dcmaterial = new THREE.ShaderMaterial({
      uniforms : {
        'map' : {
          type : 't',
          value : this.dctexture
        },
        'width' : {
          type : 'f',
          value : qwidth
        },
        'height' : {
          type : 'f',
          value : qheight
        },
        'focallength' : {
          type : 'f',
          value : this.f
        },
        'pointSize' : {
          type : 'f',
          value : this.pointSize
        },
        'zOffset' : {
          type : 'f',
          value : 0
        },
        'whiteness' : {
          type : 'f',
          value : this.whiteness
        },
        'varianceThreshold' : {
          type : 'f',
          value : this.varianceThreshold
        }
      },
      vertexShader : this.vertex_shader,
      fragmentShader : this.fragment_shader
    });
    this.dcmaterial.color = new THREE.Color( 0xffffff );
    this.mesh = new THREE.PointCloud(this.dcgeometry, this.dcmaterial);
    this.mesh.frustumCulled = false;
    this.mesh.position.set(0,0,0);

    this.add(this.mesh);

    var that = this;
    this.intervalCallback = setInterval(function() {
      if (that.video.readyState === that.video.HAVE_ENOUGH_DATA) {
        that.dctexture.needsUpdate = true;
      }
    }, 1000 / 30);
  }
};

/**
 * Start video playback
 */
ROS3D.DepthCloud.prototype.startStream = function() {
  this.video.play();
};

/**
 * Stop video playback
 */
ROS3D.DepthCloud.prototype.stopStream = function() {
  this.video.pause();
};
