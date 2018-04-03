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

    this.geom = new THREE.Geometry();
    for(var i=0;i<this.max_pts;i++){
        this.geom.vertices.push(new THREE.Vector3( ));
        this.geom.colors.push( new THREE.Color( options.color ) );
    }

    this.attribs =
    {
        customColor:   { type: 'c', value: [] },
        alpha:         { type: 'f', value: [] }
    };

    this.pointsMaterial = new THREE.PointsMaterial(
    {
        size : size,
        vertexColors : THREE.VertexColors
    });

    this.ps = new THREE.Points( this.geom, this.pointsMaterial );
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
        console.error('Attempted to draw more points than max_pts allows');
    }
}
