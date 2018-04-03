/**
 * @author David V. Lu!! - davidvlu@gmail.com
 * @author Mathieu Bredif - mathieu.bredif@ign.fr
 */

/**
 * A set of particles. Used by PointCloud2.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * tfClient - the TF client handle to use
 *  * rootObject (optional) - the root object to add this marker to use for the points.
 *  * max_pts (optional) - number of points to draw (default: 10000)
 *  * pointRatio (optional) - point subsampling ratio (default: 1, no subsampling)
 *  * messageRatio (optional) - message subsampling ratio (default: 1, no subsampling)
 *  * material (optional) - a material object or an option to construct a PointsMaterial.
 *  * colorsrc (optional) - the field to be used for coloring (default: 'rgb')
 *  * colormap (optional) - function that turns the colorsrc field value to a color
 */
ROS3D.Particles = function(options) {
  options = options || {};
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.max_pts = options.max_pts || 10000;
  this.pointRatio = options.pointRatio || 1;
  this.messageRatio = options.messageRatio || 1;
  this.messageCount = 0;
  this.material = options.material || {};
  this.colorsrc = options.colorsrc;
  this.colormap = options.colormap;
  THREE.Object3D.call(this);

  this.sn = null;
  this.buffer = null;
};

ROS3D.Particles.prototype.setup = function(frame, point_step, fields)
{
    if(this.sn===null){
        // scratch space to decode base64 buffers
        if(point_step) {
            this.buffer = new Uint8Array( this.max_pts * point_step );
        }
        // turn fields to a map
        fields = fields || [];
        this.fields = {};
        for(var i=0; i<fields.length; i++) {
            this.fields[fields[i].name] = fields[i];
        }
        this.geom = new THREE.BufferGeometry();

        this.point = new THREE.BufferAttribute( new Float32Array( this.max_pts * 3), 3, false );
        this.geom.addAttribute( 'position', this.point.setDynamic(true) );

        if(!this.colorsrc && this.fields.rgb) {
            this.colorsrc = 'rgb';
        }
        if(this.colorsrc) {
            var field = this.fields[this.colorsrc];
            if (field) {
                this.color = new THREE.BufferAttribute( new Float32Array( this.max_pts * 3), 3, false );
                this.geom.addAttribute( 'color', this.color.setDynamic(true) );
                var offset = field.offset;
                this.getColor = [
                    function(dv,base,le){return dv.getInt8(base+offset,le);},
                    function(dv,base,le){return dv.getUint8(base+offset,le);},
                    function(dv,base,le){return dv.getInt16(base+offset,le);},
                    function(dv,base,le){return dv.getUint16(base+offset,le);},
                    function(dv,base,le){return dv.getInt32(base+offset,le);},
                    function(dv,base,le){return dv.getUint32(base+offset,le);},
                    function(dv,base,le){return dv.getFloat32(base+offset,le);},
                    function(dv,base,le){return dv.getFloat64(base+offset,le);}
                ][field.datatype-1];
                this.colormap = this.colormap || function(x){return new THREE.Color(x);};
            } else {
                console.warn('unavailable field "' + this.colorsrc + '" for coloring.');
            }
        }

        if(!this.material.isMaterial) { // if it is an option, apply defaults and pass it to a PointsMaterial
            if(this.color && this.material.vertexColors === undefined) {
                this.material.vertexColors = THREE.VertexColors;
            }
            this.material = new THREE.PointsMaterial(this.material);      
        }

        this.ps = new THREE.Points( this.geom, this.material );

        this.sn = new ROS3D.SceneNode({
            frameID : frame,
            tfClient : this.tfClient,
            object : this.ps
        });

        this.rootObject.add(this.sn);
    }
    return (this.messageCount++ % this.messageRatio) === 0;
};

ROS3D.Particles.prototype.update = function(n)
{
  this.geom.setDrawRange(0,n);

  this.point.needsUpdate = true;
  this.point.updateRange.count = n * this.point.itemSize;

  if (this.color) {
    this.color.needsUpdate = true;
    this.color.updateRange.count = n * this.color.itemSize;
  }
};
