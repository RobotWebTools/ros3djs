/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A MeshResource is an THREE object that will load from a external mesh file. Currently loads
 * Collada files.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * path (optional) - the base path to the associated models that will be loaded
 *  * resource - the resource file name to load
 *  * material (optional) - the material to use for the object
 *  * warnings (optional) - if warnings should be printed
 *  * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                        ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.MeshResource = function(options) {
  var that = this;
  options = options || {};
  var path = options.path || '/';
  var resource = options.resource;
  var material = options.material || null;
  this.warnings = options.warnings;
  var loaderType = options.loader || ROS3D.COLLADA_LOADER_2;

  THREE.Object3D.call(this);

  // check for a trailing '/'
  if (path.substr(path.length - 1) !== '/') {
    this.path += '/';
  }

  var uri = path + resource;
  var fileType = uri.substr(-4).toLowerCase();

  // check the type
  var loader;
  if (fileType === '.dae') {
    if (loaderType ===  ROS3D.COLLADA_LOADER) {
      loader = new THREE.ColladaLoader();
    } else {
      loader = new ColladaLoader2();
    }
    loader.log = function(message) {
      if (that.warnings) {
        console.warn(message);
      }
    };
    loader.load(uri, function colladaReady(collada) {
      // check for a scale factor in ColladaLoader2
      if(loaderType === ROS3D.COLLADA_LOADER_2 && collada.dae.asset.unit) {
        var scale = collada.dae.asset.unit;
        collada.scene.scale.set(scale, scale, scale);
      }

      // add a texture to anything that is missing one
      if(material !== null) {
        var setMaterial = function(node, material) {
          node.material = material;
          if (node.children) {
            for (var i = 0; i < node.children.length; i++) {
              setMaterial(node.children[i], material);
            }
          }
        };

        setMaterial(collada.scene, material);
      }

      that.add(collada.scene);
    });
  } else if (fileType === '.stl') {
    loader = new THREE.STLLoader();
    {
      loader.load(uri, function ( geometry ) {
        geometry.computeFaceNormals();
        var mesh;
        if(material !== null) {
          mesh = new THREE.Mesh( geometry, material );
        } else {
          mesh = new THREE.Mesh( geometry, new THREE.MeshBasicMaterial( { color: 0x999999 } ) );
        }
        that.add(mesh);
      } );
    }
  }
};
ROS3D.MeshResource.prototype.__proto__ = THREE.Object3D.prototype;
