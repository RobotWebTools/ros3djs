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
 *  * path (optional) - the base path to the associated models that will be loaded
 *  * resource - the resource file name to load
 *  * material (optional) - the material to use for the object
 *  * warnings (optional) - if warnings should be printed
 */
ROS3D.MeshResource = function(options) {
  var that = this;
  options = options || {};
  var path = options.path || '/';
  var resource = options.resource;
  var material = options.material || null;
  this.warnings = options.warnings;

  THREE.Object3D.call(this);

  // check for a trailing '/'
  if (path.substr(path.length - 1) !== '/') {
    this.path += '/';
  }

  var uri = path + resource;
  var fileType = uri.substr(-4).toLowerCase();

  // check the type
  if (uri.substr(-4).toLowerCase() === '.dae') {
    var loader = new ColladaLoader2();
    loader.log = function(message) {
      if (that.warnings) {
        console.warn(message);
      }
    };
    loader.load(uri, function colladaReady(collada) {
      // check for a scale factor
      if(collada.dae.asset.unit) {
        var scale = collada.dae.asset.unit;
        collada.scene.scale = new THREE.Vector3(scale, scale, scale);
      }

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
  }
};
ROS3D.MeshResource.prototype.__proto__ = THREE.Object3D.prototype;
