/**
 * @author Jose Rojas - jrojas@redlinesolutions.co
 */

 /**
  * MeshLoader is a singleton factory class for using various helper classes to
  * load mesh files of different types.
  *
  * It consists of one dictionary property 'loaders'. The dictionary keys consist
  * of the file extension for each supported loader type. The dictionary values
  * are functions used to construct the loader objects. The functions have the
  * following parameters:
  *
  *  * meshRes - the MeshResource that will contain the loaded mesh
  *  * uri - the uri path to the mesh file
  *  @returns loader object
  */
ROS3D.MeshLoader = {
   loaders: {
     'dae': function(meshRes, uri, options) {
       const material = options.material;
       const loader = new THREE.ColladaLoader();
       loader.log = function(message) {
         if (meshRes.warnings) {
           console.warn(message);
         }
       };
       loader.load(
         uri,
         function colladaReady(collada) {
           // check for a scale factor in ColladaLoader2
           // add a texture to anything that is missing one
           if(material !== null) {
             collada.scene.traverse(function(child) {
               if(child instanceof THREE.Mesh) {
                 if(child.material === undefined) {
                   child.material = material;
                 }
               }
             });
           }

           meshRes.add(collada.scene);
         },
         /*onProgress=*/null,
         function onLoadError(error) {
           console.error(error);
         });
         return loader;
     },
     
     'stl': function(meshRes, uri, options) {
       const material = options.material;
       const loader = new THREE.STLLoader();
       {
         loader.load(uri,
                     function ( geometry ) {
                       geometry.computeFaceNormals();
                       var mesh;
                       if(material !== null) {
                         mesh = new THREE.Mesh( geometry, material );
                       } else {
                         mesh = new THREE.Mesh( geometry,
                                                new THREE.MeshBasicMaterial( { color: 0x999999 } ) );
                       }
                       meshRes.add(mesh);
                     },
                     /*onProgress=*/null,
                     function onLoadError(error) {
                       console.error(error);
                     });
       }
       return loader;
     }

   }
 }
