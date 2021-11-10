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
   onError: function(error) {
     console.error(error);
   },
   loaders: {
     'dae': function(meshRes, uri, options) {
       const material = options.material;
       const loader = new THREE.ColladaLoader(options.loader);
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
         ROS3D.MeshLoader.onError);
         return loader;
     },

     'obj': function(meshRes, uri, options) {
       const material = options.material;
       const loader = new THREE.OBJLoader(options.loader);
       loader.log = function(message) {
         if (meshRes.warnings) {
           console.warn(message);
         }
       };

       //Reload the mesh again after materials have been loaded
       // @todo: this should be improved so that the file doesn't need to be
       // reloaded however that would involve more changes within the OBJLoader.
       function onMaterialsLoaded(loader, materials) {
         loader.
         setMaterials(materials).
         load(
           uri,
           function OBJMaterialsReady(obj) {
             // add the container group
             meshRes.add(obj);
           },
           null,
           ROS3D.MeshLoader.onError);
       }

       loader.load(
         uri,
         function OBJFileReady(obj) {

           const baseUri = THREE.LoaderUtils.extractUrlBase( uri );

           if (obj.materialLibraries.length) {
             // load the material libraries
             const materialUri = obj.materialLibraries[0];
             new THREE.MTLLoader(options.loader).setPath(baseUri).load(
               materialUri,
               function(materials) {
                  materials.preload();
                  onMaterialsLoaded(loader, materials);
               },
               null,
               ROS3D.MeshLoader.onError
             );
           } else {
             // add the container group
             meshRes.add(obj);
           }

         },
         /*onProgress=*/null,
         ROS3D.MeshLoader.onError
         );
         return loader;
     },

     'stl': function(meshRes, uri, options) {
       const material = options.material;
       const loader = new THREE.STLLoader(options.loader);
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
                     ROS3D.MeshLoader.onError);
       }
       return loader;
     }

   }
 };
