ROS3D.MeshLoader = function(meshBaseUrl) {

  var meshLoader = this;
  if (meshBaseUrl !== undefined) {
    this.meshBaseUrl = meshBaseUrl;

    if (this.meshBaseUrl.substr(this.meshBaseUrl.length - 1) != "/") {
      this.meshBaseUrl = this.meshBaseUrl + "/";
    }
  }

  meshLoader.getMaterial = function(material) {

    var texture_uri;
    var texture;
    var geomaterial;
    if (material !== undefined) {
      if (material.texture_filename !== "") {
        texture_uri = meshBaseUrl + material.texture_filename.substr(10);
        texture = new THREE.ImageUtils.loadTexture(texture_uri);
        geomaterial = new THREE.MeshBasicMaterial({
          map : texture
        });
      } else {
        var color_name = material.name.toLowerCase();
        geomaterial = new THREE.MeshBasicMaterial({
          color : THREE.ColorKeywords[color_name]
        });
      }
    } else {
      geomaterial = new THREE.MeshBasicMaterial();
    }

    return geomaterial;
  };

  meshLoader.load = function(resource, material) {
    var objroot = new THREE.Object3D();

    if (meshBaseUrl == undefined) {
      console.log('test');
      THREE.Mesh
          .call(this, new THREE.CubeGeometry(0.01, 0.01, 0.01), new THREE.MeshBasicMaterial());
    } else {

      var loader;

      var uri = meshBaseUrl + resource.substr(10);
      var texture_uri;
      var texture;
      var geomaterial;

      // Collision model visualization support. This is not necessary for now.
      // geomaterial = this.getMaterial(material); 

      if (uri.substr(-4).toLowerCase() == ".dae") {
        loader = new ROS3D.ColladaLoader();
        loader.load(uri, function colladaReady(collada) {
          var sceneObj = collada.scene;
          //            sceneObj.children[0].material = geomaterial;

          objroot.add(sceneObj);
        });
      }
    }
    return objroot;
  };
};
