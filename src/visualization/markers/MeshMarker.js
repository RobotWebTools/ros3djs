ROS3D.MeshMarker = function(markerMsg, meshBaseUrl, overrideMaterial) {

  if (meshBaseUrl == undefined) {
    THREE.Mesh.call(this, new THREE.CubeGeometry(0.01, 0.01, 0.01), new THREE.MeshBasicMaterial());
  } else {
    THREE.Mesh.call(this, new THREE.CubeGeometry(0.01, 0.01, 0.01), new THREE.MeshBasicMaterial());
    // THREE.Object3D.call(this);

    var loader = new THREE.ColladaLoader(overrideMaterial);
    var url = meshBaseUrl + markerMsg.mesh_resource.substr(10);

    var that = this;

    loader.load(url, function colladaReady(collada) {
      var sceneObj = collada.scene;
      // sceneObj.children[0].material = new THREE.MeshLambertMaterial({
      // color:0x888888
      // });
      that.add(sceneObj);
    });
  }
};

ROS3D.MeshMarker.prototype = Object.create(THREE.Mesh.prototype);