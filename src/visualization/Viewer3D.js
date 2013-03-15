(function (root, factory) {
  if(typeof define ==='function' && define.amd) {
    define(['three_bundle','threeinteraction'],factory);
  }
  else {
    root.Viewer3D = factory(root.THREE,root.ThreeInteraction);
  }
}(this, function(THREE,ThreeInteraction) {
  var Viewer3D = function(container) {

    var viewer3d = this;
    var camera, cameraControls, scene, renderer;
    var selectableObjs;
    var directionalLight;
    var mouseHandler, highlighter;
    var imClient, imViewer;
    
    viewer3d.init = function() {
      // setup camera
      camera = new THREE.PerspectiveCamera(40,window.innerWidth/ window.innerHeight, 0.01,1000);
      camera.position.x = 3;
      camera.position.y = 3;
      camera.position.z = 3;

      // setup scene
      scene = new THREE.Scene();

      // setup camera mouse control
      cameraControls = new THREE.RosOrbitControls(scene,camera);
//      cameraControls.userZoomSpeed = 0.5;

      // add node to host selectable objects
      selectableObjs = new THREE.Object3D();
      scene.add(selectableObjs);

      // add lights
      scene.add(new THREE.AmbientLight(0x555555));
      directionalLight = new THREE.DirectionalLight(0xffffff);
      scene.add(directionalLight);


      // add x/y grid
      var numCells = 50;
      var gridGeom = new THREE.PlaneGeometry(numCells,numCells,numCells,numCells);
      var gridMaterial = new THREE.MeshBasicMaterial({
        color : 0x999999,
        wireframe : true,
        wireframeLinewidth: 1,
        transparent : true
      });
      var gridObj = new THREE.Mesh(gridGeom,gridMaterial);
      scene.add(gridObj);

      renderer = new THREE.WebGLRenderer({
        antialias : false,
      });
      renderer.setClearColorHex(0x333333,1.0);
      renderer.sortObjects = false;
//      renderer.setSize(window.innerWidth, window.innerHeight);
      renderer.shadowMapEnabled = false;
      renderer.autoClear = false;
      container.appendChild(renderer.domElement);

      // propagate mouse events to three.js objects
      mouseHandler = new ThreeInteraction.MouseHandler(renderer,camera, selectableObjs,cameraControls);

      // highlights the receiver of mouse events
      highlighter = new ThreeInteraction.Highlighter(mouseHandler);
    };

    viewer3d.animate = function() {
      cameraControls.update();
      
      // put light to the top-left of the camera
      directionalLight.position = camera.localToWorld(new THREE.Vector3(-1,1,0));
      directionalLight.position.normalize();

      renderer.clear(true,true,true);
      renderer.render(scene,camera);

      highlighter.renderHighlight(renderer,scene,camera);

      requestAnimationFrame(viewer3d.animate);
    };

    viewer3d.resize = function(width,height)
    {
      renderer.setSize(width,height);
    };

    viewer3d.addObject = function(obj)
    {
//      selectableObjs.add(obj);
      scene.add(obj);
    };
  };
  return Viewer3D;
}));

