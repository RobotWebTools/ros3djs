/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
 * @author Jihoon Lee (jihoonlee.in@gmail.com)
 */

/**
 * A Viewer can be used to render an interactive 3D scene to a HTML5 canvas.
 *
 * @constructor
 * @param options - object with following keys:
 * * divID - the ID of the div to place the viewer in
 * * width - the initial width, in pixels, of the canvas
 * * height - the initial height, in pixels, of the canvas
 * * disableGrid - if the grid feature should be disabled in the viewer
 * * gridColor - the color to render the grid lines, like #cccccc
 * * background - the color to render the background, like #efefef
 * * antialias - if antialiasing should be used
 */
ROS3D.Viewer = function(options) {
  var viewer = this;
  options = options || {};
  this.divID = options.divID;
  this.width = options.width;
  this.height = options.height;
  this.disableGrid = options.disableGrid;
  this.gridColor = options.gridColor || '#cccccc';
  this.background = options.background || '#111111';
  this.antialias = options.antialias;

  // create the canvas to render to
  this.renderer = new THREE.WebGLRenderer({
    antialias : this.antialias
  });
  this.renderer.setClearColorHex(this.background.replace('#', '0x'), 1.0);
  this.renderer.sortObjects = false;
  this.renderer.setSize(this.width, this.height);
  this.renderer.shadowMapEnabled = false;
  this.renderer.autoClear = false;

  // create the global scene
  this.scene = new THREE.Scene();

  // create the global camera
  this.camera = new THREE.PerspectiveCamera(40, this.width / this.height, 0.01, 1000);
  this.camera.position.x = 3;
  this.camera.position.y = 3;
  this.camera.position.z = 3;
  // add controls to the camera
  this.cameraControls = new ROS3D.OrbitControls(this.scene, this.camera);
  this.cameraControls.userZoomSpeed = 0.5;

  // create a grid
  if (!this.disableGrid) {
    // 50 cells
    var gridGeom = new THREE.PlaneGeometry(50, 50, 50, 50);
    var gridMaterial = new THREE.MeshBasicMaterial({
      color : this.gridColor,
      wireframe : true,
      wireframeLinewidth : 1,
      transparent : true
    });
    var gridObj = new THREE.Mesh(gridGeom, gridMaterial);
    this.scene.add(gridObj);
  }

  // lights
  this.scene.add(new THREE.AmbientLight(0x555555));
  this.directionalLight = new THREE.DirectionalLight(0xffffff);
  this.scene.add(this.directionalLight);

  // propagates mouse events to three.js objects
  this.selectableObjs = new THREE.Object3D;
  this.scene.add(this.selectableObjs);
  var mouseHandler = new ROS3D.MouseHandler(this.renderer, this.camera, this.selectableObjs,
      this.cameraControls);

  // highlights the receiver of mouse events
  this.highlighter = new ROS3D.Highlighter(mouseHandler);

  /**
   * Renders the associated scene to the viewer.
   */
  function draw() {
    // update the controls
    viewer.cameraControls.update();

    // put light to the top-left of the camera
    viewer.directionalLight.position = viewer.camera.localToWorld(new THREE.Vector3(-1, 1, 0));
    viewer.directionalLight.position.normalize();

    // set the scene
    viewer.renderer.clear(true, true, true);
    viewer.renderer.render(viewer.scene, viewer.camera);

    // reder any mouseovers
    viewer.highlighter.renderHighlight(viewer.renderer, viewer.scene, viewer.camera);

    // draw the frame
    requestAnimationFrame(draw);
  };
  
  /**
   * Add the given THREE Object3D to the global scene in the viewer.
   * 
   * @param object - the THREE Object3D to add
   */
  this.addObject = function(object) {
    viewer.scene.add(object);
  };

  // add the renderer to the page
  document.getElementById(this.divID).appendChild(this.renderer.domElement);

  // begin the animation
  draw();
};
