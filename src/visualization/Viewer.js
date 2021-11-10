/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 */

/**
 * A Viewer can be used to render an interactive 3D scene to a HTML5 canvas.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * divID - the ID of the div to place the viewer in
 *  * elem - the elem to place the viewer in (overrides divID if provided)
 *  * width - the initial width, in pixels, of the canvas
 *  * height - the initial height, in pixels, of the canvas
 *  * background (optional) - the color to render the background, like '#efefef'
 *  * alpha (optional) - the alpha of the background
 *  * antialias (optional) - if antialiasing should be used
 *  * intensity (optional) - the lighting intensity setting to use
 *  * cameraPosition (optional) - the starting position of the camera
 *  * displayPanAndZoomFrame (optional) - whether to display a frame when
 *  *                                     panning/zooming. Defaults to true.
 *  * lineTypePanAndZoomFrame - line type for the frame that is displayed when
 *  *                           panning/zooming. Only has effect when
 *  *                           displayPanAndZoomFrame is set to true.
 */
ROS3D.Viewer = function(options) {
  options = options || {};
  var divID = options.divID;
  var elem = options.elem;
  var width = options.width;
  var height = options.height;
  var background = options.background || '#111111';
  var antialias = options.antialias;
  var intensity = options.intensity || 0.66;
  var near = options.near || 0.01;
  var far = options.far || 1000;
  var alpha = options.alpha || 1.0;
  var cameraPosition = options.cameraPose || {
    x : 3,
    y : 3,
    z : 3
  };
  var cameraZoomSpeed = options.cameraZoomSpeed || 0.5;
  var displayPanAndZoomFrame = (options.displayPanAndZoomFrame === undefined) ? true : !!options.displayPanAndZoomFrame;
  var lineTypePanAndZoomFrame = options.lineTypePanAndZoomFrame || 'full';

  // create the canvas to render to
  this.renderer = new THREE.WebGLRenderer({
    antialias : antialias,
    alpha: true
  });
  this.renderer.setClearColor(parseInt(background.replace('#', '0x'), 16), alpha);
  this.renderer.sortObjects = false;
  this.renderer.setSize(width, height);
  this.renderer.shadowMap.enabled = false;
  this.renderer.autoClear = false;

  // create the global scene
  this.scene = new THREE.Scene();

  // create the global camera
  this.camera = new THREE.PerspectiveCamera(40, width / height, near, far);
  this.camera.position.x = cameraPosition.x;
  this.camera.position.y = cameraPosition.y;
  this.camera.position.z = cameraPosition.z;
  // add controls to the camera
  this.cameraControls = new ROS3D.OrbitControls({
    scene : this.scene,
    camera : this.camera,
    displayPanAndZoomFrame : displayPanAndZoomFrame,
    lineTypePanAndZoomFrame: lineTypePanAndZoomFrame
  });
  this.cameraControls.userZoomSpeed = cameraZoomSpeed;

  // lights
  this.scene.add(new THREE.AmbientLight(0x555555));
  this.directionalLight = new THREE.DirectionalLight(0xffffff, intensity);
  this.scene.add(this.directionalLight);

  // propagates mouse events to three.js objects
  this.selectableObjects = new THREE.Group();
  this.scene.add(this.selectableObjects);
  var mouseHandler = new ROS3D.MouseHandler({
    renderer : this.renderer,
    camera : this.camera,
    rootObject : this.selectableObjects,
    fallbackTarget : this.cameraControls
  });

  // highlights the receiver of mouse events
  this.highlighter = new ROS3D.Highlighter({
    mouseHandler : mouseHandler
  });

  this.stopped = true;
  this.animationRequestId = undefined;

  // add the renderer to the page
  var node = elem || document.getElementById(divID);
  node.appendChild(this.renderer.domElement);

  // begin the render loop
  this.start();
};

/**
 *  Start the render loop
 */
ROS3D.Viewer.prototype.start = function(){
  this.stopped = false;
  this.draw();
};

/**
 * Renders the associated scene to the viewer.
 */
ROS3D.Viewer.prototype.draw = function(){
  if(this.stopped){
    // Do nothing if stopped
    return;
  }

  // update the controls
  this.cameraControls.update();

  // put light to the top-left of the camera
  // BUG: position is a read-only property of DirectionalLight,
  // attempting to assign to it either does nothing or throws an error.
  //this.directionalLight.position = this.camera.localToWorld(new THREE.Vector3(-1, 1, 0));
  this.directionalLight.position.normalize();

  // set the scene
  this.renderer.clear(true, true, true);
  this.renderer.render(this.scene, this.camera);
  this.highlighter.renderHighlights(this.scene, this.renderer, this.camera);

  // draw the frame
  this.animationRequestId = requestAnimationFrame(this.draw.bind(this));
};

/**
 *  Stop the render loop
 */
ROS3D.Viewer.prototype.stop = function(){
  if(!this.stopped){
    // Stop animation render loop
    cancelAnimationFrame(this.animationRequestId);
  }
  this.stopped = true;
};

/**
 * Add the given THREE Object3D to the global scene in the viewer.
 *
 * @param object - the THREE Object3D to add
 * @param selectable (optional) - if the object should be added to the selectable list
 */
ROS3D.Viewer.prototype.addObject = function(object, selectable) {
  if (selectable) {
    this.selectableObjects.add(object);
  } else {
    this.scene.add(object);
  }
};

/**
 * Resize 3D viewer
 *
 * @param width - new width value
 * @param height - new height value
 */
ROS3D.Viewer.prototype.resize = function(width, height) {
  this.camera.aspect = width / height;
  this.camera.updateProjectionMatrix();
  this.renderer.setSize(width, height);
};
