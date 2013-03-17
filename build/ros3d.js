/**
* @author Russell Toris - rctoris@wpi.edu
*/

var ROS3D = ROS3D || {
  REVISION : '1'
};
ROS3D.UrdfModel = function(options) {
  
};

var UrdfModel = function() {
  var urdf = this;

  // members
  this.name_ = '';
  this.links_ = {};
  this.joints_ = {};
  this.materials_ = {};

  this.clear = function() {
    this.name_ = '';
    this.links_ = {};
    this.joints_ = {};
    this.materials_ = {};
  };

  this.initFile = function(src, callback) {
    var xhr = new XMLHttpRequest();
    var that = this;
    xhr.onreadystatechange = function() {
      if (xhr.readyState == 4 && (xhr.status == 200 || xhr.status == 0)) {
        window.setTimeout(function() {
          var xml = xhr.responseXML;
          xml.getElementById = function(id) {
            return xpathGetElementById(xml, id);
          };
          that.initXml(xml);

          if (callback) {
            callback(that);
          }

        }, 0);
      }
    };

    xhr.open("GET", src, true);
    xhr.overrideMimeType("text/xml");
    xhr.setRequestHeader("Content-type", "text/xml");
    xhr.send(null);
  };

  this.initXml = function(xml_doc) {
    function nsResolver(prefix) {
      var ns = {
        'c' : 'http://www.collada.org/2005/11/COLLADASchema'
      };
      return ns[prefix] || null;
    };

    function getNode(xpathexpr, ctxNode) {
      if (ctxNode == null)
        ctxNode = xml_doc;
      return xml_doc.evaluate(xpathexpr, ctxNode, null, XPathResult.FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
    }

    var robot_xml = getNode('//robot');
    if (!robot_xml) {
      console.error("Could not find the 'robot' element in the xml file");
      return false;
    }

    this.clear();
    // console.log('Parsing robot xml');

    // Get robot name
    var name = robot_xml.getAttribute('name');
    if (!name) {
      console.error("No name given for the robot.");
      return false;
    }
    this.name_ = name;

    // Get all Material elements
    for (n in robot_xml.childNodes) {
      var node = robot_xml.childNodes[n];
      if (node.tagName != "material")
        continue;
      var material_xml = node;
      var material = new UrdfMaterial();

      if (material.initXml(material_xml)) {
        if (this.getMaterial(material.name)) {
          console.error("material " + material.name + "is not unique.");
          return false;
        } else {
          this.materials_[material.name] = material;
          // console.log('Succesfully added a new material ' + material.name);
        }
      } else {
        console.error('material xml is not initialized correctly');
        return false;
      }
    }

    for (n in robot_xml.childNodes) {
      var node = robot_xml.childNodes[n];
      if (node.tagName != "link")
        continue;
      var link_xml = node;
      var link = new UrdfLink();

      if (link.initXml(link_xml)) {
        if (this.getLink(link.name)) {
          console.error("link " + link.name + " is not unique. ");
          return false;
        } else {
          // console.log("setting link " + link.name + " material");
          if (link.visual) {
            if (link.visual.material_name.length > 0) {
              if (this.getMaterial(link.visual.material_name)) {
                // console.log("Setting link " + link.name + " material to " +
                // link.visual.material_name);
                link.visual.material = this.getMaterial(link.visual.material_name);
              } else {
                if (link.visual.material) {
                  // console.log("link " + link.name + " material " + link.visual.material_name + "
                  // define in Visual.");
                  this.links_[link.visual.material.name] = link.visual.material;
                } else {
                  console.error("link " + link.name + " material " + link.visual.material_name
                      + " undefined.");
                  return false;
                }
              }
            }
          }

          this.links_[link.name] = link;
          // console.log('successfully added a new link ' + link.name);
        }
      } else {
        console.error('link xml is not initialied correctly');
        return false;
      }

      if (this.links_.length == 0) {
        console.error('No link elements found in urdf file');
        return false;
      }
    }

    // Get all Joint elements
    for (n in robot_xml.childNodes) {
      var node = robot_xml.childNodes[n];
      if (node.tagName != 'joint')
        continue;
      var joint_xml = node;
      var joint = new UrdfJoint();

      if (joint.initXml(joint_xml)) {
        if (this.getJoint(joint.name)) {
          console.error('joint ' + joint.name + ' is not unique.');
          return false;
        } else {
          this.joints_[joint.name] = joint;
          // console.log('successfully added a new joint ' + joint.name);
        }
      } else {
        console.error('joint xml is not initialized correctly');
        return false;
      }
    }

    return true;
  };

  this.getMaterial = function(name) {
    return this.materials_[name];
  };

  this.getLink = function(name) {
    return this.links_[name];
  };

  this.getLinks = function(name) {
    return this.links_;
  };

  this.getJoint = function(name) {
    return this.joints_[name];
  };

};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * An Axes object can be used to display the axis of a particular coordinate frame.
 *
 * @constructor
 * @param options - object with following keys:
 * * shaftRadius - the radius of the shaft to render
 * * headRadius - the radius of the head to render
 * * headLength - the length of the head to render
 */
ROS3D.Axes = function(options) {
  var axes = this;
  options = options || {};
  var shaftRadius = options.shaftRadius || 0.008;
  var headRadius = options.headRadius || 0.023;
  var headLength = options.headLength || 0.1;

  THREE.Object3D.call(this);

  // create the cylinders for the objects
  this.lineGeom = new THREE.CylinderGeometry(shaftRadius, shaftRadius, 1.0 - headLength);
  this.headGeom = new THREE.CylinderGeometry(0, headRadius, headLength);

  /**
   * Adds an axis marker to this axes object.
   * 
   * @param axis - the 3D vector representing the axis to add
   */
  function addAxis(axis) {
    // set the color of the axis
    var color = new THREE.Color;
    color.setRGB(axis.x, axis.y, axis.z);
    var material = new THREE.MeshBasicMaterial({
      color : color.getHex()
    });

    // setup the rotation information
    var rotAxis = new THREE.Vector3;
    rotAxis.crossVectors(axis, new THREE.Vector3(0, -1, 0));
    var rot = new THREE.Quaternion;
    rot.setFromAxisAngle(rotAxis, 0.5 * Math.PI);

    // create the arrow
    var arrow = new THREE.Mesh(axes.headGeom, material);
    arrow.position = axis.clone();
    arrow.position.multiplyScalar(0.95);
    arrow.useQuaternion = true;
    arrow.quaternion = rot;
    arrow.updateMatrix();
    axes.add(arrow);

    // create the line
    var line = new THREE.Mesh(axes.lineGeom, material);
    line.position = axis.clone();
    line.position.multiplyScalar(0.45);
    line.useQuaternion = true;
    line.quaternion = rot;
    line.updateMatrix();
    axes.add(line);
  };

  // add the three markers to the axes
  addAxis(new THREE.Vector3(1, 0, 0));
  addAxis(new THREE.Vector3(0, 1, 0));
  addAxis(new THREE.Vector3(0, 0, 1));
};
ROS3D.Axes.prototype = Object.create(THREE.Object3D.prototype);
/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Xueqiao Xu - xueqiaoxu@gmail.com
 * @author Mr.doob - http://mrdoob.com
 * @author AlteredQualia - http://alteredqualia.com
 */

/**
 * Behaves like THREE.OrbitControls, but uses right-handed coordinates and z as up vector.
 *
 * @constructor
 * @param scene - the global scene to use
 * @param object - the object (e.g., camera) to use
 */
ROS3D.OrbitControls = function(scene, object) {
  THREE.EventDispatcher.call(this);
  this.object = object;

  // In ROS, z is pointing upwards
  this.object.up = new THREE.Vector3(0, 0, 1);

  // API
  this.center = new THREE.Vector3();
  this.userZoom = true;
  this.userZoomSpeed = 1.0;
  this.userRotate = true;
  this.userRotateSpeed = 1.0;
  this.autoRotate = false;
  this.autoRotateSpeed = 2.0;

  // internals
  var scope = this;
  var EPS = 0.000001;
  var PIXELS_PER_ROUND = 1800;
  var rotateStart = new THREE.Vector2();
  var rotateEnd = new THREE.Vector2();
  var rotateDelta = new THREE.Vector2();
  var zoomStart = new THREE.Vector2();
  var zoomEnd = new THREE.Vector2();
  var zoomDelta = new THREE.Vector2();
  var moveStartCenter = new THREE.Vector3();
  var moveStartNormal = new THREE.Vector3();
  var moveStartPosition = new THREE.Vector3();
  var moveStartIntersection = new THREE.Vector3();
  var phiDelta = 0;
  var thetaDelta = 0;
  var scale = 1;
  var lastPosition = new THREE.Vector3();
  var STATE = {
    NONE : -1,
    ROTATE : 0,
    ZOOM : 1,
    MOVE : 2
  };
  var state = STATE.NONE;

  /**
   * Get the default, auto rotation angle.
   * 
   * @returns the default, auto rotation angle
   */
  function getAutoRotationAngle() {
    return 2 * Math.PI / 60 / 60 * scope.autoRotateSpeed;
  };

  /**
   * Get the default, auto zoom scale.
   * 
   * @returns the default, auto zoom scale
   */
  function getZoomScale() {
    return Math.pow(0.95, scope.userZoomSpeed);
  };

  /**
   * Intersect the main view plane based on the given information.
   * 
   * @param mouseRay - the ray from the mouse
   * @param planeOrigin - the origin of the plane
   * @param planeNormal - the normal of the plane
   * @returns the intersection
   */
  function intersectViewPlane(mouseRay, planeOrigin, planeNormal) {
    var vector = new THREE.Vector3();
    var intersection = new THREE.Vector3();

    vector.subVectors(planeOrigin, mouseRay.origin);
    dot = mouseRay.direction.dot(planeNormal);

    // bail if ray and plane are parallel
    if (Math.abs(dot) < mouseRay.precision) {
      return null;
    }

    // calc distance to plane
    scalar = planeNormal.dot(vector) / dot;

    intersection = mouseRay.direction.clone().multiplyScalar(scalar);
    return intersection;
  };

  // add the axes for the main coordinate frame
  this.axes = new ROS3D.Axes({
    shaftRadius : 0.025,
    headRadius : 0.07,
    headLength : 0.2
  });
  // initially not visible
  scene.add(this.axes);
  this.axes.traverse(function(obj) {
    obj.visible = false;
  });

  /**
   * Display the main axes for 1 second.
   */
  this.showAxes = function() {
    scope.axes.traverse(function(obj) {
      obj.visible = true;
    });
    if (this.hideTimeout) {
      clearTimeout(this.hideTimeout);
    }
    scope.hideTimeout = setTimeout(function() {
      scope.axes.traverse(function(obj) {
        obj.visible = false;
      });
      scope.hideTimeout = false;
    }, 1000);
  };

  // events
  var changeEvent = {
    type : 'change'
  };

  /**
   * Handle the mousedown 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onMouseDown(event3D) {
    var event = event3D.domEvent;
    event.preventDefault();

    switch (event.button) {
      case 0:
        state = STATE.ROTATE;
        rotateStart.set(event.clientX, event.clientY);
        break;
      case 1:
        state = STATE.MOVE;

        moveStartNormal = new THREE.Vector3(0, 0, 1);
        var rMat = new THREE.Matrix4().extractRotation(object.matrix);
        // rMat.multiplyVector3( moveStartNormal );
        moveStartNormal.applyMatrix4(rMat);

        moveStartCenter = scope.center.clone();
        moveStartPosition = scope.object.position.clone();
        moveStartIntersection = intersectViewPlane(event3D.mouseRay, moveStartCenter,
            moveStartNormal);
        break;
      case 2:
        state = STATE.ZOOM;
        zoomStart.set(event.clientX, event.clientY);
        break;
    }

    this.showAxes();
  };

  /**
   * Handle the movemove 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onMouseMove(event3D) {
    var event = event3D.domEvent;
    if (state === STATE.ROTATE) {

      rotateEnd.set(event.clientX, event.clientY);
      rotateDelta.subVectors(rotateEnd, rotateStart);

      scope.rotateLeft(2 * Math.PI * rotateDelta.x / PIXELS_PER_ROUND * scope.userRotateSpeed);
      scope.rotateUp(2 * Math.PI * rotateDelta.y / PIXELS_PER_ROUND * scope.userRotateSpeed);

      rotateStart.copy(rotateEnd);
      this.showAxes();
    } else if (state === STATE.ZOOM) {
      zoomEnd.set(event.clientX, event.clientY);
      zoomDelta.subVectors(zoomEnd, zoomStart);

      if (zoomDelta.y > 0) {
        scope.zoomIn();
      } else {
        scope.zoomOut();
      }

      zoomStart.copy(zoomEnd);
      this.showAxes();

    } else if (state === STATE.MOVE) {
      var intersection = intersectViewPlane(event3D.mouseRay, scope.center, moveStartNormal);

      if (!intersection) {
        return;
      }

      var delta = new THREE.Vector3().subVectors(moveStartIntersection.clone(), intersection
          .clone());

      scope.center.addVectors(moveStartCenter.clone(), delta.clone());
      scope.object.position.addVectors(moveStartPosition.clone(), delta.clone());
      scope.update();
      scope.object.updateMatrixWorld();
      this.showAxes();
    }
  };

  /**
   * Handle the mouseup 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onMouseUp(event3D) {
    if (!scope.userRotate) {
      return;
    }

    state = STATE.NONE;
  };

  /**
   * Handle the mousewheel 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onMouseWheel(event3D) {
    if (!scope.userZoom) {
      return;
    }

    var event = event3D.domEvent;
    // wheelDelta --> Chrome, detail --> Firefox
    if (typeof (event.wheelDelta) !== 'undefined') {
      var delta = event.wheelDelta;
    } else {
      var delta = -event.detail;
    }
    if (delta > 0) {
      scope.zoomOut();
    } else {
      scope.zoomIn();
    }

    this.showAxes();
  };

  /**
   * Handle the touchdown 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onTouchDown(event) {
    onMouseDown(event);
    event.preventDefault();
  };

  /**
   * Handle the touchmove 3D event.
   * 
   * @param event3D - the 3D event to handle
   */
  function onTouchMove(event) {
    onMouseMove(event);
    event.preventDefault();
  };

  /**
   * Rotate the camera to the left by the given angle.
   * 
   * @param angle (optional) - the angle to rotate by
   */
  this.rotateLeft = function(angle) {
    if (angle === undefined) {
      angle = getAutoRotationAngle();
    }
    thetaDelta -= angle;
  };

  /**
   * Rotate the camera to the right by the given angle.
   * 
   * @param angle (optional) - the angle to rotate by
   */
  this.rotateRight = function(angle) {
    if (angle === undefined) {
      angle = getAutoRotationAngle();
    }
    thetaDelta += angle;
  };

  /**
   * Rotate the camera up by the given angle.
   * 
   * @param angle (optional) - the angle to rotate by
   */
  this.rotateUp = function(angle) {
    if (angle === undefined) {
      angle = getAutoRotationAngle();
    }
    phiDelta -= angle;
  };

  /**
   * Rotate the camera down by the given angle.
   * 
   * @param angle (optional) - the angle to rotate by
   */
  this.rotateDown = function(angle) {
    if (angle === undefined) {
      angle = getAutoRotationAngle();
    }
    phiDelta += angle;
  };

  /**
   * Zoom in by the given scale.
   * 
   * @param zoomScale (optional) - the scale to zoom in by
   */
  this.zoomIn = function(zoomScale) {
    if (zoomScale === undefined) {
      zoomScale = getZoomScale();
    }
    scale /= zoomScale;
  };

  /**
   * Zoom out by the given scale.
   * 
   * @param zoomScale (optional) - the scale to zoom in by
   */
  this.zoomOut = function(zoomScale) {
    if (zoomScale === undefined) {
      zoomScale = getZoomScale();
    }
    scale *= zoomScale;
  };

  /**
   * Update the camera to the current settings.
   */
  this.update = function() {
    // x->y, y->z, z->x
    var position = this.object.position;
    var offset = position.clone().sub(this.center);

    // angle from z-axis around y-axis
    var theta = Math.atan2(offset.y, offset.x);

    // angle from y-axis
    var phi = Math.atan2(Math.sqrt(offset.y * offset.y + offset.x * offset.x), offset.z);

    if (this.autoRotate) {
      this.rotateLeft(getAutoRotationAngle());
    }

    theta += thetaDelta;
    phi += phiDelta;

    // restrict phi to be betwee EPS and PI-EPS
    phi = Math.max(EPS, Math.min(Math.PI - EPS, phi));

    var radius = offset.length();
    offset.y = radius * Math.sin(phi) * Math.sin(theta);
    offset.z = radius * Math.cos(phi);
    offset.x = radius * Math.sin(phi) * Math.cos(theta);
    offset.multiplyScalar(scale);

    position.copy(this.center).add(offset);

    this.object.lookAt(this.center);

    radius = offset.length();
    this.axes.position = this.center.clone();
    this.axes.scale.x = this.axes.scale.y = this.axes.scale.z = radius * 0.05;
    this.axes.updateMatrixWorld(true);

    thetaDelta = 0;
    phiDelta = 0;
    scale = 1;

    if (lastPosition.distanceTo(this.object.position) > 0) {
      this.dispatchEvent(changeEvent);
      lastPosition.copy(this.object.position);
    }
  };

  // add event listeners
  this.addEventListener('mousedown', onMouseDown);
  this.addEventListener('mouseup', onMouseUp);
  this.addEventListener('mousemove', onMouseMove);
  this.addEventListener('touchstart', onTouchDown);
  this.addEventListener('touchmove', onTouchMove);
  // Chrome/Firefox have different events here
  this.addEventListener('mousewheel', onMouseWheel);
  this.addEventListener('DOMMouseScroll', onMouseWheel);
};
ROS3D.UrdfLoader = function(options) {
  var options = options || {};
  this.ros = options.ros;
  this.xmlParam = options.xmlParam || 'robot_description';
  this.xmlFile = options.xmlFile;

  // check for what type of XML we are getting
  if (this.xmlFile) {

  } else {
    // get the value from ROS
    //ros.
  }
};

var UrdfLoader = {};

UrdfLoader.load = function(objroot, meshLoader, tfClient, urdf_src) {

  var urdf_model = new UrdfModel();
  var that = this;
  var scene_handlers = {};

  urdf_model.initFile(urdf_src, urdfReady);

  function urdfReady() {
    // load all models
    var links = urdf_model.getLinks();
    for ( var l in links) {
      var link = links[l];
      if (!link.visual)
        continue;
      if (!link.visual.geometry)
        continue;
      if (link.visual.geometry.type == link.visual.geometry.GeometryTypes.MESH) {
        var frame_id = new String("/" + link.name);
        var uri = link.visual.geometry.filename;
        var uriends = uri.substr(-4).toLowerCase();

        // ignore mesh files which are not in collada format
        if (uriends == ".dae") {
          var material_name = link.visual.material_name;
          var material = urdf_model.getMaterial(material_name);

          var collada_model = meshLoader.load(uri, material);

          var scene_node = new SceneNode({
            frame_id : frame_id,
            tfclient : tfClient,
            pose : link.visual.origin,
            model : collada_model
          });
          objroot.add(scene_node);
        } else {
          console.log("Not Supported format : ", uri);
        }
      }
    }
  }
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
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

  // add the renderer to the page
  document.getElementById(this.divID).appendChild(this.renderer.domElement);

  // begin the animation
  draw();
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A mouseover highlighter for 3D objects in the scene.
 *
 * @constructor
 * @param mouseHandler - the handler for the mouseover and mouseout events
 */
ROS3D.Highlighter = function(mouseHandler) {
  this.hoverObjs = [];

  /**
   * Add the current target of the mouseover to the hover list.
   * @param event - the event that contains the target of the mouseover
   */
  this.onMouseOver = function(event) {
    this.hoverObjs.push(event.currentTarget);
  };

  /**
   * Remove the current target of the mouseover from the hover list.
   * @param event - the event that contains the target of the mouseout
   */
  this.onMouseOut = function(event) {
    this.hoverObjs.splice(this.hoverObjs.indexOf(event.currentTarget), 1);
  };

  /**
   * Add all corresponding webgl objects in the given scene and add them to the given render list.
   * 
   * @param scene - the scene to check for webgl objects
   * @param objects - the objects list to check
   * @param renderList - the list to add to
   */
  this.getWebglObjects = function(scene, objects, renderList) {
    var objlist = scene.__webglObjects;
    // get corresponding webgl objects
    for ( var c = 0; c < objects.length; c++) {
      if (objects[c]) {
        for ( var o = objlist.length - 1; o >= 0; o--) {
          if (objlist[o].object === objects[c]) {
            renderList.push(objlist[o]);
            break;
          }
        }
        // recurse into children
        this.getWebglObjects(scene, objects[c].children, renderList);
      }
    }
  };

  /**
   * Render highlighted objects in the scene.
   * 
   * @param renderer - the renderer to use
   * @param scene - the scene to use
   * @param camera - the camera to use
   */
  this.renderHighlight = function(renderer, scene, camera) {
    // get webgl objects
    var renderList = [];
    this.getWebglObjects(scene, this.hoverObjs, renderList);

    // define highlight material
    scene.overrideMaterial = new THREE.MeshBasicMaterial({
      fog : false,
      opacity : 0.5,
      depthTest : true,
      depthWrite : false,
      polygonOffset : true,
      polygonOffsetUnits : -1,
      side : THREE.DoubleSide
    });;

    // swap render lists, render, undo
    var oldWebglObjects = scene.__webglObjects;
    scene.__webglObjects = renderList;

    renderer.render(scene, camera);

    scene.__webglObjects = oldWebglObjects;
    scene.overrideMaterial = null;
  };

  // bind the mouse events
  mouseHandler.addEventListener('mouseover', this.onMouseOver.bind(this));
  mouseHandler.addEventListener('mouseout', this.onMouseOut.bind(this));
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A handler for mouse events within a 3D viewer.
 *
 * @constructor
 * @param renderer - the main renderer
 * @param camera - the main camera in the scene
 * @param rootObj - the root object to check for mouse events
 * @param fallbackTarget - the fallback target, e.g., the camera controls
 */
ROS3D.MouseHandler = function(renderer, camera, rootObj, fallbackTarget) {
  THREE.EventDispatcher.call(this);
  this.camera = camera;
  this.rootObj = rootObj;
  this.renderer = renderer;
  this.fallbackTarget = fallbackTarget;
  this.lastTarget = fallbackTarget;
  this.dragging = false;
  this.projector = new THREE.Projector();

  // listen to DOM events
  var eventNames = [ 'contextmenu', 'click', 'dblclick', 'mouseout', 'mousedown', 'mouseup',
      'mousemove', 'mousewheel', 'DOMMouseScroll', 'touchstart', 'touchend', 'touchcancel',
      'touchleave', 'touchmove' ];
  this.listeners = {};

  /**
   * Process the particular DOM even that has occurred based on the mouse's position in the scene.
   * 
   * @param domEvent - the DOM event to process
   */
  this.processDomEvent = function(domEvent) {
    // don't deal with the default handler
    domEvent.preventDefault();

    // compute normalized device coords and 3D mouse ray
    var target = domEvent.target;
    var rect = target.getBoundingClientRect();
    var left = domEvent.clientX - rect.left - target.clientLeft + target.scrollLeft;
    var top = domEvent.clientY - rect.top - target.clientTop + target.scrollTop;
    var deviceX = left / target.clientWidth * 2 - 1;
    var deviceY = -top / target.clientHeight * 2 + 1;
    var vector = new THREE.Vector3(deviceX, deviceY, 0.5);
    this.projector.unprojectVector(vector, this.camera);
    // use the THREE raycaster
    var mouseRaycaster = new THREE.Raycaster(this.camera.position.clone(), vector.sub(
        this.camera.position).normalize());
    var mouseRay = mouseRaycaster.ray;

    // make our 3d mouse event
    var event3D = {
      mousePos : new THREE.Vector2(deviceX, deviceY),
      mouseRay : mouseRay,
      domEvent : domEvent,
      camera : this.camera,
      intersection : this.lastIntersection
    };

    // if the mouse leaves the dom element, stop everything
    if (domEvent.type == 'mouseout') {
      if (this.dragging) {
        this.notify(this.lastTarget, 'mouseup', event3D);
        this.dragging = false;
      }
      this.notify(this.lastTarget, 'mouseout', event3D);
      this.lastTarget = null;
      return;
    }

    // while the user is holding the mouse down, stay on the same target
    if (this.dragging) {
      this.notify(this.lastTarget, domEvent.type, event3D);
      // for check for right or left mouse button
      if ((domEvent.type === 'mouseup' && domEvent.button === 2) || domEvent.type === 'click') {
        this.dragging = false;
      }
      return;
    }

    // in the normal case, we need to check what is under the mouse
    var target = this.lastTarget;
    var intersections = [];
    intersections = mouseRaycaster.intersectObject(this.rootObj, true);
    if (intersections.length > 0) {
      target = intersections[0].object;
      event3D.intersection = this.lastIntersection = intersections[0];
    } else {
      target = this.fallbackTarget;
    }

    // if the mouse moves from one object to another (or from/to the 'null' object), notify both
    if (target !== this.lastTarget) {
      var eventAccepted = this.notify(target, 'mouseover', event3D);
      if (eventAccepted) {
        this.notify(this.lastTarget, 'mouseout', event3D);
      } else {
        // if target was null or no target has caught our event, fall back
        target = this.fallbackTarget;
        if (target !== this.lastTarget) {
          this.notify(target, 'mouseover', event3D);
          this.notify(this.lastTarget, 'mouseout', event3D);
        }
      }
    }

    // pass through event
    this.notify(target, domEvent.type, event3D);
    if (domEvent.type === 'mousedown') {
      this.dragging = true;
    }
    this.lastTarget = target;
  };

  /**
   * Notify the listener of the type of event that occurred.
   * 
   * @param target - the target of the event
   * @param type - the type of event that occurred 
   * @param event3D - the 3D mouse even information
   * @returns if an event was canceled
   */
  this.notify = function(target, type, event3D) {
    // ensure the type is set
    event3D.type = type;

    // make the event cancelable
    event3D.cancelBubble = false;
    event3D.stopPropagation = function() {
      event3D.cancelBubble = true;
    };
    // walk up graph until event is canceled or root node has been reached
    event3D.currentTarget = target;
    while (event3D.currentTarget) {
      // try to fire event on object
      if (event3D.currentTarget.dispatchEvent
          && event3D.currentTarget.dispatchEvent instanceof Function) {
        event3D.currentTarget.dispatchEvent(event3D);
        if (event3D.cancelBubble) {
          this.dispatchEvent(event3D);
          return true;
        }
      }
      // walk up
      event3D.currentTarget = event3D.currentTarget.parent;
    }
    return false;
  };

  /**
   * Destroy this mouse handler and its associated listeners.
   */
  this.destroy = function() {
    this.listeners.forEach(function(listener) {
      this.renderer.domElement.removeEventListener(eventName, listener, false);
    }, this);
  };

  // add event listeners for the associated mouse events
  eventNames.forEach(function(eventName) {
    this.listeners[eventName] = this.processDomEvent.bind(this);
    this.renderer.domElement.addEventListener(eventName, this.listeners[eventName], false);
  }, this);
};
