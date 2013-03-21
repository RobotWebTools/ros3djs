/**
 * @author Russell Toris - rctoris@wpi.edu
 */

var ROS3D = ROS3D || {
  REVISION : '1'
};

// URDF types
ROS3D.URDF_SPHERE = 0;
ROS3D.URDF_BOX = 1;
ROS3D.URDF_CYLINDER = 2;
ROS3D.URDF_MESH = 3;
/**
 * Class to handle the Box geometry type
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfBox = function() {
  var urdfBox = this;
  this.dim;
  this.type;

  this.initXml = function(xml) {
    this.type = ROS3D.URDF_BOX;
    // check for a size
    if (!xml.getAttribute('size')) {
      console.error("Box shape must have a size attribute'");
      return false;
    }
    // parse the string
    var xyz = xml.getAttribute('size').split(' ');
    if (xyz.length !== 3) {
      console.error('Invalid size string.');
      return false;
    } else {
      urdfBox.dim = new ROSLIB.Vector3({
        x : parseFloat(xyz[0]),
        y : parseFloat(xyz[1]),
        z : parseFloat(xyz[2])
      });
    }

    return true;
  };
};
/**
 * Class to handle color for urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfColor = function() {
  var urdfColor = this;
  this.r;
  this.g;
  this.b;
  this.a;

  this.initString = function(str) {
    // try and parse the string
    var rgba = str.split(' ');
    if (rgba.length !== 4) {
      console.error('Invalid RBGA string.');
      return false;
    } else {
      urdfColor.r = parseFloat(rgba[0]);
      urdfColor.g = parseFloat(rgba[1]);
      urdfColor.b = parseFloat(rgba[2]);
      urdfColor.a = parseFloat(rgba[3]);
      return true;
    }
  };
};
/**
 * Class to handle the Cylinder geometry type of a URDF.
 * 
 * @class
 */
ROS3D.UrdfCylinder = function() {
  this.type = 0;
  this.length = 0;
  this.radius = 0;

  this.initXml = function(xml) {
    this.type = ROS3D.URDF_CYLINDER;
    this.length = parseFloat(xml.getAttribute('length'));
    this.radius = parseFloat(xml.getAttribute('radius'));
  };
};
ROS3D.UrdfLink = function() {
  var urdfLink = this;
  this.name;
  this.visual;

  this.initXml = function(xml) {
    if (!(urdfLink.name = xml.getAttribute('name'))) {
      console.error('No name given for link.');
      return false;
    }

    // visual (optional)
    var visuals = xml.getElementsByTagName('visual');
    if (visuals.length > 0) {
      var visualXml = visuals[0];
      var visual = new ROS3D.UrdfVisual();
      if (!visual.initXml(visualXml)) {
        console.error('Could not parse visual element for Link ' + this.name);
        return false;
      }
      urdfLink.visual = visual;
    }

    return true;
  };
};
/**
 * Handles material information for the URDF
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfMaterial = function() {
  var urdfMaterial = this;
  this.name;
  this.textureFilename;
  this.color;

  this.initXml = function(xml) {
    var hasRgba = false;
    var hasFilename = false;

    // check for the name
    if (!(urdfMaterial.name = xml.getAttribute('name'))) {
      console.error('URDF Material must contain a name attribute.');
      return false;
    }

    // texture
    var textures = xml.getElementsByTagName('texture');
    if (textures.length > 0) {
      var texture = textures[0];
      if ((urdfMaterial.textureFilename = texture.getAttribute('filename'))) {
        hasFilename = true;
      } else {
        console.error('URDF texture has no filename for material ' + urdfMaterial.name + '.');
      }
    }

    // color
    var colors = xml.getElementsByTagName('color');
    if (colors.length > 0) {
      var c = colors[0];
      if (c.getAttribute('rgba')) {
        // parse the RBGA string
        urdfMaterial.color = new ROS3D.UrdfColor();
        hasRgba = urdfMaterial.color.initString(c.getAttribute('rgba'));
      } else {
        console.error('Material ' + this.name + ' color has no rgba.');
      }
    }

    // check if we have a texture or color
    return (hasRgba || hasFilename);
  };
};
/**
 * Mesh information for the urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfMesh = function() {
  var urdfMesh = this;
  this.filename = '';
  this.scale;
  this.type;

  this.initXml = function(xml) {
    this.type = ROS3D.URDF_MESH;
    if (!(urdfMesh.filename = xml.getAttribute('filename'))) {
      console.error('Mesh must contain a filename attribute.');
      return false;
    }

    // check for a scale
    if (xml.getAttribute('scale')) {
      // check the XYZ
      var xyz = xml.getAttribute('scale').split(' ');
      if (xyz.length !== 3) {
        console.error('Invalid scale string.');
        return false;
      } else {
        urdfMesh.scale = new ROSLIB.Vector3({
          x : parseFloat(xyz[0]),
          y : parseFloat(xyz[1]),
          z : parseFloat(xyz[2])
        });
      }
    }

    return true;
  };
};
ROS3D.UrdfModel = function(options) {
  var that = this;
  var options = options || {};
  var xml = options.xml;
  var string = options.string;

  this.name;
  this.materials = [];
  this.links = [];

  var initXml = function(xml) {
    // check for the robot tag
    var robotXml = xml.evaluate('//robot', xml, null, XPathResult.FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
    if (!robotXml) {
      console.error('Could not find the "robot" element in the URDF XML file.');
      return false;
    }

    // get the robot name
    if (!(that.name = robotXml.getAttribute('name'))) {
      console.error("No name given for the robot.");
      return false;
    }

    // parse all the visual elements we need
    for (n in robotXml.childNodes) {
      var node = robotXml.childNodes[n];
      if (node.tagName === 'material') {
        var material = new ROS3D.UrdfMaterial();
        if (material.initXml(node)) {
          // make sure this is unique
          if (that.materials[material.name]) {
            console.error('Material ' + material.name + 'is not unique.');
            return false;
          } else {
            that.materials[material.name] = material;
          }
        } else {
          return false;
        }
      } else if (node.tagName === 'link') {
        var link = new ROS3D.UrdfLink();

        if (link.initXml(node)) {
          if (that.links[link.name]) {
            console.error('Link ' + link.name + ' is not unique.');
            return false;
          } else {
            // check for a material
            if (link.visual && link.visual.materialName) {
              if (that.materials[link.visual.materialName]) {
                link.visual.material = that.materials[link.visual.materialName];
              } else {
                if (link.visual.material) {
                  that.materials[link.visual.material.name] = link.visual.material;
                } else {
                  console.error('Link ' + link.name + ' material ' + link.visual.material_name
                      + ' is undefined.');
                  return false;
                }
              }
            }

            // add the link
            that.links[link.name] = link;
          }
        } else {
          console.error('Could not parse link.');
          return false;
        }
      }
    }

    return true;
  };

  // check if we are using a string or an XML element
  if (string) {
    // parse the string
    var parser = new DOMParser();
    xml = parser.parseFromString(string, 'text/xml');
  }
  // pass it to the XML parser
  initXml(xml);
};
/**
 * Class to handle visualizing spheres in the urdf
 * 
 * @class
 * @augments Class
 */
ROS3D.UrdfSphere = function() {
  var urdfSphere = this;
  this.radius;
  this.type;

  this.initXml = function(xml) {
    this.type = ROS3D.URDF_SPHERE;
    if (!xml.getAttribute('radius')) {
      console.error('Sphere shape must have a radius attribute');
      return false;
    }
    urdfSphere.radius = parseFloat(xml.getAttribute("radius"));
    return true;
  };
};
ROS3D.UrdfVisual = function() {
  var urdfVisual = this;
  this.origin;
  this.geometry;
  this.material;
  this.materialName;

  this.initXml = function(xml) {
    // origin
    var origins = xml.getElementsByTagName('origin');
    if (origins.length === 0) {
      // use the identity as the default
      urdfVisual.origin = new ROSLIB.Pose();
    } else {
      // check the XYZ
      var xyz = xml.getAttribute('xyz');
      if (!xyz) {
        // use the default values
        var position = new ROSLIB.Vector3();
      } else {
        var xyz = xyz.split(' ');
        if (xyz.length !== 3) {
          console.error('Invalid XYZ string in origin.');
          return false;
        } else {
          var position = new ROSLIB.Vector3({
            x : parseFloat(xyz[0]),
            y : parseFloat(xyz[1]),
            z : parseFloat(xyz[2])
          });
        }
      }

      // check the RPY
      rpy = xml.getAttribute('rpy');
      if (!rpy) {
        // use the default values
        var orientation = new ROSLIB.Quaternion();
      } else {
        var rpy = rpy.split(' ');
        if (rpy.length !== 3) {
          console.error('Invalid RPY string in origin.');
          return false;
        } else {
          // convert from RPY
          var roll = parseFloat(rpy[0]);
          var pitch = parseFloat(rpy[1]);
          var yaw = parseFloat(rpy[2]);
          var phi = roll / 2.0;
          var the = pitch / 2.0;
          var psi = yaw / 2.0;

          var x = Math.sin(phi) * Math.cos(the) * Math.cos(psi) - Math.cos(phi) * Math.sin(the)
              * Math.sin(psi);
          var y = Math.cos(phi) * Math.sin(the) * Math.cos(psi) + Math.sin(phi) * Math.cos(the)
              * Math.sin(psi);
          var z = Math.cos(phi) * Math.cos(the) * Math.sin(psi) - Math.sin(phi) * Math.sin(the)
              * Math.cos(psi);
          var w = Math.cos(phi) * Math.cos(the) * Math.cos(psi) + Math.sin(phi) * Math.sin(the)
              * Math.sin(psi);

          var orientation = new ROSLIB.Quaternion({
            x : x,
            y : y,
            z : z,
            w : w
          });
          orientation.normalize();
        }
      }
      urdfVisual.origin = new ROSLIB.Pose({
        position : position,
        orientation : orientation
      });
    }

    var geoms = xml.getElementsByTagName('geometry');
    if (geoms.length > 0) {
      var shape;
      // check for the shape
      for (n in geoms[0].childNodes) {
        var node = geoms[0].childNodes[n];
        if (node.nodeType === 1) {
          shape = node;
          break;
        }
      }
      if (!shape) {
        console.error('Geometry tag contains no child element.');
        return false;
      }
      var type = shape.nodeName;
      if (type === 'sphere') {
        geometry = new ROS3D.UrdfSphere();
      } else if (type === 'box') {
        geometry = new ROS3D.UrdfBox();
      } else if (type === 'cylinder') {
        geometry = new ROS3D.UrdfCylinder();
      } else if (type === 'mesh') {
        geometry = new ROS3D.UrdfMesh();
      } else {
        console.error('Unknown geometry type ' + type);
        return false;
      }
      if (!geometry.initXml(shape)) {
        console.error('Could not parse visual.');
        return false;
      }

      urdfVisual.geometry = geometry;
    }

    // material (optional)
    var materials = xml.getElementsByTagName('material');
    if (materials.length > 0) {
      // get material name
      var materialXml = materials[0];
      var material = new ROS3D.UrdfMaterial();
      // could just be a name
      if (!(urdfVisual.materialName = materialXml.getAttribute('name'))) {
        console.error('URDF material must contain a name attribute');
        return false;
      }

      // try to parse material element in place
      if (material.initXml(materialXml)) {
        urdfVisual.material = material;
      }
    }
    return true;
  };
};
/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A SceneNode can be used to keep track of a 3D object with respect to a ROS frame within a scene.
 *
 * @constructor
 * @param options - object with following keys:
 *  * tfClient - a handle to the TF client
 *  * frameID - the frame ID this object belongs to
 *  * model - the THREE 3D object to be rendered
 */
ROS3D.SceneNode = function(options) {
  var options = options || {};
  var that = this;
  this.tfClient = options.tfClient;
  this.frameID = options.frameID;
  this.model = options.model;

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // add the model
  this.add(this.model);

  // listen for TF updates
  this.tfClient.subscribe(this.frameID,
      function(msg) {
        // apply the transform
        var tf = new ROSLIB.Transform(msg);
        var poseTransformed = new ROSLIB.Pose();
        poseTransformed.applyTransform(tf);

        // update the world
        that.position.x = poseTransformed.position.x;
        that.position.y = poseTransformed.position.y;
        that.position.z = poseTransformed.position.z;
        that.quaternion = new THREE.Quaternion(poseTransformed.orientation.x,
            poseTransformed.orientation.y, poseTransformed.orientation.z,
            poseTransformed.orientation.w);
        that.updateMatrixWorld(true);
      });
};
ROS3D.SceneNode.prototype.__proto__ = THREE.Object3D.prototype;
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
 *  * divID - the ID of the div to place the viewer in
 *  * width - the initial width, in pixels, of the canvas
 *  * height - the initial height, in pixels, of the canvas
 *  * disableGrid - if the grid feature should be disabled in the viewer
 *  * gridColor - the color to render the grid lines, like #cccccc
 *  * background - the color to render the background, like #efefef
 *  * antialias - if antialiasing should be used
 */
ROS3D.Viewer = function(options) {
  var that = this;
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
   * Renders the associated scene to the that.
   */
  function draw() {
    // update the controls
    that.cameraControls.update();

    // put light to the top-left of the camera
    that.directionalLight.position = that.camera.localToWorld(new THREE.Vector3(-1, 1, 0));
    that.directionalLight.position.normalize();

    // set the scene
    that.renderer.clear(true, true, true);
    that.renderer.render(that.scene, that.camera);

    // render any mouseovers
    that.highlighter.renderHighlight(that.renderer, that.scene, that.camera);

    // draw the frame
    requestAnimationFrame(draw);
  };

  // add the renderer to the page
  document.getElementById(this.divID).appendChild(this.renderer.domElement);

  // begin the animation
  draw();
};

/**
 * Add the given THREE Object3D to the global scene in the that.
 * 
 * @param object - the THREE Object3D to add
 */
ROS3D.Viewer.prototype.addObject = function(object) {
  this.scene.add(object);
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
/**
 * @author Tim Knip / http://www.floorplanner.com/ / tim at floorplanner.com
 */
ROS3D.ColladaLoader = function () {

	var COLLADA = null;
	var scene = null;
	var daeScene;

	var readyCallbackFunc = null;

 	var sources = {};
	var images = {};
	var animations = {};
	var controllers = {};
	var geometries = {};
	var materials = {};
	var effects = {};
	var cameras = {};

	var animData;
	var visualScenes;
	var baseUrl;
	var morphs;
	var skins;

	var flip_uv = true;
	var preferredShading = THREE.SmoothShading;

	var options = {
		// Force Geometry to always be centered at the local origin of the
		// containing Mesh.
		centerGeometry: false,

		// Axis conversion is done for geometries, animations, and controllers.
		// If we ever pull cameras or lights out of the COLLADA file, they'll
		// need extra work.
		convertUpAxis: false,

		subdivideFaces: true,

		upAxis: 'Y',

		// For reflective or refractive materials we'll use this cubemap
		defaultEnvMap: null

	};

	// TODO: support unit conversion as well
	var colladaUnit = 1.0;
	var colladaUp = 'Y';
	var upConversion = null;

	var TO_RADIANS = Math.PI / 180;

	function load ( url, readyCallback, progressCallback ) {

		var length = 0;

		if ( document.implementation && document.implementation.createDocument ) {

			var request = new XMLHttpRequest();

			request.onreadystatechange = function() {

				if( request.readyState == 4 ) {

					if( request.status == 0 || request.status == 200 ) {


						if ( request.responseXML ) {

							readyCallbackFunc = readyCallback;
							parse( request.responseXML, undefined, url );

						} else if ( request.responseText ) {

							readyCallbackFunc = readyCallback;
							var xmlParser = new DOMParser();
							var responseXML = xmlParser.parseFromString( request.responseText, "application/xml" );
							parse( responseXML, undefined, url );

						} else {

							console.error( "ColladaLoader: Empty or non-existing file (" + url + ")" );

						}

					}

				} else if ( request.readyState == 3 ) {

					if ( progressCallback ) {

						if ( length == 0 ) {

							length = request.getResponseHeader( "Content-Length" );

						}

						progressCallback( { total: length, loaded: request.responseText.length } );

					}

				}

			}

			request.open( "GET", url, true );
			request.send( null );

		} else {

			alert( "Don't know how to parse XML!" );

		}

	};

	function parse( doc, callBack, url ) {

		COLLADA = doc;
		callBack = callBack || readyCallbackFunc;

		if ( url !== undefined ) {

			var parts = url.split( '/' );
			parts.pop();
			baseUrl = ( parts.length < 1 ? '.' : parts.join( '/' ) ) + '/';

		}

		parseAsset();
		setUpConversion();
		images = parseLib( "//dae:library_images/dae:image", _Image, "image" );
		materials = parseLib( "//dae:library_materials/dae:material", Material, "material" );
		effects = parseLib( "//dae:library_effects/dae:effect", Effect, "effect" );
		geometries = parseLib( "//dae:library_geometries/dae:geometry", Geometry, "geometry" );
		cameras = parseLib( ".//dae:library_cameras/dae:camera", Camera, "camera" );
		controllers = parseLib( "//dae:library_controllers/dae:controller", Controller, "controller" );
		animations = parseLib( "//dae:library_animations/dae:animation", Animation, "animation" );
		visualScenes = parseLib( ".//dae:library_visual_scenes/dae:visual_scene", VisualScene, "visual_scene" );

		morphs = [];
		skins = [];

		daeScene = parseScene();
		scene = new THREE.Object3D();

  	for ( var i = 0; i < daeScene.nodes.length; i ++ ) {
	  	scene.add( createSceneGraph( daeScene.nodes[ i ] ) );
		}

		createAnimations();

		var result = {

			scene: scene,
			morphs: morphs,
			skins: skins,
			animations: animData,
			dae: {
				images: images,
				materials: materials,
				cameras: cameras,
				effects: effects,
				geometries: geometries,
				controllers: controllers,
				animations: animations,
				visualScenes: visualScenes,
				scene: daeScene
			}

		};

		if ( callBack ) {

			callBack( result );

		}

		return result;

	};

	function setPreferredShading ( shading ) {

		preferredShading = shading;

	};

	function parseAsset () {

		var elements = COLLADA.evaluate( '//dae:asset', COLLADA, _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null );

		var element = elements.iterateNext();

		if ( element && element.childNodes ) {

			for ( var i = 0; i < element.childNodes.length; i ++ ) {

				var child = element.childNodes[ i ];

				switch ( child.nodeName ) {

					case 'unit':

						var meter = child.getAttribute( 'meter' );

						if ( meter ) {

							colladaUnit = parseFloat( meter );

						}

						break;

					case 'up_axis':

						colladaUp = child.textContent.charAt(0);
						break;

				}

			}

		}

	};

	function parseLib ( q, classSpec, prefix ) {

		var elements = COLLADA.evaluate(q, COLLADA, _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null) ;

		var lib = {};
		var element = elements.iterateNext();
		var i = 0;

		while ( element ) {

			var daeElement = ( new classSpec() ).parse( element );
			if ( !daeElement.id || daeElement.id.length == 0 ) daeElement.id = prefix + ( i ++ );
			lib[ daeElement.id ] = daeElement;

			element = elements.iterateNext();

		}

		return lib;

	};

	function parseScene() {

		var sceneElement = COLLADA.evaluate( './/dae:scene/dae:instance_visual_scene', COLLADA, _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null ).iterateNext();

		if ( sceneElement ) {

			var url = sceneElement.getAttribute( 'url' ).replace( /^#/, '' );
			return visualScenes[ url.length > 0 ? url : 'visual_scene0' ];

		} else {

			return null;

		}

	};

	function createAnimations() {

		animData = [];

		// fill in the keys
		recurseHierarchy( scene );

	};

	function recurseHierarchy( node ) {

		var n = daeScene.getChildById( node.name, true ),
			newData = null;

		if ( n && n.keys ) {

			newData = {
				fps: 60,
				hierarchy: [ {
					node: n,
					keys: n.keys,
					sids: n.sids
				} ],
				node: node,
				name: 'animation_' + node.name,
				length: 0
			};

			animData.push(newData);

			for ( var i = 0, il = n.keys.length; i < il; i++ ) {

				newData.length = Math.max( newData.length, n.keys[i].time );

			}

		} else  {

			newData = {
				hierarchy: [ {
					keys: [],
					sids: []
				} ]
			}

		}

		for ( var i = 0, il = node.children.length; i < il; i++ ) {

			var d = recurseHierarchy( node.children[i] );

			for ( var j = 0, jl = d.hierarchy.length; j < jl; j ++ ) {

				newData.hierarchy.push( {
					keys: [],
					sids: []
				} );

			}

		}

		return newData;

	};

	function calcAnimationBounds () {

		var start = 1000000;
		var end = -start;
		var frames = 0;

		for ( var id in animations ) {

			var animation = animations[ id ];

			for ( var i = 0; i < animation.sampler.length; i ++ ) {

				var sampler = animation.sampler[ i ];
				sampler.create();

				start = Math.min( start, sampler.startTime );
				end = Math.max( end, sampler.endTime );
				frames = Math.max( frames, sampler.input.length );

			}

		}

		return { start:start, end:end, frames:frames };

	};

	function createMorph ( geometry, ctrl ) {

		var morphCtrl = ctrl instanceof InstanceController ? controllers[ ctrl.url ] : ctrl;

		if ( !morphCtrl || !morphCtrl.morph ) {

			console.log("could not find morph controller!");
			return;

		}

		var morph = morphCtrl.morph;

		for ( var i = 0; i < morph.targets.length; i ++ ) {

			var target_id = morph.targets[ i ];
			var daeGeometry = geometries[ target_id ];

			if ( !daeGeometry.mesh ||
				 !daeGeometry.mesh.primitives ||
				 !daeGeometry.mesh.primitives.length ) {
				 continue;
			}

			var target = daeGeometry.mesh.primitives[ 0 ].geometry;

			if ( target.vertices.length === geometry.vertices.length ) {

				geometry.morphTargets.push( { name: "target_1", vertices: target.vertices } );

			}

		}

		geometry.morphTargets.push( { name: "target_Z", vertices: geometry.vertices } );

	};

	function createSkin ( geometry, ctrl, applyBindShape ) {

		var skinCtrl = controllers[ ctrl.url ];

		if ( !skinCtrl || !skinCtrl.skin ) {

			console.log( "could not find skin controller!" );
			return;

		}

		if ( !ctrl.skeleton || !ctrl.skeleton.length ) {

			console.log( "could not find the skeleton for the skin!" );
			return;

		}

		var skin = skinCtrl.skin;
		var skeleton = daeScene.getChildById( ctrl.skeleton[ 0 ] );
		var hierarchy = [];

		applyBindShape = applyBindShape !== undefined ? applyBindShape : true;

		var bones = [];
		geometry.skinWeights = [];
		geometry.skinIndices = [];

		//createBones( geometry.bones, skin, hierarchy, skeleton, null, -1 );
		//createWeights( skin, geometry.bones, geometry.skinIndices, geometry.skinWeights );

		/*
		geometry.animation = {
			name: 'take_001',
			fps: 30,
			length: 2,
			JIT: true,
			hierarchy: hierarchy
		};
		*/

		if ( applyBindShape ) {

			for ( var i = 0; i < geometry.vertices.length; i ++ ) {

				skin.bindShapeMatrix.multiplyVector3( geometry.vertices[ i ] );

			}

		}

	};

	function setupSkeleton ( node, bones, frame, parent ) {

		node.world = node.world || new THREE.Matrix4();
		node.world.copy( node.matrix );

		if ( node.channels && node.channels.length ) {

			var channel = node.channels[ 0 ];
			var m = channel.sampler.output[ frame ];

			if ( m instanceof THREE.Matrix4 ) {

				node.world.copy( m );

			}

		}

		if ( parent ) {

			node.world.multiply( parent, node.world );

		}

		bones.push( node );

		for ( var i = 0; i < node.nodes.length; i ++ ) {

			setupSkeleton( node.nodes[ i ], bones, frame, node.world );

		}

	};

	function setupSkinningMatrices ( bones, skin ) {

		// FIXME: this is dumb...

		for ( var i = 0; i < bones.length; i ++ ) {

			var bone = bones[ i ];
			var found = -1;

			if ( bone.type != 'JOINT' ) continue;

			for ( var j = 0; j < skin.joints.length; j ++ ) {

				if ( bone.sid == skin.joints[ j ] ) {

					found = j;
					break;

				}

			}

			if ( found >= 0 ) {

				var inv = skin.invBindMatrices[ found ];

				bone.invBindMatrix = inv;
				bone.skinningMatrix = new THREE.Matrix4();
				bone.skinningMatrix.multiply(bone.world, inv); // (IBMi * JMi)

				bone.weights = [];

				for ( var j = 0; j < skin.weights.length; j ++ ) {

					for (var k = 0; k < skin.weights[ j ].length; k ++) {

						var w = skin.weights[ j ][ k ];

						if ( w.joint == found ) {

							bone.weights.push( w );

						}

					}

				}

			} else {

				throw 'ColladaLoader: Could not find joint \'' + bone.sid + '\'.';

			}

		}

	};

	function applySkin ( geometry, instanceCtrl, frame ) {

		var skinController = controllers[ instanceCtrl.url ];

		frame = frame !== undefined ? frame : 40;

		if ( !skinController || !skinController.skin ) {

			console.log( 'ColladaLoader: Could not find skin controller.' );
			return;

		}

		if ( !instanceCtrl.skeleton || !instanceCtrl.skeleton.length ) {

			console.log( 'ColladaLoader: Could not find the skeleton for the skin. ' );
			return;

		}

		var animationBounds = calcAnimationBounds();
		var skeleton = daeScene.getChildById( instanceCtrl.skeleton[0], true ) ||
					   daeScene.getChildBySid( instanceCtrl.skeleton[0], true );

		var i, j, w, vidx, weight;
		var v = new THREE.Vector3(), o, s;

		// move vertices to bind shape

		for ( i = 0; i < geometry.vertices.length; i ++ ) {

			skinController.skin.bindShapeMatrix.multiplyVector3( geometry.vertices[i] );

		}

		// process animation, or simply pose the rig if no animation

		for ( frame = 0; frame < animationBounds.frames; frame ++ ) {

			var bones = [];
			var skinned = [];

			// zero skinned vertices

			for ( i = 0; i < geometry.vertices.length; i++ ) {

				skinned.push( new THREE.Vector3() );

			}

			// process the frame and setup the rig with a fresh
			// transform, possibly from the bone's animation channel(s)

			setupSkeleton( skeleton, bones, frame );
			setupSkinningMatrices( bones, skinController.skin );

			// skin 'm

			for ( i = 0; i < bones.length; i ++ ) {

				if ( bones[ i ].type != 'JOINT' ) continue;

				for ( j = 0; j < bones[ i ].weights.length; j ++ ) {

					w = bones[ i ].weights[ j ];
					vidx = w.index;
					weight = w.weight;

					o = geometry.vertices[vidx];
					s = skinned[vidx];

					v.x = o.x;
					v.y = o.y;
					v.z = o.z;

					bones[i].skinningMatrix.multiplyVector3(v);

					s.x += (v.x * weight);
					s.y += (v.y * weight);
					s.z += (v.z * weight);

				}

			}

			geometry.morphTargets.push( { name: "target_" + frame, vertices: skinned } );

		}

	};

	function createSceneGraph ( node, parent ) {

		var obj = new THREE.Object3D();
		var skinned = false;
		var skinController;
		var morphController;
		var i, j;

		// FIXME: controllers

		for ( i = 0; i < node.controllers.length; i ++ ) {

			var controller = controllers[ node.controllers[ i ].url ];

			switch ( controller.type ) {

				case 'skin':

					if ( geometries[ controller.skin.source ] ) {

						var inst_geom = new InstanceGeometry();

						inst_geom.url = controller.skin.source;
						inst_geom.instance_material = node.controllers[ i ].instance_material;

						node.geometries.push( inst_geom );
						skinned = true;
						skinController = node.controllers[ i ];

					} else if ( controllers[ controller.skin.source ] ) {

						// urgh: controller can be chained
						// handle the most basic case...

						var second = controllers[ controller.skin.source ];
						morphController = second;
					//	skinController = node.controllers[i];

						if ( second.morph && geometries[ second.morph.source ] ) {

							var inst_geom = new InstanceGeometry();

							inst_geom.url = second.morph.source;
							inst_geom.instance_material = node.controllers[ i ].instance_material;

							node.geometries.push( inst_geom );

						}

					}

					break;

				case 'morph':

					if ( geometries[ controller.morph.source ] ) {

						var inst_geom = new InstanceGeometry();

						inst_geom.url = controller.morph.source;
						inst_geom.instance_material = node.controllers[ i ].instance_material;

						node.geometries.push( inst_geom );
						morphController = node.controllers[ i ];

					}

					console.log( 'ColladaLoader: Morph-controller partially supported.' );

				default:
					break;

			}

		}

		// FIXME: multi-material mesh?
		// geometries

		var double_sided_materials = {};

		for ( i = 0; i < node.geometries.length; i ++ ) {

			var instance_geometry = node.geometries[i];
			var instance_materials = instance_geometry.instance_material;
			var geometry = geometries[ instance_geometry.url ];
			var used_materials = {};
			var used_materials_array = [];
			var num_materials = 0;
			var first_material;

			if ( geometry ) {

				if ( !geometry.mesh || !geometry.mesh.primitives )
					continue;

				if ( obj.name.length == 0 ) {

					obj.name = geometry.id;

				}

				// collect used fx for this geometry-instance

				if ( instance_materials ) {

					for ( j = 0; j < instance_materials.length; j ++ ) {

						var instance_material = instance_materials[ j ];
						var mat = materials[ instance_material.target ];
						var effect_id = mat.instance_effect.url;
						var shader = effects[ effect_id ].shader;
						var material3js = shader.material;

						if ( geometry.doubleSided ) {

							if ( !( material3js in double_sided_materials ) ) {

								var _copied_material = material3js.clone();
								_copied_material.side = THREE.DoubleSide;
								double_sided_materials[ material3js ] = _copied_material;

							}

							material3js = double_sided_materials[ material3js ];

						}

						material3js.opacity = !material3js.opacity ? 1 : material3js.opacity;
						used_materials[ instance_material.symbol ] = num_materials;
						used_materials_array.push( material3js );
						first_material = material3js;
						first_material.name = mat.name == null || mat.name === '' ? mat.id : mat.name;
						num_materials ++;

					}

				}

				var mesh;
				var material = first_material || new THREE.MeshLambertMaterial( { color: 0xdddddd, shading: THREE.FlatShading, side: geometry.doubleSided ? THREE.DoubleSide : THREE.FrontSide } );
				var geom = geometry.mesh.geometry3js;

				if ( num_materials > 1 ) {

					material = new THREE.MeshFaceMaterial( used_materials_array );

					for ( j = 0; j < geom.faces.length; j ++ ) {

						var face = geom.faces[ j ];
						face.materialIndex = used_materials[ face.daeMaterial ]

					}

				}

				if ( skinController !== undefined ) {

					applySkin( geom, skinController );

					material.morphTargets = true;

					mesh = new THREE.SkinnedMesh( geom, material, false );
					mesh.skeleton = skinController.skeleton;
					mesh.skinController = controllers[ skinController.url ];
					mesh.skinInstanceController = skinController;
					mesh.name = 'skin_' + skins.length;

					skins.push( mesh );

				} else if ( morphController !== undefined ) {

					createMorph( geom, morphController );

					material.morphTargets = true;

					mesh = new THREE.Mesh( geom, material );
					mesh.name = 'morph_' + morphs.length;

					morphs.push( mesh );

				} else {

					mesh = new THREE.Mesh( geom, material );
					// mesh.geom.name = geometry.id;

				}

				node.geometries.length > 1 ? obj.add( mesh ) : obj = mesh;

			}

		}

		for ( i = 0; i < node.cameras.length; i ++ ) {

			var instance_camera = node.cameras[i];
			var cparams = cameras[instance_camera.url];

			obj = new THREE.PerspectiveCamera(cparams.fov, cparams.aspect_ratio, cparams.znear, cparams.zfar);

		}

		obj.name = node.id || "";
		obj.matrix = node.matrix;
		var props = node.matrix.decompose();
		obj.position = props[ 0 ];
		obj.quaternion = props[ 1 ];
		obj.useQuaternion = true;
		obj.scale = props[ 2 ];
    obj.position.multiplyScalar(colladaUnit);
		obj.scale.multiplyScalar(colladaUnit);

		if ( options.centerGeometry && obj.geometry ) {

			var delta = THREE.GeometryUtils.center( obj.geometry );
			obj.quaternion.multiplyVector3( delta.multiply( obj.scale ) );
			obj.position.sub( delta );

		}

		for ( i = 0; i < node.nodes.length; i ++ ) {

			obj.add( createSceneGraph( node.nodes[i], node ) );

		}

		return obj;

	};

	function getJointId( skin, id ) {

		for ( var i = 0; i < skin.joints.length; i ++ ) {

			if ( skin.joints[ i ] == id ) {

				return i;

			}

		}

	};

	function getLibraryNode( id ) {

		return COLLADA.evaluate( './/dae:library_nodes//dae:node[@id=\'' + id + '\']', COLLADA, _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null ).iterateNext();

	};

	function getChannelsForNode (node ) {

		var channels = [];
		var startTime = 1000000;
		var endTime = -1000000;

		for ( var id in animations ) {

			var animation = animations[id];

			for ( var i = 0; i < animation.channel.length; i ++ ) {

				var channel = animation.channel[i];
				var sampler = animation.sampler[i];
				var id = channel.target.split('/')[0];

				if ( id == node.id ) {

					sampler.create();
					channel.sampler = sampler;
					startTime = Math.min(startTime, sampler.startTime);
					endTime = Math.max(endTime, sampler.endTime);
					channels.push(channel);

				}

			}

		}

		if ( channels.length ) {

			node.startTime = startTime;
			node.endTime = endTime;

		}

		return channels;

	};

	function calcFrameDuration( node ) {

		var minT = 10000000;

		for ( var i = 0; i < node.channels.length; i ++ ) {

			var sampler = node.channels[i].sampler;

			for ( var j = 0; j < sampler.input.length - 1; j ++ ) {

				var t0 = sampler.input[ j ];
				var t1 = sampler.input[ j + 1 ];
				minT = Math.min( minT, t1 - t0 );

			}
		}

		return minT;

	};

	function calcMatrixAt( node, t ) {

		var animated = {};

		var i, j;

		for ( i = 0; i < node.channels.length; i ++ ) {

			var channel = node.channels[ i ];
			animated[ channel.sid ] = channel;

		}

		var matrix = new THREE.Matrix4();

		for ( i = 0; i < node.transforms.length; i ++ ) {

			var transform = node.transforms[ i ];
			var channel = animated[ transform.sid ];

			if ( channel !== undefined ) {

				var sampler = channel.sampler;
				var value;

				for ( j = 0; j < sampler.input.length - 1; j ++ ) {

					if ( sampler.input[ j + 1 ] > t ) {

						value = sampler.output[ j ];
						//console.log(value.flatten)
						break;

					}

				}

				if ( value !== undefined ) {

					if ( value instanceof THREE.Matrix4 ) {

						matrix = matrix.multiply( matrix, value );

					} else {

						// FIXME: handle other types

						matrix = matrix.multiply( matrix, transform.matrix );

					}

				} else {

					matrix = matrix.multiply( matrix, transform.matrix );

				}

			} else {

				matrix = matrix.multiply( matrix, transform.matrix );

			}

		}

		return matrix;

	};

	function bakeAnimations ( node ) {

		if ( node.channels && node.channels.length ) {

			var keys = [],
				sids = [];

			for ( var i = 0, il = node.channels.length; i < il; i++ ) {

				var channel = node.channels[i],
					fullSid = channel.fullSid,
					sampler = channel.sampler,
					input = sampler.input,
					transform = node.getTransformBySid( channel.sid ),
					member;

				if ( channel.arrIndices ) {

					member = [];

					for ( var j = 0, jl = channel.arrIndices.length; j < jl; j++ ) {

						member[ j ] = getConvertedIndex( channel.arrIndices[ j ] );

					}

				} else {

					member = getConvertedMember( channel.member );

				}

				if ( transform ) {

					if ( sids.indexOf( fullSid ) === -1 ) {

						sids.push( fullSid );

					}

					for ( var j = 0, jl = input.length; j < jl; j++ ) {

						var time = input[j],
							data = sampler.getData( transform.type, j ),
							key = findKey( keys, time );

						if ( !key ) {

							key = new Key( time );
							var timeNdx = findTimeNdx( keys, time );
							keys.splice( timeNdx == -1 ? keys.length : timeNdx, 0, key );

						}

						key.addTarget( fullSid, transform, member, data );

					}

				} else {

					console.log( 'Could not find transform "' + channel.sid + '" in node ' + node.id );

				}

			}

			// post process
			for ( var i = 0; i < sids.length; i++ ) {

				var sid = sids[ i ];

				for ( var j = 0; j < keys.length; j++ ) {

					var key = keys[ j ];

					if ( !key.hasTarget( sid ) ) {

						interpolateKeys( keys, key, j, sid );

					}

				}

			}

			node.keys = keys;
			node.sids = sids;

		}

	};

	function findKey ( keys, time) {

		var retVal = null;

		for ( var i = 0, il = keys.length; i < il && retVal == null; i++ ) {

			var key = keys[i];

			if ( key.time === time ) {

				retVal = key;

			} else if ( key.time > time ) {

				break;

			}

		}

		return retVal;

	};

	function findTimeNdx ( keys, time) {

		var ndx = -1;

		for ( var i = 0, il = keys.length; i < il && ndx == -1; i++ ) {

			var key = keys[i];

			if ( key.time >= time ) {

				ndx = i;

			}

		}

		return ndx;

	};

	function interpolateKeys ( keys, key, ndx, fullSid ) {

		var prevKey = getPrevKeyWith( keys, fullSid, ndx ? ndx-1 : 0 ),
			nextKey = getNextKeyWith( keys, fullSid, ndx+1 );

		if ( prevKey && nextKey ) {

			var scale = (key.time - prevKey.time) / (nextKey.time - prevKey.time),
				prevTarget = prevKey.getTarget( fullSid ),
				nextData = nextKey.getTarget( fullSid ).data,
				prevData = prevTarget.data,
				data;

			if ( prevTarget.type === 'matrix' ) {

				data = prevData;

			} else if ( prevData.length ) {

				data = [];

				for ( var i = 0; i < prevData.length; ++i ) {

					data[ i ] = prevData[ i ] + ( nextData[ i ] - prevData[ i ] ) * scale;

				}

			} else {

				data = prevData + ( nextData - prevData ) * scale;

			}

			key.addTarget( fullSid, prevTarget.transform, prevTarget.member, data );

		}

	};

	// Get next key with given sid

	function getNextKeyWith( keys, fullSid, ndx ) {

		for ( ; ndx < keys.length; ndx++ ) {

			var key = keys[ ndx ];

			if ( key.hasTarget( fullSid ) ) {

				return key;

			}

		}

		return null;

	};

	// Get previous key with given sid

	function getPrevKeyWith( keys, fullSid, ndx ) {

		ndx = ndx >= 0 ? ndx : ndx + keys.length;

		for ( ; ndx >= 0; ndx-- ) {

			var key = keys[ ndx ];

			if ( key.hasTarget( fullSid ) ) {

				return key;

			}

		}

		return null;

	};

	function _Image() {

		this.id = "";
		this.init_from = "";

	};

	_Image.prototype.parse = function(element) {

		this.id = element.getAttribute('id');

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];

			if ( child.nodeName == 'init_from' ) {

				this.init_from = child.textContent;

			}

		}

		return this;

	};

	function Controller() {

		this.id = "";
		this.name = "";
		this.type = "";
		this.skin = null;
		this.morph = null;

	};

	Controller.prototype.parse = function( element ) {

		this.id = element.getAttribute('id');
		this.name = element.getAttribute('name');
		this.type = "none";

		for ( var i = 0; i < element.childNodes.length; i++ ) {

			var child = element.childNodes[ i ];

			switch ( child.nodeName ) {

				case 'skin':

					this.skin = (new Skin()).parse(child);
					this.type = child.nodeName;
					break;

				case 'morph':

					this.morph = (new Morph()).parse(child);
					this.type = child.nodeName;
					break;

				default:
					break;

			}
		}

		return this;

	};

	function Morph() {

		this.method = null;
		this.source = null;
		this.targets = null;
		this.weights = null;

	};

	Morph.prototype.parse = function( element ) {

		var sources = {};
		var inputs = [];
		var i;

		this.method = element.getAttribute( 'method' );
		this.source = element.getAttribute( 'source' ).replace( /^#/, '' );

		for ( i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'source':

					var source = ( new Source() ).parse( child );
					sources[ source.id ] = source;
					break;

				case 'targets':

					inputs = this.parseInputs( child );
					break;

				default:

					console.log( child.nodeName );
					break;

			}

		}

		for ( i = 0; i < inputs.length; i ++ ) {

			var input = inputs[ i ];
			var source = sources[ input.source ];

			switch ( input.semantic ) {

				case 'MORPH_TARGET':

					this.targets = source.read();
					break;

				case 'MORPH_WEIGHT':

					this.weights = source.read();
					break;

				default:
					break;

			}
		}

		return this;

	};

	Morph.prototype.parseInputs = function(element) {

		var inputs = [];

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[i];
			if ( child.nodeType != 1) continue;

			switch ( child.nodeName ) {

				case 'input':

					inputs.push( (new Input()).parse(child) );
					break;

				default:
					break;
			}
		}

		return inputs;

	};

	function Skin() {

		this.source = "";
		this.bindShapeMatrix = null;
		this.invBindMatrices = [];
		this.joints = [];
		this.weights = [];

	};

	Skin.prototype.parse = function( element ) {

		var sources = {};
		var joints, weights;

		this.source = element.getAttribute( 'source' ).replace( /^#/, '' );
		this.invBindMatrices = [];
		this.joints = [];
		this.weights = [];

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[i];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'bind_shape_matrix':

					var f = _floats(child.textContent);
					this.bindShapeMatrix = getConvertedMat4( f );
					break;

				case 'source':

					var src = new Source().parse(child);
					sources[ src.id ] = src;
					break;

				case 'joints':

					joints = child;
					break;

				case 'vertex_weights':

					weights = child;
					break;

				default:

					console.log( child.nodeName );
					break;

			}
		}

		this.parseJoints( joints, sources );
		this.parseWeights( weights, sources );

		return this;

	};

	Skin.prototype.parseJoints = function ( element, sources ) {

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'input':

					var input = ( new Input() ).parse( child );
					var source = sources[ input.source ];

					if ( input.semantic == 'JOINT' ) {

						this.joints = source.read();

					} else if ( input.semantic == 'INV_BIND_MATRIX' ) {

						this.invBindMatrices = source.read();

					}

					break;

				default:
					break;
			}

		}

	};

	Skin.prototype.parseWeights = function ( element, sources ) {

		var v, vcount, inputs = [];

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'input':

					inputs.push( ( new Input() ).parse( child ) );
					break;

				case 'v':

					v = _ints( child.textContent );
					break;

				case 'vcount':

					vcount = _ints( child.textContent );
					break;

				default:
					break;

			}

		}

		var index = 0;

		for ( var i = 0; i < vcount.length; i ++ ) {

			var numBones = vcount[i];
			var vertex_weights = [];

			for ( var j = 0; j < numBones; j++ ) {

				var influence = {};

				for ( var k = 0; k < inputs.length; k ++ ) {

					var input = inputs[ k ];
					var value = v[ index + input.offset ];

					switch ( input.semantic ) {

						case 'JOINT':

							influence.joint = value;//this.joints[value];
							break;

						case 'WEIGHT':

							influence.weight = sources[ input.source ].data[ value ];
							break;

						default:
							break;

					}

				}

				vertex_weights.push( influence );
				index += inputs.length;
			}

			for ( var j = 0; j < vertex_weights.length; j ++ ) {

				vertex_weights[ j ].index = i;

			}

			this.weights.push( vertex_weights );

		}

	};

	function VisualScene () {

		this.id = "";
		this.name = "";
		this.nodes = [];
		this.scene = new THREE.Object3D();

	};

	VisualScene.prototype.getChildById = function( id, recursive ) {

		for ( var i = 0; i < this.nodes.length; i ++ ) {

			var node = this.nodes[ i ].getChildById( id, recursive );

			if ( node ) {

				return node;

			}

		}

		return null;

	};

	VisualScene.prototype.getChildBySid = function( sid, recursive ) {

		for ( var i = 0; i < this.nodes.length; i ++ ) {

			var node = this.nodes[ i ].getChildBySid( sid, recursive );

			if ( node ) {

				return node;

			}

		}

		return null;

	};

	VisualScene.prototype.parse = function( element ) {

		this.id = element.getAttribute( 'id' );
		this.name = element.getAttribute( 'name' );
		this.nodes = [];

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'node':

					this.nodes.push( ( new Node() ).parse( child ) );
					break;

				default:
					break;

			}

		}

		return this;

	};

	function Node() {

		this.id = "";
		this.name = "";
		this.sid = "";
		this.nodes = [];
		this.controllers = [];
		this.transforms = [];
		this.geometries = [];
		this.channels = [];
		this.matrix = new THREE.Matrix4();

	};

	Node.prototype.getChannelForTransform = function( transformSid ) {

		for ( var i = 0; i < this.channels.length; i ++ ) {

			var channel = this.channels[i];
			var parts = channel.target.split('/');
			var id = parts.shift();
			var sid = parts.shift();
			var dotSyntax = (sid.indexOf(".") >= 0);
			var arrSyntax = (sid.indexOf("(") >= 0);
			var arrIndices;
			var member;

			if ( dotSyntax ) {

				parts = sid.split(".");
				sid = parts.shift();
				member = parts.shift();

			} else if ( arrSyntax ) {

				arrIndices = sid.split("(");
				sid = arrIndices.shift();

				for ( var j = 0; j < arrIndices.length; j ++ ) {

					arrIndices[ j ] = parseInt( arrIndices[ j ].replace( /\)/, '' ) );

				}

			}

			if ( sid == transformSid ) {

				channel.info = { sid: sid, dotSyntax: dotSyntax, arrSyntax: arrSyntax, arrIndices: arrIndices };
				return channel;

			}

		}

		return null;

	};

	Node.prototype.getChildById = function ( id, recursive ) {

		if ( this.id == id ) {

			return this;

		}

		if ( recursive ) {

			for ( var i = 0; i < this.nodes.length; i ++ ) {

				var n = this.nodes[ i ].getChildById( id, recursive );

				if ( n ) {

					return n;

				}

			}

		}

		return null;

	};

	Node.prototype.getChildBySid = function ( sid, recursive ) {

		if ( this.sid == sid ) {

			return this;

		}

		if ( recursive ) {

			for ( var i = 0; i < this.nodes.length; i ++ ) {

				var n = this.nodes[ i ].getChildBySid( sid, recursive );

				if ( n ) {

					return n;

				}

			}
		}

		return null;

	};

	Node.prototype.getTransformBySid = function ( sid ) {

		for ( var i = 0; i < this.transforms.length; i ++ ) {

			if ( this.transforms[ i ].sid == sid ) return this.transforms[ i ];

		}

		return null;

	};

	Node.prototype.parse = function( element ) {

		var url;

		this.id = element.getAttribute('id');
		this.sid = element.getAttribute('sid');
		this.name = element.getAttribute('name');
		this.type = element.getAttribute('type');

		this.type = this.type == 'JOINT' ? this.type : 'NODE';

		this.nodes = [];
		this.transforms = [];
		this.geometries = [];
		this.cameras = [];
		this.controllers = [];
		this.matrix = new THREE.Matrix4();

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'node':

					this.nodes.push( ( new Node() ).parse( child ) );
					break;

				case 'instance_camera':

					this.cameras.push( ( new InstanceCamera() ).parse( child ) );
					break;

				case 'instance_controller':

					this.controllers.push( ( new InstanceController() ).parse( child ) );
					break;

				case 'instance_geometry':

					this.geometries.push( ( new InstanceGeometry() ).parse( child ) );
					break;

				case 'instance_light':

					break;

				case 'instance_node':

					url = child.getAttribute( 'url' ).replace( /^#/, '' );
					var iNode = getLibraryNode( url );

					if ( iNode ) {

						this.nodes.push( ( new Node() ).parse( iNode )) ;

					}

					break;

				case 'rotate':
				case 'translate':
				case 'scale':
				case 'matrix':
				case 'lookat':
				case 'skew':

					this.transforms.push( ( new Transform() ).parse( child ) );
					break;

				case 'extra':
					break;

				default:

					console.log( child.nodeName );
					break;

			}

		}

		this.channels = getChannelsForNode( this );
		bakeAnimations( this );

		this.updateMatrix();

		return this;

	};

	Node.prototype.updateMatrix = function () {

		this.matrix.identity();

		for ( var i = 0; i < this.transforms.length; i ++ ) {

			this.transforms[ i ].apply( this.matrix );

		}

	};

	function Transform () {

		this.sid = "";
		this.type = "";
		this.data = [];
		this.obj = null;

	};

	Transform.prototype.parse = function ( element ) {

		this.sid = element.getAttribute( 'sid' );
		this.type = element.nodeName;
		this.data = _floats( element.textContent );
		this.convert();

		return this;

	};

	Transform.prototype.convert = function () {

		switch ( this.type ) {

			case 'matrix':

				this.obj = getConvertedMat4( this.data );
				break;

			case 'rotate':

				this.angle = this.data[3] * TO_RADIANS;

			case 'translate':

				fixCoords( this.data, -1 );
				this.obj = new THREE.Vector3( this.data[ 0 ], this.data[ 1 ], this.data[ 2 ] );
				break;

			case 'scale':

				fixCoords( this.data, 1 );
				this.obj = new THREE.Vector3( this.data[ 0 ], this.data[ 1 ], this.data[ 2 ] );
				break;

			default:
				console.log( 'Can not convert Transform of type ' + this.type );
				break;

		}

	};

	Transform.prototype.apply = function ( matrix ) {

		switch ( this.type ) {

			case 'matrix':

				matrix.multiply( this.obj );
				break;

			case 'translate':

				matrix.translate( this.obj );
				break;

			case 'rotate':

				matrix.rotateByAxis( this.obj, this.angle );
				break;

			case 'scale':

				matrix.scale( this.obj );
				break;

		}

	};

	Transform.prototype.update = function ( data, member ) {

		var members = [ 'X', 'Y', 'Z', 'ANGLE' ];

		switch ( this.type ) {

			case 'matrix':

				if ( ! member ) {

					this.obj.copy( data );

				} else if ( member.length === 1 ) {

					switch ( member[ 0 ] ) {

						case 0:

							this.obj.n11 = data[ 0 ];
							this.obj.n21 = data[ 1 ];
							this.obj.n31 = data[ 2 ];
							this.obj.n41 = data[ 3 ];

							break;

						case 1:

							this.obj.n12 = data[ 0 ];
							this.obj.n22 = data[ 1 ];
							this.obj.n32 = data[ 2 ];
							this.obj.n42 = data[ 3 ];

							break;

						case 2:

							this.obj.n13 = data[ 0 ];
							this.obj.n23 = data[ 1 ];
							this.obj.n33 = data[ 2 ];
							this.obj.n43 = data[ 3 ];

							break;

						case 3:

							this.obj.n14 = data[ 0 ];
							this.obj.n24 = data[ 1 ];
							this.obj.n34 = data[ 2 ];
							this.obj.n44 = data[ 3 ];

							break;

					}

				} else if ( member.length === 2 ) {

					var propName = 'n' + ( member[ 0 ] + 1 ) + ( member[ 1 ] + 1 );
					this.obj[ propName ] = data;

				} else {

					console.log('Incorrect addressing of matrix in transform.');

				}

				break;

			case 'translate':
			case 'scale':

				if ( Object.prototype.toString.call( member ) === '[object Array]' ) {

					member = members[ member[ 0 ] ];

				}

				switch ( member ) {

					case 'X':

						this.obj.x = data;
						break;

					case 'Y':

						this.obj.y = data;
						break;

					case 'Z':

						this.obj.z = data;
						break;

					default:

						this.obj.x = data[ 0 ];
						this.obj.y = data[ 1 ];
						this.obj.z = data[ 2 ];
						break;

				}

				break;

			case 'rotate':

				if ( Object.prototype.toString.call( member ) === '[object Array]' ) {

					member = members[ member[ 0 ] ];

				}

				switch ( member ) {

					case 'X':

						this.obj.x = data;
						break;

					case 'Y':

						this.obj.y = data;
						break;

					case 'Z':

						this.obj.z = data;
						break;

					case 'ANGLE':

						this.angle = data * TO_RADIANS;
						break;

					default:

						this.obj.x = data[ 0 ];
						this.obj.y = data[ 1 ];
						this.obj.z = data[ 2 ];
						this.angle = data[ 3 ] * TO_RADIANS;
						break;

				}
				break;

		}

	};

	function InstanceController() {

		this.url = "";
		this.skeleton = [];
		this.instance_material = [];

	};

	InstanceController.prototype.parse = function ( element ) {

		this.url = element.getAttribute('url').replace(/^#/, '');
		this.skeleton = [];
		this.instance_material = [];

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType !== 1 ) continue;

			switch ( child.nodeName ) {

				case 'skeleton':

					this.skeleton.push( child.textContent.replace(/^#/, '') );
					break;

				case 'bind_material':

					var instances = COLLADA.evaluate( './/dae:instance_material', child, _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null );

					if ( instances ) {

						var instance = instances.iterateNext();

						while ( instance ) {

							this.instance_material.push( (new InstanceMaterial()).parse(instance) );
							instance = instances.iterateNext();

						}

					}

					break;

				case 'extra':
					break;

				default:
					break;

			}
		}

		return this;

	};

	function InstanceMaterial () {

		this.symbol = "";
		this.target = "";

	};

	InstanceMaterial.prototype.parse = function ( element ) {

		this.symbol = element.getAttribute('symbol');
		this.target = element.getAttribute('target').replace(/^#/, '');
		return this;

	};

	function InstanceGeometry() {

		this.url = "";
		this.instance_material = [];

	};

	InstanceGeometry.prototype.parse = function ( element ) {

		this.url = element.getAttribute('url').replace(/^#/, '');
		this.instance_material = [];

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[i];
			if ( child.nodeType != 1 ) continue;

			if ( child.nodeName == 'bind_material' ) {

				var instances = COLLADA.evaluate( './/dae:instance_material', child, _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null );

				if ( instances ) {

					var instance = instances.iterateNext();

					while ( instance ) {

						this.instance_material.push( (new InstanceMaterial()).parse(instance) );
						instance = instances.iterateNext();

					}

				}

				break;

			}

		}

		return this;

	};

	function Geometry() {

		this.id = "";
		this.mesh = null;

	};

	Geometry.prototype.parse = function ( element ) {

		this.id = element.getAttribute('id');

		extractDoubleSided( this, element );

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[i];

			switch ( child.nodeName ) {

				case 'mesh':

					this.mesh = (new Mesh(this)).parse(child);
					break;

				case 'extra':

					// console.log( child );
					break;

				default:
					break;
			}
		}

		return this;

	};

	function Mesh( geometry ) {

		this.geometry = geometry.id;
		this.primitives = [];
		this.vertices = null;
		this.geometry3js = null;

	};

	Mesh.prototype.parse = function( element ) {

		this.primitives = [];

		var i, j;

		for ( i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];

			switch ( child.nodeName ) {

				case 'source':

					_source( child );
					break;

				case 'vertices':

					this.vertices = ( new Vertices() ).parse( child );
					break;

				case 'triangles':

					this.primitives.push( ( new Triangles().parse( child ) ) );
					break;

				case 'polygons':

					this.primitives.push( ( new Polygons().parse( child ) ) );
					break;

				case 'polylist':

					this.primitives.push( ( new Polylist().parse( child ) ) );
					break;

				default:
					break;

			}

		}

		this.geometry3js = new THREE.Geometry();

		var vertexData = sources[ this.vertices.input['POSITION'].source ].data;

		for ( i = 0; i < vertexData.length; i += 3 ) {

			this.geometry3js.vertices.push( getConvertedVec3( vertexData, i ).clone() );

		}

		for ( i = 0; i < this.primitives.length; i ++ ) {

			var primitive = this.primitives[ i ];
			primitive.setVertices( this.vertices );
			this.handlePrimitive( primitive, this.geometry3js );

		}

		this.geometry3js.computeCentroids();
		this.geometry3js.computeFaceNormals();

		if ( this.geometry3js.calcNormals ) {

			this.geometry3js.computeVertexNormals();
			delete this.geometry3js.calcNormals;

		}

		this.geometry3js.computeBoundingBox();

		return this;

	};

	Mesh.prototype.handlePrimitive = function( primitive, geom ) {

		var j, k, pList = primitive.p, inputs = primitive.inputs;
		var input, index, idx32;
		var source, numParams;
		var vcIndex = 0, vcount = 3, maxOffset = 0;
		var texture_sets = [];

		for ( j = 0; j < inputs.length; j ++ ) {

			input = inputs[ j ];
			var offset = input.offset + 1;
			maxOffset = (maxOffset < offset)? offset : maxOffset;

			switch ( input.semantic ) {

				case 'TEXCOORD':
					texture_sets.push( input.set );
					break;

			}

		}

		for ( var pCount = 0; pCount < pList.length; ++pCount ) {

			var p = pList[ pCount ], i = 0;

			while ( i < p.length ) {

				var vs = [];
				var ns = [];
				var ts = null;
				var cs = [];

				if ( primitive.vcount ) {

					vcount = primitive.vcount.length ? primitive.vcount[ vcIndex ++ ] : primitive.vcount;

				} else {

					vcount = p.length / maxOffset;

				}


				for ( j = 0; j < vcount; j ++ ) {

					for ( k = 0; k < inputs.length; k ++ ) {

						input = inputs[ k ];
						source = sources[ input.source ];

						index = p[ i + ( j * maxOffset ) + input.offset ];
						numParams = source.accessor.params.length;
						idx32 = index * numParams;

						switch ( input.semantic ) {

							case 'VERTEX':

								vs.push( index );

								break;

							case 'NORMAL':

								ns.push( getConvertedVec3( source.data, idx32 ) );

								break;

							case 'TEXCOORD':

								ts = ts || { };
								if ( ts[ input.set ] === undefined ) ts[ input.set ] = [];
								// invert the V
								ts[ input.set ].push( new THREE.Vector2( source.data[ idx32 ], source.data[ idx32 + 1 ] ) );

								break;

							case 'COLOR':

								cs.push( new THREE.Color().setRGB( source.data[ idx32 ], source.data[ idx32 + 1 ], source.data[ idx32 + 2 ] ) );

								break;

							default:

								break;

						}

					}

				}

				if ( ns.length == 0 ) {

					// check the vertices inputs
					input = this.vertices.input.NORMAL;

					if ( input ) {

						source = sources[ input.source ];
						numParams = source.accessor.params.length;

						for ( var ndx = 0, len = vs.length; ndx < len; ndx++ ) {

							ns.push( getConvertedVec3( source.data, vs[ ndx ] * numParams ) );

						}

					} else {

						geom.calcNormals = true;

					}

				}

				if ( !ts ) {

					ts = { };
					// check the vertices inputs
					input = this.vertices.input.TEXCOORD;

					if ( input ) {

						texture_sets.push( input.set );
						source = sources[ input.source ];
						numParams = source.accessor.params.length;

						for ( var ndx = 0, len = vs.length; ndx < len; ndx++ ) {

							idx32 = vs[ ndx ] * numParams;
							if ( ts[ input.set ] === undefined ) ts[ input.set ] = [ ];
							// invert the V
							ts[ input.set ].push( new THREE.Vector2( source.data[ idx32 ], 1.0 - source.data[ idx32 + 1 ] ) );

						}

					}

				}

				if ( cs.length == 0 ) {

					// check the vertices inputs
					input = this.vertices.input.COLOR;

					if ( input ) {

						source = sources[ input.source ];
						numParams = source.accessor.params.length;

						for ( var ndx = 0, len = vs.length; ndx < len; ndx++ ) {

							idx32 = vs[ ndx ] * numParams;
							cs.push( new THREE.Color().setRGB( source.data[ idx32 ], source.data[ idx32 + 1 ], source.data[ idx32 + 2 ] ) );

						}

					}

				}

				var face = null, faces = [], uv, uvArr;

				if ( vcount === 3 ) {

					faces.push( new THREE.Face3( vs[0], vs[1], vs[2], ns, cs.length ? cs : new THREE.Color() ) );

				} else if ( vcount === 4 ) {
					faces.push( new THREE.Face4( vs[0], vs[1], vs[2], vs[3], ns, cs.length ? cs : new THREE.Color() ) );

				} else if ( vcount > 4 && options.subdivideFaces ) {

					var clr = cs.length ? cs : new THREE.Color(),
						vec1, vec2, vec3, v1, v2, norm;

					// subdivide into multiple Face3s

					for ( k = 1; k < vcount - 1; ) {

						// FIXME: normals don't seem to be quite right

						faces.push( new THREE.Face3( vs[0], vs[k], vs[k+1], [ ns[0], ns[k++], ns[k] ],  clr ) );

					}

				}

				if ( faces.length ) {

					for ( var ndx = 0, len = faces.length; ndx < len; ndx ++ ) {

						face = faces[ndx];
						face.daeMaterial = primitive.material;
						geom.faces.push( face );

						for ( k = 0; k < texture_sets.length; k++ ) {

							uv = ts[ texture_sets[k] ];

							if ( vcount > 4 ) {

								// Grab the right UVs for the vertices in this face
								uvArr = [ uv[0], uv[ndx+1], uv[ndx+2] ];

							} else if ( vcount === 4 ) {

								uvArr = [ uv[0], uv[1], uv[2], uv[3] ];

							} else {

								uvArr = [ uv[0], uv[1], uv[2] ];

							}

							if ( !geom.faceVertexUvs[k] ) {

								geom.faceVertexUvs[k] = [];

							}

							geom.faceVertexUvs[k].push( uvArr );

						}

					}

				} else {

					console.log( 'dropped face with vcount ' + vcount + ' for geometry with id: ' + geom.id );

				}

				i += maxOffset * vcount;

			}
		}

	};

	function Polygons () {

		this.material = "";
		this.count = 0;
		this.inputs = [];
		this.vcount = null;
		this.p = [];
		this.geometry = new THREE.Geometry();

	};

	Polygons.prototype.setVertices = function ( vertices ) {

		for ( var i = 0; i < this.inputs.length; i ++ ) {

			if ( this.inputs[ i ].source == vertices.id ) {

				this.inputs[ i ].source = vertices.input[ 'POSITION' ].source;

			}

		}

	};

	Polygons.prototype.parse = function ( element ) {

		this.material = element.getAttribute( 'material' );
		this.count = _attr_as_int( element, 'count', 0 );

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];

			switch ( child.nodeName ) {

				case 'input':

					this.inputs.push( ( new Input() ).parse( element.childNodes[ i ] ) );
					break;

				case 'vcount':

					this.vcount = _ints( child.textContent );
					break;

				case 'p':

					this.p.push( _ints( child.textContent ) );
					break;

				case 'ph':

					console.warn( 'polygon holes not yet supported!' );
					break;

				default:
					break;

			}

		}

		return this;

	};

	function Polylist () {

		Polygons.call( this );

		this.vcount = [];

	};

	Polylist.prototype = Object.create( Polygons.prototype );

	function Triangles () {

		Polygons.call( this );

		this.vcount = 3;

	};

	Triangles.prototype = Object.create( Polygons.prototype );

	function Accessor() {

		this.source = "";
		this.count = 0;
		this.stride = 0;
		this.params = [];

	};

	Accessor.prototype.parse = function ( element ) {

		this.params = [];
		this.source = element.getAttribute( 'source' );
		this.count = _attr_as_int( element, 'count', 0 );
		this.stride = _attr_as_int( element, 'stride', 0 );

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];

			if ( child.nodeName == 'param' ) {

				var param = {};
				param[ 'name' ] = child.getAttribute( 'name' );
				param[ 'type' ] = child.getAttribute( 'type' );
				this.params.push( param );

			}

		}

		return this;

	};

	function Vertices() {

		this.input = {};

	};

	Vertices.prototype.parse = function ( element ) {

		this.id = element.getAttribute('id');

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			if ( element.childNodes[i].nodeName == 'input' ) {

				var input = ( new Input() ).parse( element.childNodes[ i ] );
				this.input[ input.semantic ] = input;

			}

		}

		return this;

	};

	function Input () {

		this.semantic = "";
		this.offset = 0;
		this.source = "";
		this.set = 0;

	};

	Input.prototype.parse = function ( element ) {

		this.semantic = element.getAttribute('semantic');
		this.source = element.getAttribute('source').replace(/^#/, '');
		this.set = _attr_as_int(element, 'set', -1);
		this.offset = _attr_as_int(element, 'offset', 0);

		if ( this.semantic == 'TEXCOORD' && this.set < 0 ) {

			this.set = 0;

		}

		return this;

	};

	function Source ( id ) {

		this.id = id;
		this.type = null;

	};

	Source.prototype.parse = function ( element ) {

		this.id = element.getAttribute( 'id' );

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[i];

			switch ( child.nodeName ) {

				case 'bool_array':

					this.data = _bools( child.textContent );
					this.type = child.nodeName;
					break;

				case 'float_array':

					this.data = _floats( child.textContent );
					this.type = child.nodeName;
					break;

				case 'int_array':

					this.data = _ints( child.textContent );
					this.type = child.nodeName;
					break;

				case 'IDREF_array':
				case 'Name_array':

					this.data = _strings( child.textContent );
					this.type = child.nodeName;
					break;

				case 'technique_common':

					for ( var j = 0; j < child.childNodes.length; j ++ ) {

						if ( child.childNodes[ j ].nodeName == 'accessor' ) {

							this.accessor = ( new Accessor() ).parse( child.childNodes[ j ] );
							break;

						}
					}
					break;

				default:
					// console.log(child.nodeName);
					break;

			}

		}

		return this;

	};

	Source.prototype.read = function () {

		var result = [];

		//for (var i = 0; i < this.accessor.params.length; i++) {

			var param = this.accessor.params[ 0 ];

			//console.log(param.name + " " + param.type);

			switch ( param.type ) {

				case 'IDREF':
				case 'Name': case 'name':
				case 'float':

					return this.data;

				case 'float4x4':

					for ( var j = 0; j < this.data.length; j += 16 ) {

						var s = this.data.slice( j, j + 16 );
						var m = getConvertedMat4( s );
						result.push( m );
					}

					break;

				default:

					console.log( 'ColladaLoader: Source: Read dont know how to read ' + param.type + '.' );
					break;

			}

		//}

		return result;

	};

	function Material () {

		this.id = "";
		this.name = "";
		this.instance_effect = null;

	};

	Material.prototype.parse = function ( element ) {

		this.id = element.getAttribute( 'id' );
		this.name = element.getAttribute( 'name' );

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			if ( element.childNodes[ i ].nodeName == 'instance_effect' ) {

				this.instance_effect = ( new InstanceEffect() ).parse( element.childNodes[ i ] );
				break;

			}

		}

		return this;

	};

	function ColorOrTexture () {

		this.color = new THREE.Color( 0 );
		this.color.setRGB( Math.random(), Math.random(), Math.random() );
		this.color.a = 1.0;

		this.texture = null;
		this.texcoord = null;
		this.texOpts = null;

	};

	ColorOrTexture.prototype.isColor = function () {

		return ( this.texture == null );

	};

	ColorOrTexture.prototype.isTexture = function () {

		return ( this.texture != null );

	};

	ColorOrTexture.prototype.parse = function ( element ) {

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'color':

					var rgba = _floats( child.textContent );
					this.color = new THREE.Color(0);
					this.color.setRGB( rgba[0], rgba[1], rgba[2] );
					this.color.a = rgba[3];
					break;

				case 'texture':

					this.texture = child.getAttribute('texture');
					this.texcoord = child.getAttribute('texcoord');
					// Defaults from:
					// https://collada.org/mediawiki/index.php/Maya_texture_placement_MAYA_extension
					this.texOpts = {
						offsetU: 0,
						offsetV: 0,
						repeatU: 1,
						repeatV: 1,
						wrapU: 1,
						wrapV: 1
					};
					this.parseTexture( child );
					break;

				default:
					break;

			}

		}

		return this;

	};

	ColorOrTexture.prototype.parseTexture = function ( element ) {

		if ( ! element.childNodes ) return this;

		// This should be supported by Maya, 3dsMax, and MotionBuilder

		if ( element.childNodes[1] && element.childNodes[1].nodeName === 'extra' ) {

			element = element.childNodes[1];

			if ( element.childNodes[1] && element.childNodes[1].nodeName === 'technique' ) {

				element = element.childNodes[1];

			}

		}

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];

			switch ( child.nodeName ) {

				case 'offsetU':
				case 'offsetV':
				case 'repeatU':
				case 'repeatV':

					this.texOpts[ child.nodeName ] = parseFloat( child.textContent );
					break;

				case 'wrapU':
				case 'wrapV':

					this.texOpts[ child.nodeName ] = parseInt( child.textContent );
					break;

				default:
					this.texOpts[ child.nodeName ] = child.textContent;
					break;

			}

		}

		return this;

	};

	function Shader ( type, effect ) {

		this.type = type;
		this.effect = effect;
		this.material = null;

	};

	Shader.prototype.parse = function ( element ) {

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'ambient':
				case 'emission':
				case 'diffuse':
				case 'specular':
				case 'transparent':

					this[ child.nodeName ] = ( new ColorOrTexture() ).parse( child );
					break;

				case 'shininess':
				case 'reflectivity':
				case 'index_of_refraction':
				case 'transparency':

					var f = evaluateXPath( child, './/dae:float' );

					if ( f.length > 0 )
						this[ child.nodeName ] = parseFloat( f[ 0 ].textContent );

					break;

				default:
					break;

			}

		}

		this.create();
		return this;

	};

	Shader.prototype.create = function() {

		var props = {};
		var transparent = ( this['transparency'] !== undefined && this['transparency'] < 1.0 );

		for ( var prop in this ) {

			switch ( prop ) {

				case 'ambient':
				case 'emission':
				case 'diffuse':
				case 'specular':

					var cot = this[ prop ];

					if ( cot instanceof ColorOrTexture ) {

						if ( cot.isTexture() ) {

							var samplerId = cot.texture;
							var surfaceId = this.effect.sampler[samplerId].source;
              

							if (surfaceId) {

								var surface = this.effect.surface[surfaceId];
								var image = images[surface.init_from];

								if (image) {

									var texture = THREE.ImageUtils.loadTexture(baseUrl + image.init_from);
									texture.wrapS = cot.texOpts.wrapU ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;
									texture.wrapT = cot.texOpts.wrapV ? THREE.RepeatWrapping : THREE.ClampToEdgeWrapping;
									texture.offset.x = cot.texOpts.offsetU;
									texture.offset.y = cot.texOpts.offsetV;
									texture.repeat.x = cot.texOpts.repeatU;
									texture.repeat.y = cot.texOpts.repeatV;
									props['map'] = texture;

									// Texture with baked lighting?
									if (prop === 'emission') props['emissive'] = 0xffffff;

								}

							}

						} else if ( prop === 'diffuse' || !transparent ) {

							if ( prop === 'emission' ) {

								props[ 'emissive' ] = cot.color.getHex();

							} else {

								props[ prop ] = cot.color.getHex();

							}

						}

					}

					break;

				case 'shininess':

					props[ prop ] = this[ prop ];
					break;

				case 'reflectivity':

					props[ prop ] = this[ prop ];
					if( props[ prop ] > 0.0 ) props['envMap'] = options.defaultEnvMap;
					props['combine'] = THREE.MixOperation;	//mix regular shading with reflective component
					break;

				case 'index_of_refraction':

					props[ 'refractionRatio' ] = this[ prop ]; //TODO: "index_of_refraction" becomes "refractionRatio" in shader, but I'm not sure if the two are actually comparable
					if ( this[ prop ] !== 1.0 ) props['envMap'] = options.defaultEnvMap;
					break;

				case 'transparency':

					if ( transparent ) {

						props[ 'transparent' ] = true;
						props[ 'opacity' ] = this[ prop ];
						transparent = true;

					}

					break;

				default:
					break;

			}

		}

		props[ 'shading' ] = preferredShading;
		props[ 'side' ] = this.effect.doubleSided ? THREE.DoubleSide : THREE.FrontSide;

		switch ( this.type ) {

			case 'constant':

				if (props.emissive != undefined) props.color = props.emissive;
				this.material = new THREE.MeshBasicMaterial( props );
				break;

			case 'phong':
			case 'blinn':

				if (props.diffuse != undefined) props.color = props.diffuse;
				this.material = new THREE.MeshPhongMaterial( props );
				break;

			case 'lambert':
			default:

				if (props.diffuse != undefined) props.color = props.diffuse;
				this.material = new THREE.MeshLambertMaterial( props );
				break;

		}

		return this.material;

	};

	function Surface ( effect ) {

		this.effect = effect;
		this.init_from = null;
		this.format = null;

	};

	Surface.prototype.parse = function ( element ) {

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'init_from':

					this.init_from = child.textContent;
					break;

				case 'format':

					this.format = child.textContent;
					break;

				default:

					console.log( "unhandled Surface prop: " + child.nodeName );
					break;

			}

		}

		return this;

	};

	function Sampler2D ( effect ) {

		this.effect = effect;
		this.source = null;
		this.wrap_s = null;
		this.wrap_t = null;
		this.minfilter = null;
		this.magfilter = null;
		this.mipfilter = null;

	};

	Sampler2D.prototype.parse = function ( element ) {

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'source':

					this.source = child.textContent;
					break;

				case 'minfilter':

					this.minfilter = child.textContent;
					break;

				case 'magfilter':

					this.magfilter = child.textContent;
					break;

				case 'mipfilter':

					this.mipfilter = child.textContent;
					break;

				case 'wrap_s':

					this.wrap_s = child.textContent;
					break;

				case 'wrap_t':

					this.wrap_t = child.textContent;
					break;

				default:

					console.log( "unhandled Sampler2D prop: " + child.nodeName );
					break;

			}

		}

		return this;

	};

	function Effect () {

		this.id = "";
		this.name = "";
		this.shader = null;
		this.surface = {};
		this.sampler = {};

	};

	Effect.prototype.create = function () {

		if ( this.shader == null ) {

			return null;

		}

	};

	Effect.prototype.parse = function ( element ) {

		this.id = element.getAttribute( 'id' );
		this.name = element.getAttribute( 'name' );

		extractDoubleSided( this, element );

		this.shader = null;

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'profile_COMMON':

					this.parseTechnique( this.parseProfileCOMMON( child ) );
					break;

				default:
					break;

			}

		}

		return this;

	};

	Effect.prototype.parseNewparam = function ( element ) {

		var sid = element.getAttribute( 'sid' );

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'surface':

					this.surface[sid] = ( new Surface( this ) ).parse( child );
					break;

				case 'sampler2D':

					this.sampler[sid] = ( new Sampler2D( this ) ).parse( child );
					break;

				case 'extra':

					break;

				default:

					console.log( child.nodeName );
					break;

			}

		}

	};

	Effect.prototype.parseProfileCOMMON = function ( element ) {

		var technique;

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];

			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'profile_COMMON':

					this.parseProfileCOMMON( child );
					break;

				case 'technique':

					technique = child;
					break;

				case 'newparam':

					this.parseNewparam( child );
					break;

				case 'image':

					var _image = ( new _Image() ).parse( child );
					images[ _image.id ] = _image;
					break;

				case 'extra':
					break;

				default:

					console.log( child.nodeName );
					break;

			}

		}

		return technique;

	};

	Effect.prototype.parseTechnique= function ( element ) {

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[i];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'constant':
				case 'lambert':
				case 'blinn':
				case 'phong':

					this.shader = ( new Shader( child.nodeName, this ) ).parse( child );
					break;

				default:
					break;

			}

		}

	};

	function InstanceEffect () {

		this.url = "";

	};

	InstanceEffect.prototype.parse = function ( element ) {

		this.url = element.getAttribute( 'url' ).replace( /^#/, '' );
		return this;

	};

	function Animation() {

		this.id = "";
		this.name = "";
		this.source = {};
		this.sampler = [];
		this.channel = [];

	};

	Animation.prototype.parse = function ( element ) {

		this.id = element.getAttribute( 'id' );
		this.name = element.getAttribute( 'name' );
		this.source = {};

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];

			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'animation':

					var anim = ( new Animation() ).parse( child );

					for ( var src in anim.source ) {

						this.source[ src ] = anim.source[ src ];

					}

					for ( var j = 0; j < anim.channel.length; j ++ ) {

						this.channel.push( anim.channel[ j ] );
						this.sampler.push( anim.sampler[ j ] );

					}

					break;

				case 'source':

					var src = ( new Source() ).parse( child );
					this.source[ src.id ] = src;
					break;

				case 'sampler':

					this.sampler.push( ( new Sampler( this ) ).parse( child ) );
					break;

				case 'channel':

					this.channel.push( ( new Channel( this ) ).parse( child ) );
					break;

				default:
					break;

			}

		}

		return this;

	};

	function Channel( animation ) {

		this.animation = animation;
		this.source = "";
		this.target = "";
		this.fullSid = null;
		this.sid = null;
		this.dotSyntax = null;
		this.arrSyntax = null;
		this.arrIndices = null;
		this.member = null;

	};

	Channel.prototype.parse = function ( element ) {

		this.source = element.getAttribute( 'source' ).replace( /^#/, '' );
		this.target = element.getAttribute( 'target' );

		var parts = this.target.split( '/' );

		var id = parts.shift();
		var sid = parts.shift();

		var dotSyntax = ( sid.indexOf(".") >= 0 );
		var arrSyntax = ( sid.indexOf("(") >= 0 );

		if ( dotSyntax ) {

			parts = sid.split(".");
			this.sid = parts.shift();
			this.member = parts.shift();

		} else if ( arrSyntax ) {

			var arrIndices = sid.split("(");
			this.sid = arrIndices.shift();

			for (var j = 0; j < arrIndices.length; j ++ ) {

				arrIndices[j] = parseInt( arrIndices[j].replace(/\)/, '') );

			}

			this.arrIndices = arrIndices;

		} else {

			this.sid = sid;

		}

		this.fullSid = sid;
		this.dotSyntax = dotSyntax;
		this.arrSyntax = arrSyntax;

		return this;

	};

	function Sampler ( animation ) {

		this.id = "";
		this.animation = animation;
		this.inputs = [];
		this.input = null;
		this.output = null;
		this.strideOut = null;
		this.interpolation = null;
		this.startTime = null;
		this.endTime = null;
		this.duration = 0;

	};

	Sampler.prototype.parse = function ( element ) {

		this.id = element.getAttribute( 'id' );
		this.inputs = [];

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'input':

					this.inputs.push( (new Input()).parse( child ) );
					break;

				default:
					break;

			}

		}

		return this;

	};

	Sampler.prototype.create = function () {

		for ( var i = 0; i < this.inputs.length; i ++ ) {

			var input = this.inputs[ i ];
			var source = this.animation.source[ input.source ];

			switch ( input.semantic ) {

				case 'INPUT':

					this.input = source.read();
					break;

				case 'OUTPUT':

					this.output = source.read();
					this.strideOut = source.accessor.stride;
					break;

				case 'INTERPOLATION':

					this.interpolation = source.read();
					break;

				case 'IN_TANGENT':

					break;

				case 'OUT_TANGENT':

					break;

				default:

					console.log(input.semantic);
					break;

			}

		}

		this.startTime = 0;
		this.endTime = 0;
		this.duration = 0;

		if ( this.input.length ) {

			this.startTime = 100000000;
			this.endTime = -100000000;

			for ( var i = 0; i < this.input.length; i ++ ) {

				this.startTime = Math.min( this.startTime, this.input[ i ] );
				this.endTime = Math.max( this.endTime, this.input[ i ] );

			}

			this.duration = this.endTime - this.startTime;

		}

	};

	Sampler.prototype.getData = function ( type, ndx ) {

		var data;

		if ( type === 'matrix' && this.strideOut === 16 ) {

			data = this.output[ ndx ];

		} else if ( this.strideOut > 1 ) {

			data = [];
			ndx *= this.strideOut;

			for ( var i = 0; i < this.strideOut; ++i ) {

				data[ i ] = this.output[ ndx + i ];

			}

			if ( this.strideOut === 3 ) {

				switch ( type ) {

					case 'rotate':
					case 'translate':

						fixCoords( data, -1 );
						break;

					case 'scale':

						fixCoords( data, 1 );
						break;

				}

			} else if ( this.strideOut === 4 && type === 'matrix' ) {

				fixCoords( data, -1 );

			}

		} else {

			data = this.output[ ndx ];

		}

		return data;

	};

	function Key ( time ) {

		this.targets = [];
		this.time = time;

	};

	Key.prototype.addTarget = function ( fullSid, transform, member, data ) {

		this.targets.push( {
			sid: fullSid,
			member: member,
			transform: transform,
			data: data
		} );

	};

	Key.prototype.apply = function ( opt_sid ) {

		for ( var i = 0; i < this.targets.length; ++i ) {

			var target = this.targets[ i ];

			if ( !opt_sid || target.sid === opt_sid ) {

				target.transform.update( target.data, target.member );

			}

		}

	};

	Key.prototype.getTarget = function ( fullSid ) {

		for ( var i = 0; i < this.targets.length; ++i ) {

			if ( this.targets[ i ].sid === fullSid ) {

				return this.targets[ i ];

			}

		}

		return null;

	};

	Key.prototype.hasTarget = function ( fullSid ) {

		for ( var i = 0; i < this.targets.length; ++i ) {

			if ( this.targets[ i ].sid === fullSid ) {

				return true;

			}

		}

		return false;

	};

	// TODO: Currently only doing linear interpolation. Should support full COLLADA spec.
	Key.prototype.interpolate = function ( nextKey, time ) {

		for ( var i = 0; i < this.targets.length; ++i ) {

			var target = this.targets[ i ],
				nextTarget = nextKey.getTarget( target.sid ),
				data;

			if ( target.transform.type !== 'matrix' && nextTarget ) {

				var scale = ( time - this.time ) / ( nextKey.time - this.time ),
					nextData = nextTarget.data,
					prevData = target.data;

				// check scale error

				if ( scale < 0 || scale > 1 ) {

					console.log( "Key.interpolate: Warning! Scale out of bounds:" + scale );
					scale = scale < 0 ? 0 : 1;

				}

				if ( prevData.length ) {

					data = [];

					for ( var j = 0; j < prevData.length; ++j ) {

						data[ j ] = prevData[ j ] + ( nextData[ j ] - prevData[ j ] ) * scale;

					}

				} else {

					data = prevData + ( nextData - prevData ) * scale;

				}

			} else {

				data = target.data;

			}

			target.transform.update( data, target.member );

		}

	};

	function Camera() {

		this.id = "";
		this.name = "";
		this.technique = "";

	};

	Camera.prototype.parse = function ( element ) {

		this.id = element.getAttribute( 'id' );
		this.name = element.getAttribute( 'name' );

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			var child = element.childNodes[ i ];
			if ( child.nodeType != 1 ) continue;

			switch ( child.nodeName ) {

				case 'optics':

					this.parseOptics( child );
					break;

				default:
					break;

			}

		}

		return this;

	};

	Camera.prototype.parseOptics = function ( element ) {

		for ( var i = 0; i < element.childNodes.length; i ++ ) {

			if ( element.childNodes[ i ].nodeName == 'technique_common' ) {

				var technique = element.childNodes[ i ];

				for ( var j = 0; j < technique.childNodes.length; j ++ ) {

					this.technique = technique.childNodes[ j ].nodeName;

					if ( this.technique == 'perspective' ) {

						var perspective = technique.childNodes[ j ];

						for ( var k = 0; k < perspective.childNodes.length; k ++ ) {

							var param = perspective.childNodes[ k ];

							switch ( param.nodeName ) {

								case 'yfov':
									this.yfov = param.textContent;
									break;
								case 'xfov':
									this.xfov = param.textContent;
									break;
								case 'znear':
									this.znear = param.textContent;
									break;
								case 'zfar':
									this.zfar = param.textContent;
									break;
								case 'aspect_ratio':
									this.aspect_ratio = param.textContent;
									break;

							}

						}

					} else if ( this.technique == 'orthographic' ) {

						var orthographic = technique.childNodes[ j ];

						for ( var k = 0; k < orthographic.childNodes.length; k ++ ) {

							var param = orthographic.childNodes[ k ];

							switch ( param.nodeName ) {

								case 'xmag':
									this.xmag = param.textContent;
									break;
								case 'ymag':
									this.ymag = param.textContent;
									break;
								case 'znear':
									this.znear = param.textContent;
									break;
								case 'zfar':
									this.zfar = param.textContent;
									break;
								case 'aspect_ratio':
									this.aspect_ratio = param.textContent;
									break;

							}

						}

					}

				}

			}

		}

		return this;

	};

	function InstanceCamera() {

		this.url = "";

	};

	InstanceCamera.prototype.parse = function ( element ) {

		this.url = element.getAttribute('url').replace(/^#/, '');

		return this;

	};

	function _source( element ) {

		var id = element.getAttribute( 'id' );

		if ( sources[ id ] != undefined ) {

			return sources[ id ];

		}

		sources[ id ] = ( new Source(id )).parse( element );
		return sources[ id ];

	};

	function _nsResolver( nsPrefix ) {

		if ( nsPrefix == "dae" ) {

			return "http://www.collada.org/2005/11/COLLADASchema";

		}

		return null;

	};

	function _bools( str ) {

		var raw = _strings( str );
		var data = [];

		for ( var i = 0, l = raw.length; i < l; i ++ ) {

			data.push( (raw[i] == 'true' || raw[i] == '1') ? true : false );

		}

		return data;

	};

	function _floats( str ) {

		var raw = _strings(str);
		var data = [];

		for ( var i = 0, l = raw.length; i < l; i ++ ) {

			data.push( parseFloat( raw[ i ] ) );

		}

		return data;

	};

	function _ints( str ) {

		var raw = _strings( str );
		var data = [];

		for ( var i = 0, l = raw.length; i < l; i ++ ) {

			data.push( parseInt( raw[ i ], 10 ) );

		}

		return data;

	};

	function _strings( str ) {

		return ( str.length > 0 ) ? _trimString( str ).split( /\s+/ ) : [];

	};

	function _trimString( str ) {

		return str.replace( /^\s+/, "" ).replace( /\s+$/, "" );

	};

	function _attr_as_float( element, name, defaultValue ) {

		if ( element.hasAttribute( name ) ) {

			return parseFloat( element.getAttribute( name ) );

		} else {

			return defaultValue;

		}

	};

	function _attr_as_int( element, name, defaultValue ) {

		if ( element.hasAttribute( name ) ) {

			return parseInt( element.getAttribute( name ), 10) ;

		} else {

			return defaultValue;

		}

	};

	function _attr_as_string( element, name, defaultValue ) {

		if ( element.hasAttribute( name ) ) {

			return element.getAttribute( name );

		} else {

			return defaultValue;

		}

	};

	function _format_float( f, num ) {

		if ( f === undefined ) {

			var s = '0.';

			while ( s.length < num + 2 ) {

				s += '0';

			}

			return s;

		}

		num = num || 2;

		var parts = f.toString().split( '.' );
		parts[ 1 ] = parts.length > 1 ? parts[ 1 ].substr( 0, num ) : "0";

		while( parts[ 1 ].length < num ) {

			parts[ 1 ] += '0';

		}

		return parts.join( '.' );

	};

	function evaluateXPath( node, query ) {

		var instances = COLLADA.evaluate( query, node, _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null );

		var inst = instances.iterateNext();
		var result = [];

		while ( inst ) {

			result.push( inst );
			inst = instances.iterateNext();

		}

		return result;

	};

	function extractDoubleSided( obj, element ) {

		obj.doubleSided = false;

		var node = COLLADA.evaluate( './/dae:extra//dae:double_sided', element, _nsResolver, XPathResult.ORDERED_NODE_ITERATOR_TYPE, null );

		if ( node ) {

			node = node.iterateNext();

			if ( node && parseInt( node.textContent, 10 ) === 1 ) {

				obj.doubleSided = true;

			}

		}

	};

	// Up axis conversion

	function setUpConversion() {

		if ( !options.convertUpAxis || colladaUp === options.upAxis ) {

			upConversion = null;

		} else {

			switch ( colladaUp ) {

				case 'X':

					upConversion = options.upAxis === 'Y' ? 'XtoY' : 'XtoZ';
					break;

				case 'Y':

					upConversion = options.upAxis === 'X' ? 'YtoX' : 'YtoZ';
					break;

				case 'Z':

					upConversion = options.upAxis === 'X' ? 'ZtoX' : 'ZtoY';
					break;

			}

		}

	};

	function fixCoords( data, sign ) {

		if ( !options.convertUpAxis || colladaUp === options.upAxis ) {

			return;

		}

		switch ( upConversion ) {

			case 'XtoY':

				var tmp = data[ 0 ];
				data[ 0 ] = sign * data[ 1 ];
				data[ 1 ] = tmp;
				break;

			case 'XtoZ':

				var tmp = data[ 2 ];
				data[ 2 ] = data[ 1 ];
				data[ 1 ] = data[ 0 ];
				data[ 0 ] = tmp;
				break;

			case 'YtoX':

				var tmp = data[ 0 ];
				data[ 0 ] = data[ 1 ];
				data[ 1 ] = sign * tmp;
				break;

			case 'YtoZ':

				var tmp = data[ 1 ];
				data[ 1 ] = sign * data[ 2 ];
				data[ 2 ] = tmp;
				break;

			case 'ZtoX':

				var tmp = data[ 0 ];
				data[ 0 ] = data[ 1 ];
				data[ 1 ] = data[ 2 ];
				data[ 2 ] = tmp;
				break;

			case 'ZtoY':

				var tmp = data[ 1 ];
				data[ 1 ] = data[ 2 ];
				data[ 2 ] = sign * tmp;
				break;

		}

	};

	function getConvertedVec3( data, offset ) {

		var arr = [ data[ offset ], data[ offset + 1 ], data[ offset + 2 ] ];
		fixCoords( arr, -1 );
		return new THREE.Vector3( arr[ 0 ], arr[ 1 ], arr[ 2 ] );

	};

	function getConvertedMat4( data ) {

		if ( options.convertUpAxis ) {

			// First fix rotation and scale

			// Columns first
			var arr = [ data[ 0 ], data[ 4 ], data[ 8 ] ];
			fixCoords( arr, -1 );
			data[ 0 ] = arr[ 0 ];
			data[ 4 ] = arr[ 1 ];
			data[ 8 ] = arr[ 2 ];
			arr = [ data[ 1 ], data[ 5 ], data[ 9 ] ];
			fixCoords( arr, -1 );
			data[ 1 ] = arr[ 0 ];
			data[ 5 ] = arr[ 1 ];
			data[ 9 ] = arr[ 2 ];
			arr = [ data[ 2 ], data[ 6 ], data[ 10 ] ];
			fixCoords( arr, -1 );
			data[ 2 ] = arr[ 0 ];
			data[ 6 ] = arr[ 1 ];
			data[ 10 ] = arr[ 2 ];
			// Rows second
			arr = [ data[ 0 ], data[ 1 ], data[ 2 ] ];
			fixCoords( arr, -1 );
			data[ 0 ] = arr[ 0 ];
			data[ 1 ] = arr[ 1 ];
			data[ 2 ] = arr[ 2 ];
			arr = [ data[ 4 ], data[ 5 ], data[ 6 ] ];
			fixCoords( arr, -1 );
			data[ 4 ] = arr[ 0 ];
			data[ 5 ] = arr[ 1 ];
			data[ 6 ] = arr[ 2 ];
			arr = [ data[ 8 ], data[ 9 ], data[ 10 ] ];
			fixCoords( arr, -1 );
			data[ 8 ] = arr[ 0 ];
			data[ 9 ] = arr[ 1 ];
			data[ 10 ] = arr[ 2 ];

			// Now fix translation
			arr = [ data[ 3 ], data[ 7 ], data[ 11 ] ];
			fixCoords( arr, -1 );
			data[ 3 ] = arr[ 0 ];
			data[ 7 ] = arr[ 1 ];
			data[ 11 ] = arr[ 2 ];

		}

		return new THREE.Matrix4(
			data[0], data[1], data[2], data[3],
			data[4], data[5], data[6], data[7],
			data[8], data[9], data[10], data[11],
			data[12], data[13], data[14], data[15]
			);

	};

	function getConvertedIndex( index ) {

		if ( index > -1 && index < 3 ) {

			var members = ['X', 'Y', 'Z'],
				indices = { X: 0, Y: 1, Z: 2 };

			index = getConvertedMember( members[ index ] );
			index = indices[ index ];

		}

		return index;

	};

	function getConvertedMember( member ) {

		if ( options.convertUpAxis ) {

			switch ( member ) {

				case 'X':

					switch ( upConversion ) {

						case 'XtoY':
						case 'XtoZ':
						case 'YtoX':

							member = 'Y';
							break;

						case 'ZtoX':

							member = 'Z';
							break;

					}

					break;

				case 'Y':

					switch ( upConversion ) {

						case 'XtoY':
						case 'YtoX':
						case 'ZtoX':

							member = 'X';
							break;

						case 'XtoZ':
						case 'YtoZ':
						case 'ZtoY':

							member = 'Z';
							break;

					}

					break;

				case 'Z':

					switch ( upConversion ) {

						case 'XtoZ':

							member = 'X';
							break;

						case 'YtoZ':
						case 'ZtoX':
						case 'ZtoY':

							member = 'Y';
							break;

					}

					break;

			}

		}

		return member;

	};

	return {

		load: load,
		parse: parse,
		setPreferredShading: setPreferredShading,
		applySkin: applySkin,
		geometries : geometries,
		options: options

	};

};
/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 */

/**
 * A mesh load can be used to load a given 3D mesh into a THREE object.
 * 
 * @constructor
 * @param options - object with following keys:
 *  * path - the base path to the associated Collada models that will be loaded
 */
ROS3D.MeshLoader = function(options) {
  var options = options || {};
  this.path = options.path || '';

  // check for a trailing '/'
  if (this.path.substr(this.path.length - 1) !== '/') {
    this.path += '/';
  }
};

/**
 * Load the given mesh resource into a THREE Object3D.
 * 
 * @param resource - the mesh resource to load.
 * @returns the THREE Object 3D containing the loaded mesh
 */
ROS3D.MeshLoader.prototype.load = function(resource) {
  var object = new THREE.Object3D();
  var uri = this.path + resource;
  var fileType = uri.substr(-4).toLowerCase();

  // check the type
  if (uri.substr(-4).toLowerCase() === '.dae') {
    var loader = new ROS3D.ColladaLoader();
    loader.load(uri, function colladaReady(collada) {
      object.add(collada.scene);
    });
  }
  return object;
};
/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A URDF loader can be used to load a URDF and its associated models into a 3D object. By default,
 * the URDF is loaded from the ROS parameter server.
 *
 * Emits the following events:
 * * 'ready' - emited after the URDF and its meshes have been loaded into the root object
 * 
 * @constructor
 * @param options - object with following keys:
 *  * tfClient - a handle to the TF client to be used to update the model's pose
 *  * path - the base path to the associated Collada models that will be loaded
 */
ROS3D.UrdfLoader = function(options) {
  var that = this;
  var options = options || {};
  this.tfClient = options.tfClient;
  this.path = options.path || 'resources/';
};

/**
 * Load the given URDF into a THREE Object3D.
 * 
 * @param string - the URDF as a string
 * @returns the THREE Object3D that contains the entire URDF
 */
ROS3D.UrdfLoader.prototype.load = function(string) {
  // hand off the XML string to the URDF model
  var urdfModel = new ROS3D.UrdfModel({
    string : string
  });

  var objectRoot = new THREE.Object3D();

  // load all models
  var links = urdfModel.links;
  var meshLoader = new ROS3D.MeshLoader({
    path : this.path
  });
  for ( var l in links) {
    var link = links[l];
    if (link.visual && link.visual.geometry) {
      if (link.visual.geometry.type === ROS3D.URDF_MESH) {
        var frameID = '/' + link.name;
        var uri = link.visual.geometry.filename;
        var fileType = uri.substr(-4).toLowerCase();

        // ignore mesh files which are not in Collada format
        if (fileType === '.dae') {
          var colladaModel = meshLoader.load(uri.substring(10));
          // create a scene node with the model
          var sceneNode = new ROS3D.SceneNode({
            frameID : frameID,
            tfClient : this.tfClient,
            model : colladaModel
          });
          objectRoot.add(sceneNode);
        }
      }
    }
  }
  return objectRoot;
};
/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * An Axes object can be used to display the axis of a particular coordinate frame.
 * 
 * @constructor
 * @param options - object with following keys:
 *   * shaftRadius - the radius of the shaft to render
 *   * headRadius - the radius of the head to render 
 *   * headLength - the length of the head to render
 */
ROS3D.Axes = function(options) {
  THREE.Object3D.call(this);
  var that = this;
  var options = options || {};
  var shaftRadius = options.shaftRadius || 0.008;
  var headRadius = options.headRadius || 0.023;
  var headLength = options.headLength || 0.1;

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
    var arrow = new THREE.Mesh(that.headGeom, material);
    arrow.position = axis.clone();
    arrow.position.multiplyScalar(0.95);
    arrow.useQuaternion = true;
    arrow.quaternion = rot;
    arrow.updateMatrix();
    that.add(arrow);

    // create the line
    var line = new THREE.Mesh(that.lineGeom, material);
    line.position = axis.clone();
    line.position.multiplyScalar(0.45);
    line.useQuaternion = true;
    line.quaternion = rot;
    line.updateMatrix();
    that.add(line);
  };

  // create the cylinders for the objects
  this.lineGeom = new THREE.CylinderGeometry(shaftRadius, shaftRadius, 1.0 - headLength);
  this.headGeom = new THREE.CylinderGeometry(0, headRadius, headLength);

  // add the three markers to the axes
  addAxis(new THREE.Vector3(1, 0, 0));
  addAxis(new THREE.Vector3(0, 1, 0));
  addAxis(new THREE.Vector3(0, 0, 1));
};
ROS3D.Axes.prototype.__proto__ = THREE.Object3D.prototype;
