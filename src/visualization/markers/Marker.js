ROS3D.Marker = function(options) {
  var that = this;
  var options = options || {};
  this.path = options.path || '/';
  var message = options.message;

  // check for a trailing '/'
  if (this.path.substr(this.path.length - 1) !== '/') {
    this.path += '/';
  }

  /**
   * Create a THREE material based on the given RGBA values.
   * 
   * @param r - the red value
   * @param g - the green value
   * @param b - the blue value
   * @param a - the alpha value
   * @returns the THREE material
   */
  function makeColorMaterial(r, g, b, a) {
    var color = new THREE.Color();
    color.setRGB(r, g, b);
    if (a <= 0.99) {
      return new THREE.MeshBasicMaterial({
        color : color.getHex(),
        opacity : a + 0.1,
        transparent : true,
        depthWrite : true,
        blendSrc : THREE.SrcAlphaFactor,
        blendDst : THREE.OneMinusSrcAlphaFactor,
        blendEquation : THREE.ReverseSubtractEquation,
        blending : THREE.NormalBlending
      });
    } else {
      return new THREE.MeshLambertMaterial({
        color : color.getHex(),
        opacity : a,
        blending : THREE.NormalBlending
      });
    }
  }

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // set the pose and get the color
  this.setPose(message.pose);
  var colorMaterial = makeColorMaterial(message.color.r, message.color.g, message.color.b,
      message.color.a);

  // create the object based on the type
  switch (message.type) {
    case ROS3D.MARKER_ARROW:
      // get the sizes for the arrow
      var len = message.scale.x;
      var headLen = len * 0.23;
      var headR = message.scale.y;
      var shaftR = headR * 0.5;

      // determine the points
      if (message.points.length === 2) {
        var p1 = new THREE.Vector3(message.points[0].x, message.points[0].y, message.points[0].z);
        var p2 = new THREE.Vector3(message.points[1].x, message.points[1].y, message.points[1].z);
        var direction = p1.clone().negate().add(p2);
        // direction = p2 - p1;
        len = direction.length();
        headR = message.scale.y;
        shaftR = message.scale.x;

        if (message.scale.z !== 0.0) {
          headLen = message.scale.z;
        }
      }
      // add the marker
      this.add(new ROS3D.ArrowMarker({
        direction : direction,
        origin : p1,
        length : len,
        headLength : headLen,
        shaftDiameter : shaftR,
        headDiameter : headR,
        material : colorMaterial
      }));
      break;
    case ROS3D.MARKER_CUBE:
      var geom = new THREE.CubeGeometry(message.scale.x, message.scale.y, message.scale.z);
      this.add(new THREE.Mesh(geom, colorMaterial));
      break;
    case ROS3D.MARKER_SPHERE:
      var geom = new THREE.SphereGeometry(0.5);
      var mesh = new THREE.Mesh(geom, colorMaterial);
      mesh.scale.x = message.scale.x;
      mesh.scale.y = message.scale.y;
      mesh.scale.z = message.scale.z;
      that.add(mesh);
      break;
    case ROS3D.MARKER_CYLINDER:
      var geom = new THREE.CylinderGeometry(0.5, 0.5, 1, 16, 1, false);
      var mesh = new THREE.Mesh(geom, colorMaterial);
      mesh.useQuaternion = true;
      mesh.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * 0.5);
      mesh.scale = new THREE.Vector3(message.scale.x, message.scale.y, message.scale.z);
      this.add(mesh);
      break;
    case ROS3D.MARKER_CUBE_LIST:
    case ROS3D.MARKER_SPHERE_LIST:
    case ROS3D.MARKER_POINTS:
      var geometry = new THREE.Geometry();
      var material = new THREE.ParticleBasicMaterial({
        size : message.scale.x
      });

      for ( var i = 0; i < message.points.length; i++) {
        var vertex = new THREE.Vector3();
        vertex.x = message.points[i].x;
        vertex.y = message.points[i].y;
        vertex.z = message.points[i].z;
        geometry.vertices.push(vertex);
      }

      if (message.colors.length == message.points.length) {
        material.vertexColors = true;
        for ( var i = 0; i < message.points.length; i++) {
          var color = new THREE.Color();
          color.setRGB(message.colors[i].r, message.colors[i].g, message.colors[i].b);
          geometry.colors.push(color);
        }
      } else {
        material.color.setRGB(message.color.r, message.color.g, message.color.b);
      }

      var particles = new THREE.ParticleSystem(geometry, material);
      this.add(particles);
      break;
    case ROS3D.MARKER_TEXT_VIEW_FACING:
      var textGeo = new THREE.TextGeometry(message.text, {

        size : message.scale.x * 0.5,
        height : 0.1 * message.scale.x,
        curveSegments : 4,
        font : "helvetiker",

        bevelEnabled : false,
        bevelThickness : 2,
        bevelSize : 2,

        material : 0,
        extrudeMaterial : 0

      });

      textGeo.computeVertexNormals();
      textGeo.computeBoundingBox();

      var mesh = new THREE.Mesh(textGeo, colorMaterial);

      var centerOffset = -0.5 * (textGeo.boundingBox.max.x - textGeo.boundingBox.min.x);
      mesh.position.y = -centerOffset;

      mesh.rotation.x = Math.PI * 0.5;
      mesh.rotation.y = Math.PI * 1.5;
      this.add(mesh);
      break;
    case ROS3D.MARKER_MESH_RESOURCE:
      if (message.mesh_use_embedded_materials) {
        var meshMarker = new ROS3D.MeshMarker(message, this.path, false);
      } else {
        var meshMarker = new ROS3D.MeshMarker(message, this.path, colorMaterial);
      }
      this.add(meshMarker);
      break;
    case ROS3D.MARKER_TRIANGLE_LIST:
      var tri = new ROS3D.TriangleListMarker(colorMaterial, message.points, message.colors);
      tri.scale = new THREE.Vector3(message.scale.x, message.scale.y, message.scale.z);
      this.add(tri);
      break;
    case ROS3D.MARKER_LINE_STRIP:
    case ROS3D.MARKER_LINE_LIST:
      var geom = new THREE.CubeGeometry(0.1, 0.1, 0.1);
      this.add(new THREE.Mesh(geom, colorMaterial));
      break;
    default:
      console.error('Unknown marker type: ' + message.type);
      break;
  }

};
ROS3D.Marker.prototype.__proto__ = THREE.Object3D.prototype;

ROS3D.Marker.prototype.setPose = function(pose) {
  // set position information
  this.position.x = pose.position.x;
  this.position.y = pose.position.y;
  this.position.z = pose.position.z;

  // set the rotation
  this.quaternion = new THREE.Quaternion(pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);
  this.quaternion.normalize();

  // update the world
  this.updateMatrixWorld();
};
