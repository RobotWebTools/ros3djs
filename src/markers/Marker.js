/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A Marker can convert a ROS marker message into a THREE object.
 *
 * @constructor
 * @param options - object with following keys:
 *   * path - the base path or URL for any mesh files that will be loaded for this marker
 *   * message - the marker message
 */
ROS3D.Marker = function(options) {
  var options = options || {};
  var path = options.path || '/';
  var message = options.message;

  // check for a trailing '/'
  if (path.substr(path.length - 1) !== '/') {
    path += '/';
  }

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // set the pose and get the color
  this.setPose(message.pose);
  var colorMaterial = ROS3D.makeColorMaterial(message.color.r, message.color.g, message.color.b,
      message.color.a);

  // create the object based on the type
  switch (message.type) {
    case ROS3D.MARKER_ARROW:
      // get the sizes for the arrow
      var len = message.scale.x;
      var headLength = len * 0.23;
      var headDiameter = message.scale.y;
      var shaftDiameter = headDiameter * 0.5;

      // determine the points
      if (message.points.length === 2) {
        var p1 = new THREE.Vector3(message.points[0].x, message.points[0].y, message.points[0].z);
        var p2 = new THREE.Vector3(message.points[1].x, message.points[1].y, message.points[1].z);
        var direction = p1.clone().negate().add(p2);
        // direction = p2 - p1;
        len = direction.length();
        headDiameter = message.scale.y;
        shaftDiameter = message.scale.x;

        if (message.scale.z !== 0.0) {
          headLength = message.scale.z;
        }
      }

      // add the marker
      this.add(new ROS3D.Arrow({
        direction : direction,
        origin : p1,
        length : len,
        headLength : headLength,
        shaftDiameter : shaftDiameter,
        headDiameter : headDiameter,
        material : colorMaterial
      }));
      break;
    case ROS3D.MARKER_CUBE:
      // set the cube dimensions
      var geom = new THREE.CubeGeometry(message.scale.x, message.scale.y, message.scale.z);
      this.add(new THREE.Mesh(geom, colorMaterial));
      break;
    case ROS3D.MARKER_SPHERE:
      // set the sphere dimensions
      var geom = new THREE.SphereGeometry(0.5);
      var mesh = new THREE.Mesh(geom, colorMaterial);
      mesh.scale.x = message.scale.x;
      mesh.scale.y = message.scale.y;
      mesh.scale.z = message.scale.z;
      this.add(mesh);
      break;
    case ROS3D.MARKER_CYLINDER:
      // set the cylinder dimensions
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
      // for now, use a particle system for the lists
      var geometry = new THREE.Geometry();
      var material = new THREE.ParticleBasicMaterial({
        size : message.scale.x
      });

      // add the points
      for ( var i = 0; i < message.points.length; i++) {
        var vertex = new THREE.Vector3();
        vertex.x = message.points[i].x;
        vertex.y = message.points[i].y;
        vertex.z = message.points[i].z;
        geometry.vertices.push(vertex);
      }

      // determine the colors for each
      if (message.colors.length === message.points.length) {
        material.vertexColors = true;
        for ( var i = 0; i < message.points.length; i++) {
          var color = new THREE.Color();
          color.setRGB(message.colors[i].r, message.colors[i].g, message.colors[i].b);
          geometry.colors.push(color);
        }
      } else {
        material.color.setRGB(message.color.r, message.color.g, message.color.b);
      }

      // add the particle system
      this.add(new THREE.ParticleSystem(geometry, material));
      break;
    case ROS3D.MARKER_TEXT_VIEW_FACING:
      // setup the text
      var textGeo = new THREE.TextGeometry(message.text, {
        size : message.scale.x * 0.5,
        height : 0.1 * message.scale.x,
        curveSegments : 4,
        font : 'helvetiker',
        bevelEnabled : false,
        bevelThickness : 2,
        bevelSize : 2,
        material : 0,
        extrudeMaterial : 0
      });
      textGeo.computeVertexNormals();
      textGeo.computeBoundingBox();

      // position the text and add it
      var mesh = new THREE.Mesh(textGeo, colorMaterial);
      var centerOffset = -0.5 * (textGeo.boundingBox.max.x - textGeo.boundingBox.min.x);
      mesh.position.y = -centerOffset;
      mesh.rotation.x = Math.PI * 0.5;
      mesh.rotation.y = Math.PI * 1.5;
      this.add(mesh);
      break;
    case ROS3D.MARKER_MESH_RESOURCE:
      // check if we are using an emedded material
      if (message.mesh_use_embedded_materials) {
        this.add(new ROS3D.MeshMarker(message, path, false));
      } else {
        this.add(new ROS3D.MeshMarker(message, path, colorMaterial));
      }
      break;
    case ROS3D.MARKER_TRIANGLE_LIST:
      // create the list of triangles
      var tri = new ROS3D.TriangleList({
        material : colorMaterial,
        vertices : message.points,
        colors : message.colors
      });
      tri.scale = new THREE.Vector3(message.scale.x, message.scale.y, message.scale.z);
      this.add(tri);
      break;
    default:
      console.error('Currently unsupported marker type: ' + message.type);
      break;
  }
};
ROS3D.Marker.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Set the pose of this marker to the given values.
 * 
 * @param pose - the pose to set for this marker
 */
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
