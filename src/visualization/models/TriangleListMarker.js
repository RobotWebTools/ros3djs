ROS3D.TriangleListMarker = function(material, vertices, colors) {
  THREE.Object3D.call(this);

  if (material === undefined)
    material = new THREE.MeshBasicMaterial();

  material.side = THREE.DoubleSide;

  var geometry = new THREE.Geometry();

  for (i = 0; i < vertices.length; i++) {
    geometry.vertices.push(new THREE.Vector3(vertices[i].x, vertices[i].y, vertices[i].z));
  }

  if (colors.length === vertices.length) {
    // use per-vertex color
    for (i = 0; i < vertices.length; i += 3) {
      var face = new THREE.Face3(i, i + 1, i + 2);
      for (j = i * 3; j < i * 3 + 3; i++) {
        var color = new THREE.Color();
        color.setRGB(colors[i].r, colors[i].g, colors[i].b);
        face.vertexColors.push(color);
      }
      geometry.faces.push(face);
    }
    material.vertexColors = THREE.VertexColors;
  } else if (colors.length === vertices.length / 3) {
    // use per-triangle color
    for (i = 0; i < vertices.length; i += 3) {
      var face = new THREE.Face3(i, i + 1, i + 2);
      face.color.setRGB(colors[i / 3].r, colors[i / 3].g, colors[i / 3].b);
      geometry.faces.push(face);
    }
    material.vertexColors = THREE.FaceColors;
  } else {
    // use marker color
    for (i = 0; i < vertices.length; i += 3) {
      var face = new THREE.Face3(i, i + 1, i + 2);
      geometry.faces.push(face);
    }
  }

  geometry.computeBoundingBox();
  geometry.computeBoundingSphere();
  geometry.computeCentroids();
  geometry.computeFaceNormals();

  this.mesh = new THREE.Mesh(geometry, material);
  this.add(this.mesh);
};

ROS3D.TriangleListMarker.prototype = Object.create(THREE.Object3D.prototype);

ROS3D.TriangleListMarker.prototype.setColor = function(hex) {
  this.mesh.material.color.setHex(hex);
};
