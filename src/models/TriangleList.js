/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A TriangleList is a THREE object that can be used to display a list of triangles as a geometry.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * material (optional) - the material to use for the object
 *   * vertices - the array of vertices to use
 *   * colors - the associated array of colors to use
 */
ROS3D.TriangleList = function(options) {
  options = options || {};
  var material = options.material || new THREE.MeshBasicMaterial();
  var vertices = options.vertices;
  var colors = options.colors;

  THREE.Object3D.call(this);

  // set the material to be double sided
  material.side = THREE.DoubleSide;

  // construct the geometry
  var geometry = new THREE.Geometry();
  for (i = 0; i < vertices.length; i++) {
    geometry.vertices.push(new THREE.Vector3(vertices[i].x, vertices[i].y, vertices[i].z));
  }

  // set the colors
  var i, j;
  if (colors.length === vertices.length) {
    // use per-vertex color
    for (i = 0; i < vertices.length; i += 3) {
      var faceVert = new THREE.Face3(i, i + 1, i + 2);
      for (j = i * 3; j < i * 3 + 3; i++) {
        var color = new THREE.Color();
        color.setRGB(colors[i].r, colors[i].g, colors[i].b);
        faceVert.vertexColors.push(color);
      }
      geometry.faces.push(faceVert);
    }
    material.vertexColors = THREE.VertexColors;
  } else if (colors.length === vertices.length / 3) {
    // use per-triangle color
    for (i = 0; i < vertices.length; i += 3) {
      var faceTri = new THREE.Face3(i, i + 1, i + 2);
      faceTri.color.setRGB(colors[i / 3].r, colors[i / 3].g, colors[i / 3].b);
      geometry.faces.push(faceTri);
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
  geometry.computeFaceNormals();

  this.add(new THREE.Mesh(geometry, material));
};
ROS3D.TriangleList.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Set the color of this object to the given hex value.
 *
 * @param hex - the hex value of the color to set
 */
ROS3D.TriangleList.prototype.setColor = function(hex) {
  this.mesh.material.color.setHex(hex);
};
