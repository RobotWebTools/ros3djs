/**
 * @fileOverview
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
ROS3D.TriangleList = function (options) {
  options = options || {};
  var material = options.material || new THREE.MeshBasicMaterial();
  var vertices = options.vertices;
  var colors = options.colors;

  THREE.Object3D.call(this);

  // set the material to be double sided
  material.side = THREE.DoubleSide;


  // Construct the geometry
  let geometry = new THREE.BufferGeometry();
  let verticesArray = new Float32Array(vertices.length * 3);
  for (let i = 0; i < vertices.length; i++) {
    verticesArray[i * 3] = vertices[i].x;
    verticesArray[i * 3 + 1] = vertices[i].y;
    verticesArray[i * 3 + 2] = vertices[i].z;
  }
  geometry.setAttribute('position', new THREE.BufferAttribute(verticesArray, 3));

  // Set the colors
  let i, j;
  if (colors.length === vertices.length) {
    // Use per-vertex color
    let vertexColors = new Float32Array(colors.length * 3);
    for (i = 0; i < colors.length; i++) {
      vertexColors[i * 3] = colors[i].r;
      vertexColors[i * 3 + 1] = colors[i].g;
      vertexColors[i * 3 + 2] = colors[i].b;
    }
    geometry.setAttribute('color', new THREE.BufferAttribute(vertexColors, 3));
    material.vertexColors = true;
  } else if (colors.length === vertices.length / 3) {
    // Use per-triangle color
    let faceColors = new Float32Array(vertices.length);
    for (i = 0; i < colors.length; i++) {
      let color = new THREE.Color(colors[i].r, colors[i].g, colors[i].b);
      for (j = 0; j < 9; j += 3) {
        faceColors[i * 9 + j] = color.r;
        faceColors[i * 9 + j + 1] = color.g;
        faceColors[i * 9 + j + 2] = color.b;
      }
    }
    geometry.setAttribute('color', new THREE.BufferAttribute(faceColors, 3));
    material.vertexColors = true;
  } else {
    // Use marker color
    let defaultColor = new THREE.Color(1, 1, 1); // Default color
    let faceColors = new Float32Array(vertices.length);
    for (i = 0; i < vertices.length; i += 3) {
      faceColors[i * 3] = defaultColor.r;
      faceColors[i * 3 + 1] = defaultColor.g;
      faceColors[i * 3 + 2] = defaultColor.b;
    }
    geometry.setAttribute('color', new THREE.BufferAttribute(faceColors, 3));
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
ROS3D.TriangleList.prototype.setColor = function (hex) {
  this.mesh.material.color.setHex(hex);
};
