import THREE from '../shims/three/core.js';

/**
 * @author David Gossow - dgossow@willowgarage.com
 */

export class TriangleList extends THREE.Object3D {

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
  constructor(options) {
    options = options || {};
    var material = options.material || new THREE.MeshBasicMaterial();
    var vertices = [];
    var vertexColors = [];
    var colors = options.colors;

    super();

    // set the material to be double sided
    material.side = THREE.DoubleSide;

    // construct the geometry
    
    for (i = 0; i < options.vertices.length; i++) {
      vertices.push(options.vertices[i].x, options.vertices[i].y, options.vertices[i].z);
    }

    var geometry = new THREE.BufferGeometry();
    geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( vertices, 3 ) );



    // set the colors
    var i, j;
    if (colors.length === options.vertices.length ) {

      // use per-vertex color
      for (i = 0; i < options.vertices.length; i += 3) {
        for (j = i * 3; j < i * 3 + 3; i++) {
          vertexColors.push(colors[i].r, colors[i].g, colors[i].b);
        }
      }
      material.vertexColors = true;
      geometry.setAttribute( 'color', new THREE.Float32BufferAttribute( vertexColors, 3 ) );

    } else if (colors.length === options.vertices.length / 3) {

      // use per-triangle color
      for (i = 0; i < options.vertices.length; i += 3) {
        const idx = i / 3
        vertexColors.push(colors[idx].r, colors[idx].g, colors[idx].b,
          colors[idx].r, colors[idx].g, colors[idx].b,
          colors[idx].r, colors[idx].g, colors[idx].b
        );
      }

      material.vertexColors = true;
      geometry.setAttribute( 'color', new THREE.Float32BufferAttribute( vertexColors, 3 ) );

    } else {

      // use marker color

    }

    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();
    geometry.computeVertexNormals ();

    this.add(new THREE.Mesh(geometry, material));
  };

  /**
   * Set the color of this object to the given hex value.
   *
   * @param hex - the hex value of the color to set
   */
  setColor(hex) {
    this.mesh.material.color.setHex(hex);
  };
}
