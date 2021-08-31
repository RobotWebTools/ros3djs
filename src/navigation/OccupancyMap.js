import THREE from '../../shims/three/core.js';

/**
 * @author Peter Sari - sari@photoneo.com
 */

/**
 Quick and dirty helper class
 to read ArrayBuffer in a streamed data-like fashion with mixed types in it
*/
function InStream(data, isLittleEndian) {
  this.buffer = data.buffer;
  this.length = data.length;
  this.isLittleEndian = (typeof isLittleEndian !== 'undefined') ? !!isLittleEndian : true;
  this._dataView = new DataView(this.buffer);
  this._cursor = 0;

  // Creates a set of wrapper functions for DataView
  // also flattens all dependencies
  [
    { kind: 'Int8', width: 1 },
    { kind: 'Uint8', width: 1 },
    { kind: 'Int16', width: 2 },
    { kind: 'Uint16', width: 2 },
    { kind: 'Int32', width: 4 },
    { kind: 'Uint32', width: 4 },
    { kind: 'BigInt64', width: 8 },
    { kind: 'BigUint64', width: 8 },
    { kind: 'Float32', width: 4 },
    { kind: 'Float64', width: 8 },
  ]
    .forEach(wrap => {
      const interfaceFunction = 'read' + wrap.kind;
      const wrappedFunction = 'get' + wrap.kind;       // Function name which going to be wrapped from DataView

      this[interfaceFunction] = () => {
        if (this._cursor + wrap.width > this.length) { throw new Error('Cannot read data stream. Overflow. Len=' + this.length + ' crsr=' + this._cursor); }
        const returningValue = this._dataView[wrappedFunction](this._cursor, this.isLittleEndian);
        this._cursor += wrap.width;
        return returningValue;
      };
    });

  Object.defineProperty(this, 'isEnd', { get: () => this.cursor >= this.data.length });
  return this;
};

// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

/**
 * Base node type that represents one voxel as a node of the tree
 */

export var OcTreeBaseNode = function () {
  this._children = [null, null, null, null, null, null, null, null];
  this.value = null;
};

createChildNodeAt (newNode, index) {
  this._children[index % 8] = newNode;
};

hasChildAt (index) {
  return this._children[index % 8] !== null;
};

getChildAt (index) {
  return this._children[index % 8];
};

isLeafNode () {
  for (let i = 0; i < 8; ++i) {
    if (this._children[i] !== null) { return false; }
  }
  return true;
};

hasChildren () {
  for (let i = 0; i < 8; ++i) {
    if (this._children[i] !== null) { return true; }
  }
  return false;
};


// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

/**
 * Toggles voxel visibility
 *
 *    * `occupied` - only voxels that are above or equal to the occupation threshold are shown
 *    * `free` - only voxels that are below the occupation threshold are shown
 *    * `all` - all allocated voxels are shown
 */
export var OcTreeVoxelRenderMode = {
  OCCUPIED: 'occupied',
  FREE: 'free',
  ALL: 'all',
};

/**
 * Coloring modes for each voxel
 *
 *     * 'solid' - voxels will have a single solid color set by the tree globally
 *     * 'occupancy' - voxels are false colored by their occupancy value. Fall back for `solid` if not available.
 *     * 'color' - voxels will colorized by their
 */
export var OcTreeColorMode = {
  SOLID: 'solid',
  OCCUPANCY: 'occupancy',
  COLOR: 'color'
};

/**
 * Represensta a BaseTree that can be build from ros message and create a THREE node from it.
 * Due a tree can be represented different ways in a message, this class is also a base class to
 * represent specialized versions fo the ree.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *    * resolution - the size of leaf nodes in meter
 *    * color - color of the visualized map (if solid coloring option was set)
 *    * voxelRenderMode - toggle between rendering modes @see ROS3D.OcTreeVoxelRenderMode
 */
export var OcTreeBase = function (options) {

  this.resolution = (typeof options.resolution !== 'undefined') ? options.resolution : 1.;
  this.color = new THREE.Color((typeof options.color !== 'undefined') ? options.color : 'green');
  this.opacity = (typeof options.opacity !== 'undefined') ? options.opacity : 1.;

  this.voxelRenderMode = (typeof options.voxelRenderMode !== 'undefined') ? options.voxelRenderMode : OcTreeVoxelRenderMode.OCCUPIED;

  this._rootNode = null;
  this._treeDepth = 16;
  this._treeMaxKeyVal = 32768;

  // Table of voxel size for each level of the tree
  this.nodeSizeTable = new Array(this._treeDepth);
  let _val = this.resolution;
  for (let i = this._treeDepth - 1; i >= 0; --i) {
    this.nodeSizeTable[i] = _val;
    _val = 2. * _val;
  }

  this._defaultOccupiedValue = true;
  this._defaultFreeValue = false;

  this.object = null;
};


/*
 * Finds a key in a given depth. Search is performed on the lowest level by default.
 * @return the node at given position, null if not found
 */
searchAtDepth (key, depth) {
  depth = (typeof depth !== 'undefined') ? (depth > 0 ? depth : this._treeDepth) : this._treeDepth;

  const keyAtDepth = this._adjustKeyAtDepth(key, depth);
  const diff = this._treeDepth - depth;
  let currentNode = this._rootNode;

  // follow nodes down to requested level (for diff = 0 it's the last level)
  // Return the closest node, or null if not any

  for (let i = (this._treeDepth - 1); i >= diff; --i) {
    const pos = this._computeChildIdx(keyAtDepth, i);
    if (currentNode.hasChildAt(pos)) {
      currentNode = currentNode.getChildAt(pos);
    } else {
      // we expected a child but did not get it
      // is the current node a leaf already?
      if (!currentNode.hasChildren()) { return currentNode; }
      // it is not, search failed
      return null;
    }
  }

  return currentNode;

};

/**
 *
 */
_computeCoordFromKey (key) {
  return key.map(keyVal => this.resolution * (keyVal - this._treeMaxKeyVal));
};

_computeChildIdx (key, depth) {
  let pos = 0;
  if (key[0] & (1 << depth)) { pos += 1; }
  if (key[1] & (1 << depth)) { pos += 2; }
  if (key[2] & (1 << depth)) { pos += 4; }

  return pos;
};

_computeKeyFromChildIdx (index, offset, depth) {
  const diff = this._treeDepth - depth - 1;

  return [
    offset[0] + (!!(index & 1) << diff),
    offset[1] + (!!(index & 2) << diff),
    offset[2] + (!!(index & 4) << diff),
  ];

};

_adjustKeyAtDepth (key, depth) {
  // generate appropriate key_at_depth for queried depth
  let diff = this._treeDepth - depth;
  if (diff === 0) { return key; }

  return key.map(keyVal => (((keyVal - this._treeMaxKeyVal) >> diff) << diff) + (1 << (diff - 1)) + this._treeMaxKeyVal);
};

/**
 *
 */
export var OcTreeBase.prototype._BINARY_UNALLOCATED = 0b00;
export var OcTreeBase.prototype._BINARY_LEAF_FREE = 0b01;
export var OcTreeBase.prototype._BINARY_LEAF_OCCUPIED = 0b10;
export var OcTreeBase.prototype._BINARY_HAS_CHILDREN = 0b11;


_newNode () { return new OcTreeBaseNode(); };

/*
 * Reads and builds a tree which was represented in a binary form from a message
 * Binary form only contains the tree structure to be allocated, all the data of voxels are stripped,
 * occupation is represented as a binary value.
 * Each node is represented as a 2-bit value which makes up the 8 child nodes of the parent (16 bits in total)
 * starting with the root node.
 */

readBinary (data) {
  if (this._rootNode !== null) {
    delete this._rootNode;
  }
  this._rootNode = this._newNode();

  let dataStream = new InStream(data, true);

  let stack = new Array();
  stack.push(this._rootNode);

  while (stack.length > 0) {
    let node = stack.pop();

    // 2 bits per children, 16 bit total
    const childAllocationMap = dataStream.readUint16();

    // Insert all children and leaves
    let index = 8;
    while (index !== 0) {
      --index;
      const allocation = (childAllocationMap & (0b11 << (2 * index))) >> (2 * index);

      if (allocation !== this._BINARY_UNALLOCATED) {
        let child = this._newNode();

        const fn = this._BINARY_CHILD_BUILD_TABLE[allocation].bind(this);
        fn(child);

        node.createChildNodeAt(child, index);
        if (allocation === this._BINARY_HAS_CHILDREN) { stack.push(child); }
      }
    }
  }

};

export var OcTreeBase.prototype._BINARY_CHILD_BUILD_TABLE = {};

export var OcTreeBase.prototype._BINARY_CHILD_BUILD_TABLE[ROS3D.OcTreeBase.prototype._BINARY_LEAF_FREE] = function (child) {
  child.value = this._defaultFreeValue;
};

export var OcTreeBase.prototype._BINARY_CHILD_BUILD_TABLE[ROS3D.OcTreeBase.prototype._BINARY_LEAF_OCCUPIED] = function (child) {
  child.value = this._defaultOccupiedValue;
};

export var OcTreeBase.prototype._BINARY_CHILD_BUILD_TABLE[ROS3D.OcTreeBase.prototype._BINARY_HAS_CHILDREN] = function (child) {
  child.value = null;
};


/**
 * Reads a full tree (with node data) from a message.
 * A pacjet starts with the node data, followed by the allocation map of their children.
 * Each type of tree has different data structure @see ROS3DJS.OcTreeBase._readNodeData
 */

read (data) {
  if (this._rootNode !== null) {
    delete this._rootNode;
  }

  this._rootNode = this._newNode();

  let dataStream = new InStream(data, true);

  let stack = new Array();
  stack.push(this._rootNode);

  while (stack.length > 0) {
    let node = stack.pop();

    // Data comes first
    this._readNodeData(dataStream, node);

    const childAllocationMap = dataStream.readUint8();

    // Insert all children and leaves
    let index = 8;
    while (index !== 0) {
      --index;
      const hasChild = childAllocationMap & (1 << index);
      if (hasChild) {
        let child = this._newNode();
        child.value = null;
        node.createChildNodeAt(child, index);
        stack.push(child);
      }
    }
  }

};

/**
 * Abstract function; Reads and sets data of a node
 */
_readNodeData (dataStream, node) {
  // This needs to be implemented by specialized tree
  console.error('Not implemented');
};

/**
* Builds up THREE.js geometry from tree data.
*/
buildGeometry () {
  console.assert(this._rootNode !== null, 'No tree data');
  const { vertices, normals, colors, indices } = this._buildFaces();

  const geometry = new THREE.BufferGeometry();

  const material = new THREE.MeshBasicMaterial({
    color: 'white',
    flatShading: true,
    vertexColors: THREE.VertexColors,
    transparent: this.opacity < 1.0,
    opacity: this.opacity
  });

  geometry.addAttribute('position', new THREE.BufferAttribute(new Float32Array(vertices), 3));
  geometry.addAttribute('normal', new THREE.BufferAttribute(new Float32Array(normals), 3));
  geometry.addAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));

  geometry.setIndex(indices);

  const mesh = new THREE.Mesh(geometry, material);
  this.object = new THREE.Object3D();
  this.object.add(mesh);
};

_traverseLeaves (callback) {
  let stack = new Array();
  stack.push({ node: this._rootNode, depth: 0, key: [0, 0, 0] });

  while (stack.length > 0) {
    let current = stack.pop();
    if (current.node.isLeafNode()) {
      callback(current.node, current.key, current.depth - 1);
    } else {
      for (let index = 0; index < 8; ++index) {
        if (current.node.hasChildAt(index)) {
          const key = this._computeKeyFromChildIdx(index, current.key, current.depth);
          stack.push({
            node: current.node.getChildAt(index),
            depth: current.depth + 1,
            key
          });
        }
      }
    }
  }
};

/**
 * Abstract function; to implement different coloring schemes
 */
_obtainColor (node) {
  return this.color;
};

_checkOccupied (node) {
  return node.value !== false;
};

_buildFaces () {
  let geometry = {
    vertices: [],
    indices: [],
    normals: [],
    colors: [],

    _insertFace: function (face, pos, size, color) {
      const indexCount = this.vertices.length / 3;

      for (let vertex of face.vertices) {
        this.vertices.push(
          pos[0] + vertex[0] * size,
          pos[1] + vertex[1] * size,
          pos[2] + vertex[2] * size
        );
      };

      const colorArr = [color.r, color.g, color.b];

      this.colors.push(...colorArr, ...colorArr, ...colorArr, ...colorArr);
      this.normals.push(...face.normal, ...face.normal, ...face.normal, ...face.normal);

      this.indices.push(
        indexCount, indexCount + 1, indexCount + 2,
        indexCount + 2, indexCount + 1, indexCount + 3
      );
    },

    _checkNeighborsTouchingFace: function (face, neighborNode, voxelRenderMode) {
      // Finds if there's not a node at a given position, aka a 'hole'
      let stack = new Array();
      stack.push(neighborNode);
      while (stack.length !== 0) {
        const node = stack.pop();
        if (node.hasChildren()) {
          for (let childIndex of face.childIndex) {

            if (node.hasChildAt(childIndex)) {
              const child = node.getChildAt(childIndex);

              // filter occupancy
              const isOccupied = this._checkOccupied(node);
              const isNeedsToRender = (isOccupied && voxelRenderMode === ROS3D.OcTreeVoxelRenderMode.OCCUPIED) || (!isOccupied && voxelRenderMode === OcTreeVoxelRenderMode.FREE);

              if (isNeedsToRender) { stack.push(child); }
            }
            else {
              return true;
            }
          }
        }
      }
      return false;
    }

  };

  this._traverseLeaves((node, key, depth) => {
    const pos = this._computeCoordFromKey(key);
    const size = this.nodeSizeTable[depth];
    const diff = this._treeDepth - depth;

    const isOccupied = this._checkOccupied(node);

    // By default it will show ALL
    // Hide free voxels if set
    if (!isOccupied && this.voxelRenderMode === OcTreeVoxelRenderMode.OCCUPIED) { return; }

    // Hide occuped voxels if set.
    if (isOccupied && this.voxelRenderMode === OcTreeVoxelRenderMode.FREE) { return; }

    for (let face of this.FACES)
    // let face = this.FACES[1] ;
    {
      // Add geometry where there is no neighbor voxel
      const neighborKey = [
        key[0] + face.normal[0] * diff * diff,
        key[1] + face.normal[1] * diff * diff,
        key[2] + face.normal[2] * diff * diff,
      ];
      const neighborNode = this.searchAtDepth(neighborKey);
      if (neighborNode === null) {
        // 1. Simply add geometry where there is no neighbors
        geometry._insertFace(face, pos, size, this._obtainColor(node));
      } else if (depth < this._treeDepth) {
        // 2. Special case, when a node (voxel) is not on the lowest level
        // of the tree, but also need to add a geometry, because might
        // not be "fully covered" by neighboring voxels on the lowest level

        if (geometry._checkNeighborsTouchingFace(face, neighborNode, this.voxelRenderMode)) {
          geometry._insertFace(face, pos, size, this._obtainColor(node));
        }
      }

    }

  });

  // return geometry;
  return {
    vertices: geometry.vertices,
    normals: geometry.normals,
    colors: geometry.colors,
    indices: geometry.indices
  };

};

/**
 * Table which we are building the geometry data from.
 */
export var OcTreeBase.prototype.FACES = [
  { // 0. left (x=0)
    normal: [-1, 0, 0,],
    vertices: [
      [0, 1, 0],
      [0, 0, 0],
      [0, 1, 1],
      [0, 0, 1],
    ],
    childIndex: [
      0b001,
      0b011,
      0b101,
      0b111
    ]
  },
  { // 1. right (x=1)
    normal: [1, 0, 0,],
    vertices: [
      [1, 1, 1],
      [1, 0, 1],
      [1, 1, 0],
      [1, 0, 0],
    ],

    childIndex: [
      0b000,
      0b010,
      0b100,
      0b110
    ]
  },
  { // 2. bottom (y=0)
    normal: [0, -1, 0,],
    vertices: [
      [1, 0, 1],
      [0, 0, 1],
      [1, 0, 0],
      [0, 0, 0],
    ],
    childIndex: [
      0b010,
      0b011,
      0b110,
      0b111
    ]
  },
  { // 3. top (y=1)
    normal: [0, 1, 0,],
    vertices: [
      [0, 1, 1],
      [1, 1, 1],
      [0, 1, 0],
      [1, 1, 0],
    ],
    childIndex: [
      0b000,
      0b001,
      0b100,
      0b101
    ]
  },
  { // 4. back (z=0)
    normal: [0, 0, -1,],
    vertices: [
      [1, 0, 0],
      [0, 0, 0],
      [1, 1, 0],
      [0, 1, 0],
    ],
    childIndex: [
      0b100,
      0b101,
      0b110,
      0b111
    ]
  },
  { // 5.front (z=1)
    normal: [0, 0, 1,],
    vertices: [
      [0, 0, 1],
      [1, 0, 1],
      [0, 1, 1],
      [1, 1, 1],
    ],
    childIndex: [
      0b000,
      0b001,
      0b010,
      0b011
    ]
  },
];

// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
/**
 * Specilaization of BaseOcTree
 *
 * @constructor
 * @param options - object with following keys:
 *    * inherited from BaseOctree
 *    * occupancyThreshold (optional) - threshold value that separates occupied and free voxels from each other. (Default: 0)
 *    * colorMode (optional) - Coloring mode @see ROS3D.OcTreeColorMode.
 *    * palette (optional) - Palette used for false-coloring (default: predefined palette)
 *    * paletteSclae (optional) - Scale of palette to represent a wider range of values (default: 1.)
 */

export var OcTree = function (options) {
  OcTreeBase.call(this, options);

  this._defaultOccupiedValue = 1.;
  this._defaultFreeValue = -1.;

  this.occupancyThreshold = (typeof options.occupancyThreshold !== 'undefined') ? options.occupancyThreshold : 0.0000001;

  this.useFlatColoring = (typeof options.colorMode !== 'undefined') && options.colorMode === OcTreeColorMode.SOLID;

  this.palette = (typeof options.palette !== 'undefined') ? options.palette.map(color => new THREE.Color(color)) :
    [
      { r: 0, g: 0, b: 128, }, // dark blue (low)
      { r: 0, g: 255, b: 0, }, // green
      { r: 255, g: 255, b: 0, }, // yellow (mid)
      { r: 255, g: 128, b: 0, }, // orange
      { r: 255, g: 0, b: 0, } // red (high)
    ];

  this.paletteScale = (typeof options.paletteScale !== 'undefined') ? options.paletteScale : 1.;
};

export var OcTree.prototype = Object.create(OcTreeBase.prototype);

_readNodeData (dataStream, node) {
  node.value = dataStream.readFloat32();
};

_obtainColor (node) {
  if (this.useFlatColoring) {
    return this.color;
  }

  // Use a simple sigmoid curve to fit values from -inf..inf into 0..1 range
  const value = 1. / (1. + Math.exp(-node.value * this.paletteScale)) * this.palette.length; // Normalize

  const intVal = Math.trunc(value);
  const fracVal = value - intVal;

  if (intVal < 0) { return this.palette[0]; }
  if (intVal >= this.palette.length - 1) { return this.palette[this.palette.length - 1]; }

  // Simple lerp
  return {
    r: fracVal * this.palette[intVal].r + (1. - fracVal) * this.palette[intVal + 1].r,
    g: fracVal * this.palette[intVal].g + (1. - fracVal) * this.palette[intVal + 1].g,
    b: fracVal * this.palette[intVal].b + (1. - fracVal) * this.palette[intVal + 1].b,
  };

};

_checkOccupied (node) {
  return node.value >= this.occupancyThreshold;
};

export var ColorOcTree = function (options) {
  OcTree.call(this, options);
  this.useOwnColor = (typeof options.palette !== 'undefined') && options.colorMode === OcTreeColorMode.COLOR;
};

export var ColorOcTree.prototype = Object.create(OcTree.prototype);

_readNodeData (dataStream, node) {
  node.value = dataStream.readFloat32(); // occupancy
  node.color = {
    r: dataStream.readUint8(), // red
    g: dataStream.readUint8(), // green
    b: dataStream.readUint8(), // blue
  };

};

_obtainColor (node) {
  if (!this.useOwnColor) { return OcTree.prototype._obtainColor.call(this, node); }
  return node.color;
};

_checkOccupied (node) {
  return node.value < this.freeThreshold;
};
