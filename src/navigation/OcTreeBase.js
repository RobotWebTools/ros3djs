/**
 * @author Peter Sari - sari@photoneo.com
 */

/**
 * Toggles voxel visibility
 *
 *    * `occupied` - only voxels that are above or equal to the occupation threshold are shown
 *    * `free` - only voxels that are below the occupation threshold are shown
 *    * `all` - all allocated voxels are shown
 */
ROS3D.OcTreeVoxelRenderMode = {
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
ROS3D.OcTreeColorMode = {
  SOLID: 'solid',
  OCCUPANCY: 'occupancy',
  COLOR: 'color'
};

// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

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
ROS3D.OcTreeBase = function(options) {

  this.resolution = (typeof options.resolution !== 'undefined') ? options.resolution : 1.;
  this.color = new THREE.Color((typeof options.color !== 'undefined') ? options.color : 'green');
  this.opacity = (typeof options.opacity !== 'undefined') ? options.opacity : 1.;

  this.voxelRenderMode = (typeof options.voxelRenderMode !== 'undefined') ? options.voxelRenderMode : ROS3D.OcTreeVoxelRenderMode.OCCUPIED;

  this._rootNode = null;
  this._treeDepth = 16;
  this._treeMaxKeyVal = 32768;

  this._BINARY_UNALLOCATED = 0b00;
  this._BINARY_LEAF_FREE = 0b01;
  this._BINARY_LEAF_OCCUPIED = 0b10;
  this._BINARY_HAS_CHILDREN = 0b11;

  this._BINARY_CHILD_BUILD_TABLE = {};

  this._BINARY_CHILD_BUILD_TABLE[this._BINARY_LEAF_FREE] = function (child) {
    child.value = this._defaultFreeValue;
  };

  this._BINARY_CHILD_BUILD_TABLE[this._BINARY_LEAF_OCCUPIED] = function (child) {
    child.value = this._defaultOccupiedValue;
  };

  this._BINARY_CHILD_BUILD_TABLE[this._BINARY_HAS_CHILDREN] = function (child) {
    child.value = null;
  };

  /**
   * Table which we are building the geometry data from.
   */
  this._FACES = [
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
ROS3D.OcTreeBase.prototype.searchAtDepth = function (key, depth) {
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
ROS3D.OcTreeBase.prototype._computeCoordFromKey = function (key) {
  return key.map(keyVal => this.resolution * (keyVal - this._treeMaxKeyVal));
};

ROS3D.OcTreeBase.prototype._computeChildIdx = function (key, depth) {
  let pos = 0;
  if (key[0] & (1 << depth)) { pos += 1; }
  if (key[1] & (1 << depth)) { pos += 2; }
  if (key[2] & (1 << depth)) { pos += 4; }

  return pos;
};

ROS3D.OcTreeBase.prototype._computeKeyFromChildIdx = function (index, offset, depth) {
  const diff = this._treeDepth - depth - 1;

  return [
    offset[0] + (!!(index & 1) << diff),
    offset[1] + (!!(index & 2) << diff),
    offset[2] + (!!(index & 4) << diff),
  ];

};

ROS3D.OcTreeBase.prototype._adjustKeyAtDepth = function (key, depth) {
  // generate appropriate key_at_depth for queried depth
  let diff = this._treeDepth - depth;
  if (diff === 0) { return key; }

  return key.map(keyVal => (((keyVal - this._treeMaxKeyVal) >> diff) << diff) + (1 << (diff - 1)) + this._treeMaxKeyVal);
};

ROS3D.OcTreeBase.prototype._newNode = function () { return new ROS3D.OcTreeBaseNode(); };

/*
 * Reads and builds a tree which was represented in a binary form from a message
 * Binary form only contains the tree structure to be allocated, all the data of voxels are stripped,
 * occupation is represented as a binary value.
 * Each node is represented as a 2-bit value which makes up the 8 child nodes of the parent (16 bits in total)
 * starting with the root node.
 */

ROS3D.OcTreeBase.prototype.readBinary = function (data) {
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

/**
 * Reads a full tree (with node data) from a message.
 * A pacjet starts with the node data, followed by the allocation map of their children.
 * Each type of tree has different data structure @see ROS3DJS.OcTreeBase._readNodeData
 */
ROS3D.OcTreeBase.prototype.read = function (data) {
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
ROS3D.OcTreeBase.prototype._readNodeData = function (dataStream, node) {
  // This needs to be implemented by specialized tree
  console.error('Not implemented');
};

/**
* Builds up THREE.js geometry from tree data.
*/
ROS3D.OcTreeBase.prototype.buildGeometry = function () {
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

ROS3D.OcTreeBase.prototype._traverseLeaves = function (callback) {
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
ROS3D.OcTreeBase.prototype._obtainColor = function (node) {
  return this.color;
};

ROS3D.OcTreeBase.prototype._checkOccupied = function (node) {
  return node.value !== false;
};

ROS3D.OcTreeBase.prototype._buildFaces = function () {
  let geometry = {
    vertices: [],
    indices: [],
    normals: [],
    colors: [],

    _insertFace: function (face, pos, size, color) {
      const indexCount = this.vertices.length / 3;

      face.vertices.forEach(function(vertex) {
        this.vertices.push(
          pos[0] + vertex[0] * size,
          pos[1] + vertex[1] * size,
          pos[2] + vertex[2] * size
        );
      });

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
          face.childIndex.forEach(function(childIndex) {
            if (node.hasChildAt(childIndex)) {
              const child = node.getChildAt(childIndex);

              // filter occupancy
              const isOccupied = this._checkOccupied(node);
              const isNeedsToRender = (isOccupied && voxelRenderMode === ROS3D.OcTreeVoxelRenderMode.OCCUPIED) || (!isOccupied && voxelRenderMode === ROS3D.OcTreeVoxelRenderMode.FREE);

              if (isNeedsToRender) { stack.push(child); }
            }
            else {
              return true;
            }
          });
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
    if (!isOccupied && this.voxelRenderMode === ROS3D.OcTreeVoxelRenderMode.OCCUPIED) { return; }

    // Hide occuped voxels if set.
    if (isOccupied && this.voxelRenderMode === ROS3D.OcTreeVoxelRenderMode.FREE) { return; }

    this._FACES.forEach(function(face) {
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

    });

  });

  // return geometry;
  return {
    vertices: geometry.vertices,
    normals: geometry.normals,
    colors: geometry.colors,
    indices: geometry.indices
  };

};
