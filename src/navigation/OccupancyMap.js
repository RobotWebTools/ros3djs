/**
 * @author Peter Sari - sari@photoneo.com
 */

/**
 * Creates a THREE.JS node from a 3D Occupancy map message
 * TODO: TBD
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
      const wrappedFunction = 'get' + wrap.kind;       // Function name that is going to be wrapped from DataView

      this[interfaceFunction] = () => {
        if (this._cursor + wrap.width > this.length) { throw new Error('Cannot read data stream. Overflow.'); }
        const returningValue = this._dataView[wrappedFunction](this._cursor, this.isLittleEndian);
        this._cursor += wrap.width;
        return returningValue;
      };
    });

  Object.defineProperty(this, 'isEnd', { get: () => this.cursor >= this.data.length });
  return this;
};

// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

ROS3D.OcTreeBaseNode = function () {
  this._children = [null, null, null, null, null, null, null, null];
  this.value = null;
};

ROS3D.OcTreeBaseNode.prototype.createChildNodeAt = function (newNode, index) {
  this._children[index % 8] = newNode;
};

ROS3D.OcTreeBaseNode.prototype.hasChildAt = function (index) {
  return this._children[index % 8] !== null;
};

ROS3D.OcTreeBaseNode.prototype.getChildAt = function (index) {
  return this._children[index % 8];
};

ROS3D.OcTreeBaseNode.prototype.isLeafNode = function () {
  for (let i = 0; i < 8; ++i) {
    if (this._children[i] !== null) { return false; }
  }
  return true;
};

ROS3D.OcTreeBaseNode.prototype.hasChildren = function () {
  for (let i = 0; i < 8; ++i) {
    if (this._children[i] !== null) { return true; }
  }
  return false;
};


// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
ROS3D.OcTreeBase = function (params) {

  this.resolution = (typeof params.resolution !== 'undefined') ? params.resolution : 1.;
  this.color = new THREE.Color((typeof params.color !== 'undefined') ? params.color : 'green');

  this._rootNode = null;
  this._treeDepth = 16;
  this._treeMaxKeyVal = 32768;

  // Table of voxel size for each level
  this.nodeSizeTable = new Array(this._treeDepth);
  let _val = this.resolution;
  for (let i = this._treeDepth - 1; i >= 0; --i) {
    this.nodeSizeTable[i] = _val;
    _val = 2. * _val;
  }

  this.object = null;
};

/*
 * 
 */
ROS3D.OcTreeBase.prototype.search = function (key) { return this.searchAtDepth(key, 0); };

/*
 * 
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
  } // end for

  return currentNode;

};

ROS3D.OcTreeBase.prototype._computeBaseCoord = function (key) {
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

/*
 * 
 */

ROS3D.OcTreeBase.prototype._BINARY_UNALLOCATED = 0b00;
ROS3D.OcTreeBase.prototype._BINARY_LEAF_FREE = 0b01;
ROS3D.OcTreeBase.prototype._BINARY_LEAF_OCCUPIED = 0b10;
ROS3D.OcTreeBase.prototype._BINARY_HAS_CHILDREN = 0b11;

ROS3D.OcTreeBase.prototype._newNode = function () { return new ROS3D.OcTreeBaseNode(); };
ROS3D.OcTreeBase.prototype._defaultOccupiedValue = function () { return true; };
ROS3D.OcTreeBase.prototype._defaultFreeValue = function () { return false; };

/*
 * 
 */

ROS3D.OcTreeBase.prototype.readBinary = function (data) {
  if (this._rootNode !== null) {
    delete this._rootNode;
  }
  this._rootNode = this._newNode();

  let dataStream = new InStream(data);

  let stack = new Array();
  stack.push(this._rootNode);

  while (stack.length > 0) {
    let node = stack.pop();

    // 2 bits per children, 16 bit total
    const childAllocationMap = dataStream.readUint16();

    // Insert all children and leaves
    for (let index = 0; index < 8; ++index) {
      const allocation = (childAllocationMap & (0b11 << (2 * index))) >> (2 * index);
      if (allocation !== this._BINARY_UNALLOCATED) {

        if (allocation === this._BINARY_LEAF_FREE) {
          let child = this._newNode();
          child.value = this._defaultFreeValue();
          node.createChildNodeAt(child, index);
        }
        else if (allocation === this._BINARY_LEAF_OCCUPIED) {
          let child = this._newNode();
          child.value = this._defaultOccupiedValue();
          node.createChildNodeAt(child, index);
        }
        else if (allocation === this._BINARY_HAS_CHILDREN) {
          let child = this._newNode();
          child.value = null; // Child occupancy is unknown, we leave it uninitialized
          node.createChildNodeAt(child, index);
        };

      }
    }

    // Proceed to children if it was not initialized (has children)
    for (let index = 7; index >= 0; --index) {
      if (node.hasChildAt(index)) {
        let child = node.getChildAt(index);
        if (child.value === null) { stack.push(child); }
      }
    } // for 

  } // while

};

/**
 * 
 */

ROS3D.OcTreeBase.prototype.read = function (data) {
  if (this._rootNode !== null) {
    delete this._rootNode;
  }
  this._rootNode = this._newNode();

  let dataStream = new InStream(data);

  let stack = new Array();
  stack.push(this._rootNode);

  while (stack.length > 0) {
    let node = stack.pop();

    // Data comes first
    this._readNodeData(dataStream, node);

    const childAllocationMap = dataStream.readUint8();

    // Insert all children and leaves
    for (let i = 7; i >= 0; --i) {
      const hasChildren = (childAllocationMap & (1 << (i))) !== 0;
      // console.log(hasChildren);

      if (hasChildren) {
        let child = this._newNode();
        child.value = null;

        node.createChildNodeAt(i, child);

        stack.push(child);
      } // if 

    } // for  

  } // while
};

ROS3D.OcTreeBase.prototype._readNodeData = function (dataStream, node) {
  // This needs to be implemented by specialized tree
  console.error('Not implemented');
};

/**
* 
*/
ROS3D.OcTreeBase.prototype.buildGeometry = function () {
  console.assert(this._rootNode !== null, 'No tree data');
  const { vertices, normals, colors, indices } = this._buildFaces();

  const geometry = new THREE.BufferGeometry();

  const material = new THREE.MeshBasicMaterial({
    color: 'white',
    flatShading: true,
    vertexColors: THREE.VertexColors,
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
      } // for
    } // if 
  } // while 
};

ROS3D.OcTreeBase.prototype._obtainColor = function (node) {
  return [this.color.r, this.color.g, this.color.b];
};

ROS3D.OcTreeBase.prototype._buildFaces = function () {
  let vertices = [];
  let indices = [];
  let normals = [];
  let colors = [];
  this._traverseLeaves((node, key, depth) => {
    const pos = this._computeBaseCoord(key);
    const size = this.nodeSizeTable[depth];

    for (let face of this.FACES) {
      // Add geomety where there is no neighour voxel
      const neighborKey = [
        key[0] + face.normal[0],
        key[1] + face.normal[1],
        key[2] + face.normal[2],
      ];
      const neighborNode = this.searchAtDepth(neighborKey, depth);
      if (neighborNode === null || neighborNode.hasChildren()) {

        for (let vertex of face.vertices) {
          vertices.push(
            pos[0] + vertex[0] * size,
            pos[1] + vertex[1] * size,
            pos[2] + vertex[2] * size
          );
        };

        const color = this._obtainColor(node);
        colors.push(...color, ...color, ...color, ...color);

        normals.push(...face.normal, ...face.normal, ...face.normal, ...face.normal);

        // TOOD: Toggle Occupied and free voxels
        const indexCount = vertices.length / 3;
        indices.push(
          indexCount, indexCount + 1, indexCount + 2,
          indexCount + 2, indexCount + 1, indexCount + 3
        );
      } // if
    } //for
  });

  return { vertices, normals, colors, indices };
};

ROS3D.OcTreeBase.prototype.FACES = [
  { // left
    normal: [-1, 0, 0,],
    vertices: [
      [0, 1, 0],
      [0, 0, 0],
      [0, 1, 1],
      [0, 0, 1],
    ],
  },
  { // right
    normal: [1, 0, 0,],
    vertices: [
      [1, 1, 1],
      [1, 0, 1],
      [1, 1, 0],
      [1, 0, 0],
    ],
  },
  { // bottom
    normal: [0, -1, 0,],
    vertices: [
      [1, 0, 1],
      [0, 0, 1],
      [1, 0, 0],
      [0, 0, 0],
    ],
  },
  { // top
    normal: [0, 1, 0,],
    vertices: [
      [0, 1, 1],
      [1, 1, 1],
      [0, 1, 0],
      [1, 1, 0],
    ],
  },
  { // back
    normal: [0, 0, -1,],
    vertices: [
      [1, 0, 0],
      [0, 0, 0],
      [1, 1, 0],
      [0, 1, 0],
    ],
  },
  { // front
    normal: [0, 0, 1,],
    vertices: [
      [0, 0, 1],
      [1, 0, 1],
      [0, 1, 1],
      [1, 1, 1],
    ],
  },
];


// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

// Extending the tree

ROS3D.OcTree = function (props) {
  ROS3D.OcTreeBase.prototype.constructor.call(this, props);
};

ROS3D.OcTree.prototype = Object.create(ROS3D.OcTreeBase.prototype);

ROS3D.OcTree.prototype._readNodeData = function (dataStream, node) {

  const value = dataStream.readFloat32();
  // console.log(value);
  node.value = value;

};

// ROS3D.OcTreeBase.prototype._defaultOccupiedValue = function () { return true; };
// ROS3D.OcTreeBase.prototype._defaultFreeValue = function () { return false; };

// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----


ROS3D.ColorOcTree = function (props) {
  ROS3D.OcTreeBase.prototype.constructor.call(this, props);
};

ROS3D.ColorOcTree.prototype = Object.create(ROS3D.OcTreeBase.prototype);

ROS3D.ColorOcTree.prototype._readNodeData = function (dataStream, node) {
  // let dataStream = new InStream(data)


  // TODO https://stackoverflow.com/questions/42699162/javascript-convert-array-of-4-bytes-into-a-float-value-from-modbustcp-read
  // https://github.com/OctoMap/octomap/blob/ebff3f0a53551ad6ccee73e6a09d1edda1916784/octomap/include/octomap/OcTreeBaseImpl.hxx#L771

  // - put this to parent class + add a virtua lfunction that calls for data parse for ea. node


};

// ROS3D.OcTreeBase.prototype._defaultOccupiedValue = function () { return true; };
// ROS3D.OcTreeBase.prototype._defaultFreeValue = function () { return false; };

// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

export default { ROS3D };