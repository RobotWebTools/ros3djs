/**
 * @author Peter Sari - sari@photoneo.com
 */

/**
 * Creates a THREE.JS node from a 3D Occupancy map message
 * TODO: TBD
 */

let ROS3D = {};

let InStream = function (data) {
  // Quick and dirty helper class
  this.data = data;
  this.cursor = 0;
};

InStream.prototype.readBytes = function (length) {
  if (this.cursor + length > this.data.length) { throw new Error('Cannot read, overflow'); }
  const returningData = Array.prototype.slice.call(
    this.data,
    this.cursor,
    this.cursor + length
  );
  this.cursor += length;
  return returningData;
};

// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

ROS3D.OcTreeBaseNode = function () {
  this.children = [null, null, null, null, null, null, null, null];
  this.data = null;
};

ROS3D.OcTreeBaseNode.prototype.createChildNodeAt = function (newNode, index) {
  this.children[index % 8] = newNode;
};

ROS3D.OcTreeBaseNode.prototype.hasChildAt = function (index) {
  return this.children[index % 8] !== null;
};

ROS3D.OcTreeBaseNode.prototype.getChildAt = function (index) {
  return this.children[index % 8];
};

// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
ROS3D.OcTreeBase = function (params) {
  this.resolution = params.resolution || 1.;
  this.rootNode = null;
};

ROS3D.OcTreeBase.prototype.readBinary = function (data) {
  if (this.rootNode !== null) {
    delete this.rootNode;
  }
  this.rootNode = this._newNode();

  this._readBinaryNode(new InStream(data), this.rootNode);
};

ROS3D.OcTreeBase.prototype._newNode = function(){ return new ROS3D.OcTreeBaseNode(); };

ROS3D.OcTreeBase.prototype._BINARY_UNKNOWN = 0b00;
ROS3D.OcTreeBase.prototype._BINARY_LEAF_FREE = 0b10;
ROS3D.OcTreeBase.prototype._BINARY_LEAF_OCCUPIED = 0b01;
ROS3D.OcTreeBase.prototype._BINARY_HAS_CHILDREN = 0b11;

ROS3D.OcTreeBase.prototype._readBinaryNode = function (dataStream, node) {
  if (node === null) { throw new Error('Nullptr'); }

  // 2 bits per children, 16 bit total
  const childrenDataSegment = dataStream.readBytes(2).map(x => (x >>> 0) & 255);
  const childrenData = childrenDataSegment[0] | (childrenDataSegment[1] << 8);

  // Insert all children and leaves
  for (let i = 0; i < 8; ++i) {
    const nodeValue = (childrenData & (0b11 << (2 * i))) >> (2 * i);
    if (nodeValue !== this._BINARY_UNKNOWN) { this._binaryNodeFactoryTable[nodeValue].bind(this)(node, i); }
  }

  // Proceed to children
  for (let i = 0; i < 8; ++i) {
    if (node.hasChildAt(i)) {
      let child = node.getChildAt(i);
      this._readBinaryNode(dataStream, child);
    }
  }
};

ROS3D.OcTreeBase.prototype._binaryNodeFactoryTable = {};

ROS3D.OcTreeBase.prototype._binaryNodeFactoryTable[
  ROS3D.OcTreeBase.prototype._BINARY_LEAF_FREE
] = function (node, index) {
  let child = this._newNode();
  child.data = false;
  node.createChildNodeAt(child, index);
};

ROS3D.OcTreeBase.prototype._binaryNodeFactoryTable[
  ROS3D.OcTreeBase.prototype._BINARY_LEAF_OCCUPIED
] = function (node, index) {
  let child = this._newNode();
  child.data = true;
  node.createChildNodeAt(child, index);
};

ROS3D.OcTreeBase.prototype._binaryNodeFactoryTable[
  ROS3D.OcTreeBase.prototype._BINARY_HAS_CHILDREN
] = function (node, index) {
  let child = this._newNode();
  child.data = null; // child is unkown, we leave it uninitialized
  node.createChildNodeAt(child, index);
};

// ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----

export default { ROS3D };
