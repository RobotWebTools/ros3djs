/**
 * @author Peter Sari - sari@photoneo.com
 */

/**
 * Base node type that represents one voxel as a node of the tree
 */

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
