/**
 * @fileOverview
 * @author Peter Sari - sari@photoneo.com
 */

export class OcTreeBaseNode {

  /**
   * Base node type that represents one voxel as a node of the tree
   */
  
  constructor() {
    this._children = [null, null, null, null, null, null, null, null];
    this.value = null;
  };

  createChildNodeAt(newNode, index) {
    this._children[index % 8] = newNode;
  };

  hasChildAt(index) {
    return this._children[index % 8] !== null;
  };

  getChildAt(index) {
    return this._children[index % 8];
  };

  isLeafNode() {
    for (let i = 0; i < 8; ++i) {
      if (this._children[i] !== null) { return false; }
    }
    return true;
  };

  hasChildren() {
    for (let i = 0; i < 8; ++i) {
      if (this._children[i] !== null) { return true; }
    }
    return false;
  };
}
