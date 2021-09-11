import {OcTree, OcTreeColorMode} from './OcTree.js';

export class ColorOcTree extends OcTree {
  constructor(options) {
    super(options);
    this.useOwnColor = typeof options.palette !== 'undefined' && options.colorMode === OcTreeColorMode.COLOR;
  }

  _readNodeData(dataStream, node) {
    node.value = dataStream.readFloat32(); // occupancy
    node.color = {
      r: dataStream.readUint8(), // red
      g: dataStream.readUint8(), // green
      b: dataStream.readUint8(), // blue
    };
  }

  _obtainColor(node) {
    if (!this.useOwnColor) {
      return OcTree.prototype._obtainColor.call(this, node);
    }
    return node.color;
  }
}
