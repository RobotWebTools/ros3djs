import THREE from '../../shims/three/core.js';

import { OcTreeBase, OcTreeColorMode } from './OcTreeBase'

/**
 * @fileOverview
 * @author Peter Sari - sari@photoneo.com
 */

export class OcTree extends OcTreeBase {

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
  
  constructor(options) {
    super(options);

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


  _readNodeData(dataStream, node) {
    node.value = dataStream.readFloat32();
  };

  _obtainColor(node) {
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

  _checkOccupied(node) {
    return node.value >= this.occupancyThreshold;
  };
}
