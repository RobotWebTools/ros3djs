/**
 * @author Peter Sari - sari@photoneo.com
 */

ROS3D.ColorOcTree = function(options) {
  ROS3D.OcTree.call(this, options);
  this.useOwnColor = (typeof options.palette !== 'undefined') && options.colorMode === ROS3D.OcTreeColorMode.COLOR;
};

ROS3D.ColorOcTree.prototype.__proto__ = ROS3D.OcTree.prototype;

ROS3D.ColorOcTree.prototype._readNodeData = function (dataStream, node) {
  node.value = dataStream.readFloat32(); // occupancy
  node.color = {
    r: dataStream.readUint8(), // red
    g: dataStream.readUint8(), // green
    b: dataStream.readUint8(), // blue
  };

};

ROS3D.ColorOcTree.prototype._obtainColor = function (node) {
  if (!this.useOwnColor) { return ROS3D.OcTree.prototype._obtainColor.call(this, node); }
  return node.color;
};
