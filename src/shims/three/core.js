import * as _THREE from 'three/build/three.module.js';

const THREE = Object.assign({}, _THREE, );
export default THREE;

// While we are at it (bundling Three.js into ROS3D, which is something we need
// to avoid in a future release so that users can pass their version of THREE
// from the outside), we should at least expose it as a global so that they
// can use the THREE global variable, otherwise they have no way to use
// Three.js APIs that ROS3D embeds inside of it, and therefore the user may do
// something like import a second version of Three.js which can cause issues
// (I've seen this problem happen in real-world ROS3D projects, especially with
// roboticists or data scientists that are inexperienced with web
// development).
globalThis.THREE = THREE;
