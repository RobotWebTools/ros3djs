const rollup = require('rollup');

// plugin that transpiles output into commonjs format
const commonjs = require('rollup-plugin-commonjs');
// plugin that shows output file info
const filesize = require('rollup-plugin-filesize');
/// plugin that resolves node module imports
const resolve = require('rollup-plugin-node-resolve');
// plugin that minifies and obfuscates code
const uglify = require('rollup-plugin-uglify');

const pkg = require('./package.json');
const input = 'src-esm-test/index.js'

const threeExtensions = {
  // 'three/add/ColladaLoader': 'THREE.ColladaLoader',
  // 'three/add/STLLoader': 'THREE.STLLoader',
}

export default [
  // build main as a CommonJS module for compatibility
  {
    input,
    output: {
      name: 'ROS3D',
      file: pkg.main,
      format: 'cjs',
    },
    external: [
      'roslib',
    ],
    plugins: [
      resolve(),
      commonjs(),
      filesize(),
    ],
  },
  // build module as ES6 module for modern tooling
  {
    input,
    output: {
      name: 'ROS3D',
      file: pkg.module,
      format: 'es',
    },
    external: [
      'roslib',
    ],
    plugins: [
      resolve(),
      commonjs(),
      filesize(),
    ],
  },
  // build browser as IIFE module for script tag inclusion, unminified
  // Usage:
  // <script src="../build/ros3d.js"></script>
  {
    input,
    output: {
      name: 'ROS3D',
      file: pkg.browser.replace(/(\.min)?\.js/, '.js'),
      format: 'iife',
      globals: {
        three: 'THREE',
        roslib: 'ROSLIB',
        eventemitter2: 'EventEmitter2',
        ...threeExtensions,
      },
    },
    external: [
      'three',
      'roslib',
      'eventemitter2',
      ...Object.keys(threeExtensions),
    ],
    plugins: [
      resolve(),
      commonjs(),
      filesize(),
    ],
  },
  // build browser as IIFE module for script tag inclusion, minified
  // Usage:
  // <script src="../build/ros3d.min.js"></script>
  {
    input,
    output: {
      name: 'ROS3D',
      file: pkg.browser.replace(/(\.min)?\.js/, '.min.js'),
      format: 'iife',
      globals: {
        three: 'THREE',
        roslib: 'ROSLIB',
        eventemitter2: 'EventEmitter2',
        ...threeExtensions,
      },
    },
    external: [
      'three',
      'roslib',
      'eventemitter2',
      ...Object.keys(threeExtensions),
    ],
    plugins: [
      resolve(),
      commonjs(),
      filesize(),
      uglify(),
    ],
  },
]
