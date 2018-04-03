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
const input = 'src-esm/index.js'

export default [
  // build main as a CommonJS module for compatibility
  {
    input,
    output: {
      name: 'ROS3D',
      file: pkg.main,
      format: 'cjs',
    },
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
    plugins: [
      resolve(),
      commonjs(),
      filesize(),
    ],
  },
  // build browser as IIFE module for script tag inclusion
  {
    input,
    output: {
      name: 'ROS3D',
      file: pkg.browser,
      format: 'iife',
    },
    plugins: [
      resolve(),
      commonjs(),
      filesize(),
      uglify(),
    ],
  },
]
