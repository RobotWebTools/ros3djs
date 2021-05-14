const rollup = require('rollup');

// plugin that transpiles output into commonjs format
const commonjs = require('@rollup/plugin-commonjs');
// plugin that transpiles es6 to es5 for legacy platforms
const buble = require('@rollup/plugin-buble');
// plugin that shows output file info
const filesize = require('rollup-plugin-filesize');
/// plugin that resolves node module imports
const { nodeResolve } = require('@rollup/plugin-node-resolve');
// plugin that minifies and obfuscates code
const { terser } = require('rollup-plugin-terser');

const pkg = require('./package.json');
const input = 'src-esm/index.js';

const browserGlobals = {
  roslib: 'ROSLIB',
};

const moduleGlobals = {
  roslib: 'ROSLIB',
};

const outputFiles = {
  commonModule: pkg.main,
  esModule: pkg.module,
  browserGlobal: './build/ros3d.js',
  browserGlobalMinified: './build/ros3d.min.js',
};

export default [
  // build main as ES5 in CommonJS format for compatibility
  {
    input,
    output: {
      name: 'ROS3D',
      file: outputFiles.commonModule,
      format: 'cjs',
      globals: {
        ...moduleGlobals,
      }
    },
    external: [
      ...Object.keys(moduleGlobals)
    ],
    plugins: [
      nodeResolve({ browser: true }),
      commonjs(),
      buble(),
      filesize(),
    ],
  },
  // build module as ES5 in ES module format for modern tooling
  {
    input,
    output: {
      name: 'ROS3D',
      file: outputFiles.esModule,
      format: 'es',
      globals: {
        ...moduleGlobals,
      }
    },
    external: [
      ...Object.keys(moduleGlobals)
    ],
    plugins: [
      nodeResolve({ browser: true }),
      commonjs(),
      buble(),
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
      file: outputFiles.browserGlobal,
      format: 'iife',
      globals: {
        ...browserGlobals,
      },
    },
    external: [
      ...Object.keys(browserGlobals),
    ],
    plugins: [
      nodeResolve({ browser: true }),
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
      file: outputFiles.browserGlobalMinified,
      format: 'iife',
      globals: {
        ...browserGlobals,
      },
    },
    external: [
      ...Object.keys(browserGlobals),
    ],
    plugins: [
      nodeResolve({ browser: true }),
      commonjs(),
      filesize(),
      terser(),
    ],
  },
];
