# ros3djs

[![CI](https://github.com/RobotWebTools/ros3djs/actions/workflows/main.yml/badge.svg)](https://github.com/RobotWebTools/ros3djs/actions/workflows/main.yml)


#### 3D Visualization Library for use with the ROS JavaScript Libraries

For full documentation, see [the ROS wiki](http://ros.org/wiki/ros3djs) or check out some [working demos](http://robotwebtools.org/demos.html).

[JSDoc](http://robotwebtools.org/ros3djs) can be found on the Robot Web Tools website.

This project is released as part of the [Robot Web Tools](http://robotwebtools.org/) effort.

### Usage

Pre-built files can be found in either [ros3d.js](build/ros3d.js) or [ros3d.min.js](build/ros3d.min.js).


Alternatively, you can use the current release via the [JsDelivr](https://www.jsdelivr.com/) CDN: ([full](https://cdn.jsdelivr.net/npm/ros3d@1/build/ros3d.js)) | ([min](https://cdn.jsdelivr.net/npm/ros3d@1/build/ros3d.min.js))

### Dependencies

ros3djs depends on:

[EventEmitter2](https://github.com/hij1nx/EventEmitter2). The current supported version is 6.4. The current supported version can be found on the JsDeliver CDN: ([full](https://cdn.jsdelivr.net/npm/eventemitter2@6.4/lib/eventemitter2.js)) | ([min](https://cdn.jsdelivr.net/npm/eventemitter2@6.4/lib/eventemitter2.min.js))

[three.js](https://github.com/mrdoob/three.js/). The current supported version is r89. The current supported version can be found on the Robot Web Tools CDN: ([full](https://static.robotwebtools.org/threejs/r89/three.js)) | ([min](https://static.robotwebtools.org/threejs/r89/three.min.js))

[THREE.ColladaLoader](https://github.com/mrdoob/three.js/blob/master/examples/js/loaders/ColladaLoader.js). The current supported version is r89. The current supported version can be found on the Robot Web Tools CDN: ([full](https://static.robotwebtools.org/threejs/r89/ColladaLoader.js))

[THREE.STLLoader](https://github.com/mrdoob/three.js/blob/master/examples/js/loaders/STLLoader.js). The current supported version is r89. The current supported version can be found on the Robot Web Tools CDN: ([full](https://static.robotwebtools.org/threejs/r89/STLLoader.js))

(ROS)ColladaLoader. We support patched version of ColladaLoader to workaround ros-visualization/rviz#1045. This version can be found on the Robot Web Tools CDN: ([full](https://static.robotwebtools.org/ros3djs/0.18.0/ColladaLoader.js))

[roslibjs](https://github.com/RobotWebTools/roslibjs). The current supported version is 1.3.0. The current supported version can be found on the JsDeliver CDN: ([full](https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.js)) | ([min](https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js))

### Build

[Grunt](http://gruntjs.com/) is used for building, including concatenating, minimizing, documenting, linting, and testing.

### Install Grunt and its Dependencies

#### Ubuntu 18.04/20.04

 1. Install Node.js and its package manager, NPM
   * `sudo apt-get install nodejs nodejs-legacy npm` or any other way you like
 2. Install Grunt
   * `sudo npm install -g grunt-cli`
   * (optional) `sudo rm -rf ~/.npm ~/tmp`
 3. Install the Grunt tasks specific to this project
   * `cd /path/to/ros3djs/`
   * `npm install .`

### Build with Grunt

Before proceeding, please confirm you have installed the dependencies above.

To run the build tasks:

 1. `cd /path/to/ros3djs/`
 2. `grunt build`

`grunt build` will concatenate and minimize the files under src and replace ros3d.js and ros3d.min.js in the build directory. It will also run the linter and test cases. This is what [Travis CI](https://travis-ci.org/RobotWebTools/ros3djs) runs when a Pull Request is submitted.

`grunt dev` will watch for any changes to any of the src/ files and automatically concatenate and minimize the files. This is ideal for those developing as you should only have to run `grunt dev` once.

`grunt doc` will rebuild all JSDoc for the project.

### Testing

Utilizes [mocha](https://mochajs.org/) and [chai](http://chaijs.com/) for in browser testing.

To run tests simply open `tests/index.html` in a web browser

### Examples

There are a variety of [examples](examples) of the different things that can be done with ros3djs.

There are also some examples of how ros3djs can be used in different environments:

- [Classic html script tag inclusion](examples)
- [Modular html script tag inclusion](examples/html-import)

### License

ros3djs is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors

See the [AUTHORS.md](AUTHORS.md) file for a full list of contributors.
