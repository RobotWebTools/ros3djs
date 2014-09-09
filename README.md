ros3djs [![Build Status](https://api.travis-ci.org/RobotWebTools/ros3djs.png)](https://travis-ci.org/RobotWebTools/ros3djs)
=======

#### 3D Visualization Library for use with the ROS JavaScript Libraries
For full documentation, see [the ROS wiki](http://ros.org/wiki/ros3djs) or check out some [working demos](http://robotwebtools.org/).

[JSDoc](http://robotwebtools.org/jsdoc/ros3djs/current/) can be found on the Robot Web Tools website.

This project is released as part of the [Robot Web Tools](http://robotwebtools.org/) effort.

### Usage
Pre-built files can be found in either [ros3d.js](build/ros3d.js) or [ros3d.min.js](build/ros3d.min.js).

Alternatively, you can use the current release via the Robot Web Tools CDN: ([full](http://cdn.robotwebtools.org/ros3djs/current/ros3d.js)) | ([min](http://cdn.robotwebtools.org/ros3djs/current/ros3d.min.js))

### Dependencies
ros3djs depends on:

[EventEmitter2](https://github.com/hij1nx/EventEmitter2). The current supported version is 0.4.14. The current supported version can be found [in this project](include/EventEmitter2/eventemitter2.js) or on the Robot Web Tools CDN: ([full](http://cdn.robotwebtools.org/EventEmitter2/0.4.14/eventemitter2.js)) | ([min](http://cdn.robotwebtools.org/EventEmitter2/0.4.14/eventemitter2.min.js))

[three.js](https://github.com/mrdoob/three.js/). The current supported version is r61. The current supported version can be found [in this project](include/threejs/three.js) or on the Robot Web Tools CDN: ([full](http://cdn.robotwebtools.org/threejs/r61/three.js)) | ([min](http://cdn.robotwebtools.org/threejs/r61/three.min.js))

[THREE.ColladaLoader](https://github.com/mrdoob/three.js/blob/master/examples/js/loaders/ColladaLoader.js). The current supported version is r61. The current supported version can be found [in this project](include/threejs/ColladaLoader.js) or on the Robot Web Tools CDN: ([full](http://cdn.robotwebtools.org/threejs/r61/ColladaLoader.js)) | ([min](http://cdn.robotwebtools.org/threejs/r61/ColladaLoader.min.js))

ColladaLoader2. The current supported version is 0.0.2. The current supported version can be found [in this project](include/ColladaAnimationCompress/ColladaLoader2.js) or on the Robot Web Tools CDN: ([full](http://cdn.robotwebtools.org/ColladaAnimationCompress/0.0.2/ColladaLoader2.js)) | ([min](http://cdn.robotwebtools.org/ColladaAnimationCompress/0.0.2/ColladaLoader2.min.js))

[roslibjs](https://github.com/RobotWebTools/roslibjs). The current supported version is 0.9.0. The current supported version can be found [in this project](include/roslibjs/roslib.js) or on the Robot Web Tools CDN: ([full](http://cdn.robotwebtools.org/roslibjs/0.9.0/roslib.js)) | ([min](http://cdn.robotwebtools.org/roslibjs/0.9.0/roslib.min.js))

### Build
Checkout [utils/README.md](utils/README.md) for details on building.

### License
ros3djs is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS.md](AUTHORS.md) file for a full list of contributors.
