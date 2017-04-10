var assert = chai.assert;

describe('Initialization', function() {

  describe('depthCloud Initialization', function() {
    // Setup Kinect DepthCloud stream
    depthCloud = new ROS3D.DepthCloud({
      url : 'http://'+window.location.hostname+':9999/stream?topic=/depthcloud_encoded&type=vp8&bitrate=250000&quality=best',
      f : 525.0
    });

    it('should return 525.0 for value of f', function() {
      assert.equal(525.0, depthCloud.f);
    });
  });

  // describe('InteractiveMarkerClient Initialization', function() {
  //   // Connect to ROS.
  //   var ros = new ROSLIB.Ros({
  //     url : 'ws://localhost:9090'
  //   });
  //
  //   // Create the main viewer.
  //   var viewer = new ROS3D.Viewer({
  //     divID : 'markers',
  //     width : 800,
  //     height : 600,
  //     antialias : true,
  //     alpha : 0.5
  //   });
  //
  //   var imClient = new ROS3D.InteractiveMarkerClient({
  //     ros : ros,
  //     tfClient : tfClient,
  //     topic : '/basic_controls',
  //     camera : viewer.camera,
  //     rootObject : viewer.selectableObjects
  //   });
  //
  //   it('should return \'/basic_controls\' for value of topic', function() {
  //     assert.equal('/basic_controls', imClient.topic);
  //   });
  // });

});
