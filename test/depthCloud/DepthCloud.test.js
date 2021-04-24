var assert = chai.assert;

describe('depthCloud', function() {
    // Setup Kinect DepthCloud stream
    var depthCloud = new ROS3D.DepthCloud({
      url : 'http://'+window.location.hostname+':9999/stream?topic=/depthcloud_encoded&type=vp8&bitrate=250000&quality=best',
      f : 525.0
    });

    it('should return 525.0 for value of f', function() {
      assert.equal(525.0, depthCloud.f);
    });
});
