var assert = chai.assert;

describe('Initialization', () => {


  describe('Arrow', () => {
    arrow = new ROS3D.Arrow();

    it('matrix should be equal to the proper Matrix4', () => {
      var a = new THREE.Vector3(0, 1, 0)
      arrow.setDirection(a);
      b = new THREE.Matrix4()
      b.set(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
      assert.isTrue(_.isEqual(arrow.matrix.toArray(), b.toArray()));
    });

    it('scale should be equal to THREE.Vector3(2, 2, 2)', () => {
      arrow.setLength(2);
      assert.isTrue(arrow.scale.equals(new THREE.Vector3(2, 2, 2)));
    });

    it('material.color should be equal to THREE.Color(0xfff000)', () => {
      arrow.setColor(0xfff000);
      assert.equal(arrow.material.color.getHex(), new THREE.Color(0xfff000).getHex());
    });

  });

  describe('depthCloud', () => {
    // Setup Kinect DepthCloud stream
    depthCloud = new ROS3D.DepthCloud({
      url : 'http://'+window.location.hostname+':9999/stream?topic=/depthcloud_encoded&type=vp8&bitrate=250000&quality=best',
      f : 525.0
    });

    it('should return 525.0 for value of f', () => {
      assert.equal(525.0, depthCloud.f);
    });
  });

  describe('Grid', () => {
    grid = new ROS3D.Grid();

    it('should default to 22 children', () => {
      assert.equal(grid.children.length, 22);
    });

    it('each child\'s color is THREE.Color(\'#cccccc\') by default', () => {
      sample = new THREE.Color('#cccccc').getHex()
      function correctColor(element, index, array) {
        return element.material.color.getHex() == sample;
      }
      assert.isTrue(grid.children.every(correctColor));
    });

    it('each child\'s linewidth is 1 by default', () => {
      function isOne(element, index, array) {
        return element.material.linewidth == 1;
      }
      assert.isTrue(grid.children.every(isOne));
    });

  });

});
