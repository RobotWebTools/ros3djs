/*global describe, it, before, beforeEach, after, afterEach, chai, ROS3D, _ */

var assert = chai.assert;

describe('Initialization', function() {


  describe('Arrow', function() {
    var arrow = new ROS3D.Arrow();

    it('matrix should be equal to the proper Matrix4', function() {
      var a = new THREE.Vector3(0, 1, 0);
      arrow.setDirection(a);
      var b = new THREE.Matrix4();
      b.set(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
      assert.isTrue(_.isEqual(arrow.matrix.toArray(), b.toArray()));
    });

    it('scale should be equal to THREE.Vector3(2, 2, 2)', function() {
      arrow.setLength(2);
      assert.isTrue(arrow.scale.equals(new THREE.Vector3(2, 2, 2)));
    });

    it('material.color should be equal to THREE.Color(0xfff000)', function() {
      arrow.setColor(0xfff000);
      assert.equal(arrow.material.color.getHex(), new THREE.Color(0xfff000).getHex());
    });

  });

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

  describe('Grid', function() {
    var grid = new ROS3D.Grid();

    it('should default to 22 children', function() {
      assert.equal(grid.children.length, 22);
    });

    it('each child\'s color is THREE.Color(\'#cccccc\') by default', function() {
      var sample = new THREE.Color('#cccccc').getHex();
      function correctColor(element, index, array) {
        return element.material.color.getHex() === sample;
      }
      assert.isTrue(grid.children.every(correctColor));
    });

    it('each child\'s linewidth is 1 by default', function() {
      function isOne(element, index, array) {
        return element.material.linewidth === 1;
      }
      assert.isTrue(grid.children.every(isOne));
    });

  });

  describe('Marker', function() {
  var marker;
  afterEach(function () {
    if (marker !== undefined) {
      marker.dispose();
    }
  });
    describe('Arrow', function() {
    });
    describe('Cube', function() {
    });
    describe('Sphere', function() {
    });
    describe('Cylinder', function() {
    });
    describe('Line Strip', function() {
      var message = {};
      message.type = ROS3D.MARKER_LINE_STRIP;
      message.color = {r: 0, g: 0, b: 0, a: 1},
      message.pose = {};
      message.pose.position = {x: 0, y: 0, z: 0};
      message.pose.orientation = {x: 0, y: 0, z: 0, w: 1};
      message.scale = {x: 1, y: 0, z: 0};
      message.points = [
        {x: 0, y: 0, z: 0},
        {x: 1, y: 0, z: 0},
        {x: 1, y: 1, z: 0},
        {x: 0, y: 1, z: 0},
      ];
      message.colors = [
        {r: 1, g: 0, b: 0, a: 1},
        {r: 0, g: 1, b: 0, a: 1},
        {r: 0, g: 0, b: 1, a: 1},
        {r: 1, g: 1, b: 1, a: 1},
      ];
      var options = {};
      options.message = message;
      marker = new ROS3D.Marker(options);
      it('Correct child object', function() {
        assert.equal(marker.children.length, 1);
        assert.equal(marker.children[0].type, 'Line');
      });
    });
    describe('Line List', function() {
    });
    describe('Cube List', function() {
    });
    describe('Sphere List', function() {
    });
    describe('Points', function() {
    });
    describe('Text View Facing', function() {
    });
    describe('Mesh Resource', function() {
    });
    describe('Triangle List', function() {
    });
  });

});
