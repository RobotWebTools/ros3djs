var assert = chai.assert;

describe('Marker', function() {
  var marker;
  afterEach(function () {
    if (marker !== undefined) {
      marker.dispose();
    }
  });
    describe('Arrow', function() {
    })
    describe('Cube', function() {
    })
    describe('Sphere', function() {
    })
    describe('Cylinder', function() {
    })
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
      ]
      message.colors = [
        {r: 1, g: 0, b: 0, a: 1},
        {r: 0, g: 1, b: 0, a: 1},
        {r: 0, g: 0, b: 1, a: 1},
        {r: 1, g: 1, b: 1, a: 1},
      ]
      var options = {};
      options.message = message;
      marker = new ROS3D.Marker(options);
      it('Correct child object', function() {
        assert.equal(marker.children.length, 1);
        assert.equal(marker.children[0].type, 'Line');
      })
    })
    describe('Line List', function() {
    })
    describe('Cube List', function() {
    })
    describe('Sphere List', function() {
    })
    describe('Points', function() {
    })
    describe('Text View Facing', function() {
    })
    describe('Mesh Resource', function() {
    })
    describe('Triangle List', function() {
    })
});
