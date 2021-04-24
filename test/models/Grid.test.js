var assert = chai.assert;

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
