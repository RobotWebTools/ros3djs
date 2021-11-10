var assert = chai.assert;

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
