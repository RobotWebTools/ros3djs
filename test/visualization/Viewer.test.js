var assert = chai.assert;

describe('Viewer', function() {
  const WIDTH = 500;
  const HEIGHT = 300;
  const DIV_ID = 'ros3d';

  const div = document.createElement('div');
  div.id = DIV_ID;
  const viewer = new ROS3D.Viewer({
    divID: DIV_ID,
    height: HEIGHT,
    width: WIDTH
  });

  it('initializes correctly with canvas passed in as argument', function() {
    assert.isTrue(true);
  });

});

describe('Viewer with canvas as argument', function() {
    const WIDTH = 500;
    const HEIGHT = 300;
    const canvas = document.createElement('canvas');
    canvas.width = WIDTH;
    canvas.height = HEIGHT;
    const viewer = new ROS3D.Viewer({
      canvas: canvas,
      height: HEIGHT,
      width: WIDTH
    });

    it('initializes correctly with canvas passed in as argument', function() {
      assert.isTrue(viewer.renderer.domElement === canvas);
      assert.isTrue(viewer.renderer.getSize().width === WIDTH);
      assert.isTrue(viewer.renderer.getSize().height === HEIGHT);
    });

  });
