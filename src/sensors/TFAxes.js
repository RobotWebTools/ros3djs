/**
 * @author Jihoon Lee - jihoon.lee@kakaobrain.com
 */
/**
 * An Axes node can be used to display the axis of a particular coordinate frame.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * frame_id - the frame id to visualize axes
 *   * tfClient - the TF client handle to use
 *   * shaftRadius (optional) - the radius of the shaft to render
 *   * headRadius (optional) - the radius of the head to render
 *   * headLength (optional) - the length of the head to render
 *   * scale (optional) - the scale of the frame (defaults to 1.0)
 *   * lineType (optional) - the line type for the axes. Supported line types:
 *                           'dashed' and 'full'.
 *   * lineDashLength (optional) - the length of the dashes, relative to the length of the axis.
 *                                 Maximum value is 1, which means the dash length is
 *                                 equal to the length of the axis. Parameter only applies when
 *                                 lineType is set to dashed.
 */
ROS3D.TFAxes = function(options) {
  THREE.Object3D.call(this);
  var that = this;
  options = options || {};

  this.frame_id = options.frame_id;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.axes = new ROS3D.Axes(
    {
      shaftRadius: options.shaftRadius || 0.025,
      headRadius: options.headRaidus || 0.07,
      headLength: options.headLength || 0.2,
      scale: options.scale || 1.0,
      lineType: options.lineType || 'full',
      lineDashLength: options.lineDashLength || 0.1
    });

  this.sn = new ROS3D.SceneNode({
    frameID: this.frame_id,
    tfClient : this.tfClient,
    object : this.axes
  });

  this.rootObject.add(this.sn);

};


ROS3D.TFAxes.prototype.__proto__ = THREE.Object3D.prototype;
