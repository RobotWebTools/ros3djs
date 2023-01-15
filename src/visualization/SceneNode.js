/**
 * @fileOverview
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A SceneNode can be used to keep track of a 3D object with respect to a ROS frame within a scene.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * tfClient - a handle to the TF client
 *  * frameID - the frame ID this object belongs to
 *  * pose (optional) - the pose associated with this object
 *  * object - the THREE 3D object to be rendered
 */
ROS3D.SceneNode = function(options) {
  THREE.Object3D.call(this);
  options = options || {};
  this.tfClient = options.tfClient;
  this.frameID = options.frameID;
  var object = options.object;
  this.pose = options.pose || new ROSLIB.Pose();

  // Do not render this object until we receive a TF update
  this.visible = false;
  this.layers.disable(0);

  // add the model
  this.add(object);

  // set the inital pose
  this.updatePose(this.pose);

  // save the TF handler so we can remove it later
  this.tfUpdate = function(msg) {

    // apply the transform
    var tf = new ROSLIB.Transform(msg);
    var poseTransformed = new ROSLIB.Pose(this.pose);
    poseTransformed.applyTransform(tf);

    // update the world
    this.updatePose(poseTransformed);
    this.visible = true;
    this.layers.enable(0);
  };

  // listen for TF updates
  this.tfUpdateBound = this.tfUpdate.bind(this);
  this.tfClient.subscribe(this.frameID, this.tfUpdateBound);
};
ROS3D.SceneNode.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Set the pose of the associated model.
 *
 * @param pose - the pose to update with
 */
ROS3D.SceneNode.prototype.updatePose = function(pose) {
  this.position.set( pose.position.x, pose.position.y, pose.position.z );
  this.quaternion.set(pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);
  this.updateMatrixWorld(true);
};

ROS3D.SceneNode.prototype.unsubscribeTf = function() {
  this.tfClient.unsubscribe(this.frameID, this.tfUpdateBound);
};
