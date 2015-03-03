/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A SceneNode can be used to keep track of a 3D object with respect to a ROS frame within a scene.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * tfClient (optional) - a handle to the TF client
 *  * frameID - the frame ID this object belongs to
 *  * pose (optional) - the pose associated with this object
 *  * object - the THREE 3D object to be rendered
 */
ROS3D.SceneNode = function(options) {
  options = options || {};
  var that = this;
  var tfClient = options.tfClient || null;
  var frameID = options.frameID;
  var object = options.object;
  this.pose = options.pose || new ROSLIB.Pose();

  THREE.Object3D.call(this);

  // add the model
  this.add(object);

  // set the inital pose
  this.updatePose(this.pose);

  // listen for TF updates
  if (tfClient) {
      tfClient.subscribe(frameID, function(tf) { that.transformPose(tf);} );
  }

};

ROS3D.SceneNode.prototype.__proto__ = THREE.Object3D.prototype;

/**
 * Set the pose of the associated model.
 *
 * @param pose - the pose to update with
 */
ROS3D.SceneNode.prototype.updatePose = function(pose) {
  this.position.x = pose.position.x;
  this.position.y = pose.position.y;
  this.position.z = pose.position.z;
  this.quaternion.set(pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);
  this.updateMatrixWorld(true);
};

/**
 * Transform the pose of the associated model.
 * @param transform - A ROS Transform like object which has a translation and orientation property.
 */
ROS3D.SceneNode.prototype.transformPose = function(transform) {
  // apply the transform
  var tf = new ROSLIB.Transform( transform );
  var poseTransformed = new ROSLIB.Pose(this.pose);
  poseTransformed.applyTransform(tf);

  // update the world
  this.updatePose(poseTransformed);
};


/**
 * Removes the TF callback from this SceneNode.
 */
ROS3D.SceneNode.prototype.removeTF = function() {
  if (this.tfClient) {
    this.tfClient.unsubscribe(this.frameID, this.transformPose);
  }
  this.tfCallback = null;
};

/**
 * Removes the TF callback from this SceneNode and
 * all its child nodes.
 */
ROS3D.SceneNode.prototype.removeTFRecursive = function() {
  this.removeTF();
  for (var i = 0; i < this.children.length; i++) {
    if (this.children[i].hasOwnProperty( 'removeTFRecursive' )) {
      this.children[i].removeTFRecursive();
    }
  }
};

