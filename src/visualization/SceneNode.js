/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A SceneNode can be used to keep track of a 3D object with respect to a ROS frame within a scene.
 *
 * @constructor
 * @param options - object with following keys:
 *  * tfClient - a handle to the TF client
 *  * frameID - the frame ID this object belongs to
 *  * model - the THREE 3D object to be rendered
 */
ROS3D.SceneNode = function(options) {
  var options = options || {};
  var that = this;
  this.tfClient = options.tfClient;
  this.frameID = options.frameID;
  this.model = options.model;

  THREE.Object3D.call(this);
  this.useQuaternion = true;

  // add the model
  this.add(this.model);

  // listen for TF updates
  this.tfClient.subscribe(this.frameID,
      function(msg) {
        // apply the transform
        var tf = new ROSLIB.Transform(msg);
        var poseTransformed = new ROSLIB.Pose();
        poseTransformed.applyTransform(tf);

        // update the world
        that.position.x = poseTransformed.position.x;
        that.position.y = poseTransformed.position.y;
        that.position.z = poseTransformed.position.z;
        that.quaternion = new THREE.Quaternion(poseTransformed.orientation.x,
            poseTransformed.orientation.y, poseTransformed.orientation.z,
            poseTransformed.orientation.w);
        that.updateMatrixWorld(true);
      });
};
ROS3D.SceneNode.prototype.__proto__ = THREE.Object3D.prototype;
