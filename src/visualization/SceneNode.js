ROS3D.SceneNode = function(options) {
  var sceneNode = this;
  THREE.Object3D.call(this);

  this.tfclient = options.tfclient;
  this.frame_id = options.frame_id;

  this.subscribeTf = function(frame_id) {
    sceneNode.tfclient.subscribe(frame_id, sceneNode.tfUpdate);
  };

  this.setPoseFromServer = function(poseMsg) {
    sceneNode.pose = new ROSLIB.Pose(poseMsg);
    sceneNode.emitServerPoseUpdate();
  };

  this.tfUpdate = function(transformMsg) {
    console.log(transformMsg);
    sceneNode.tfTransform = new ROSLIB.Transform(transformMsg);
    sceneNode.emitServerPoseUpdate();
  };

  this.emitServerPoseUpdate = function() {
    var poseTransformed = new ROSLIB.Pose({
      position : sceneNode.pose.position,
      orientation : sceneNode.pose.orientation
    });
    poseTransformed.applyTransform(sceneNode.tfTransform);
    sceneNode.position.x = poseTransformed.position.x;
    sceneNode.position.y = poseTransformed.position.y;
    sceneNode.position.z = poseTransformed.position.z;

    sceneNode.useQuaternion = true;
    sceneNode.quaternion = new THREE.Quaternion(poseTransformed.orientation.x,
        poseTransformed.orientation.y, poseTransformed.orientation.z, poseTransformed.orientation.w);
    sceneNode.updateMatrixWorld(true);
  };

  sceneNode.add(options.model);
  sceneNode.tfTransform = new ROSLIB.Transform();
  sceneNode.pose = new ROSLIB.Pose();
  sceneNode.setPoseFromServer(options.pose);

  sceneNode.subscribeTf(sceneNode.frame_id);
};
ROS3D.SceneNode.prototype = Object.create(THREE.Object3D.prototype);
