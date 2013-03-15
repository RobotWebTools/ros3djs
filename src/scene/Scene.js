(function (root, factory) {
 if (typeof define === 'function' && define.amd) {
  define(['three','robotwebtools/eventemitter2','robotwebtools/tfclient'],factory);
 }
 else {
  root.SceneNode = factory(root.THREE,root.EventEmitter2,root.TfClient);
 }
}(this, function(THREE,EventEmitter2,TfClient) {
  var SceneNodeHandle = function(options) {
    THREE.Object3D.call(this);

    this.tfclient = options.tfclient;
    this.frame_id = options.frame_id;

    this.add(options.model);
    this.tfTransform = new TfClient.Transform();
    this.pose = new TfClient.Pose();
    this.setPoseFromServer(options.pose);

    this.subscribeTf(this.frame_id);
  };

  SceneNodeHandle.prototype = Object.create(THREE.Object3D.prototype);

  SceneNodeHandle.prototype.subscribeTf = function(frame_id) {
    this.tfclient.subscribe(frame_id,this.tfUpdate.bind(this));
  };

  SceneNodeHandle.prototype.setPoseFromServer = function(poseMsg) {
    this.pose.copy(poseMsg);
    this.emitServerPoseUpdate();
  };

  SceneNodeHandle.prototype.tfUpdate = function(transformMsg) {
    this.tfTransform.copy(transformMsg);
    this.emitServerPoseUpdate();
  };

  SceneNodeHandle.prototype.emitServerPoseUpdate = function() {
    var poseTransformed = new TfClient.Pose(this.pose.position,this.pose.orientation);
    this.tfTransform.apply(poseTransformed);
//    this.emit('server_updated_pose',poseTransformed);

    this.position.x = poseTransformed.position.x;
    this.position.y = poseTransformed.position.y;
    this.position.z = poseTransformed.position.z;

    this.useQuaternion = true;
    this.quaternion = new THREE.Quaternion(poseTransformed.orientation.x,poseTransformed.orientation.y,poseTransformed.orientation.z,poseTransformed.orientation.w);
    this.updateMatrixWorld(true);
  };

  return SceneNodeHandle;
}));
