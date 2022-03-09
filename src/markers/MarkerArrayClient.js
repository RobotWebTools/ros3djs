/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Nils Berg - berg.nils@gmail.com
 */

/**
 * A MarkerArray client that listens to a given topic.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the MarkerArray
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic - the marker topic to listen to
 *   * tfClient - the TF client handle to use
 *   * rootObject (optional) - the root object to add the markers to
 *   * path (optional) - the base path to any meshes that will be loaded
 */
ROS3D.MarkerArrayClient = function(options) {
  options = options || {};
  this.ros = options.ros;
  this.topicName = options.topic;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.path = options.path || '/';

  // Markers that are displayed (Map ns+id--Marker)
  this.markers = {};
  this.rosTopic = undefined;

  this.subscribe();
};
ROS3D.MarkerArrayClient.prototype.__proto__ = EventEmitter2.prototype;

ROS3D.MarkerArrayClient.prototype.subscribe = function(){
  this.unsubscribe();

  // subscribe to MarkerArray topic
  this.rosTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : this.topicName,
    messageType : 'visualization_msgs/MarkerArray',
    compression : 'png'
  });
  this.rosTopic.subscribe(this.processMessage.bind(this));
};

ROS3D.MarkerArrayClient.prototype.processMessage = function(arrayMessage){
  arrayMessage.markers.forEach(function(message) {
    var key = message.ns + message.id;
    if(message.action === 0) {
      var updated = false;
      if(key in this.markers) { // "MODIFY"
        updated = this.markers[key].children[0].update(message);
        if(!updated) { // "REMOVE"
          this.removeMarker(key);
        }
      }
      if(!updated) { // "ADD"
        var newMarker = new ROS3D.Marker({
          message : message,
          path : this.path,
        });
        this.markers[key] = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : this.tfClient,
          object : newMarker
        });
        this.rootObject.add(this.markers[key]);
      }
    }
    else if(message.action === 1) { // "DEPRECATED"
      console.warn('Received marker message with deprecated action identifier "1"');
    }
    else if(message.action === 2) { // "DELETE"
      this.removeMarker(key);
    }
    else if(message.action === 3) { // "DELETE ALL"
      for (var m in this.markers){
        this.removeMarker(m);
      }
      this.markers = {};
    }
    else {
      console.warn('Received marker message with unknown action identifier "'+message.action+'"');
    }
  }.bind(this));

  this.emit('change');
};

ROS3D.MarkerArrayClient.prototype.unsubscribe = function(){
  if(this.rosTopic){
    this.rosTopic.unsubscribe(this.processMessage);
  }
};

ROS3D.MarkerArrayClient.prototype.removeMarker = function(key) {
  var oldNode = this.markers[key];
  if(!oldNode) {
    return;
  }
  oldNode.unsubscribeTf();
  this.rootObject.remove(oldNode);
  oldNode.children.forEach(child => {
    child.dispose();
  });
  delete(this.markers[key]);
};
