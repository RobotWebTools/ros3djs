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
 *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER
 *                         ROS3D.COLLADA_LOADER_2) -- defaults to ROS3D.COLLADA_LOADER_2
 */
ROS3D.MarkerArrayClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.path = options.path || '/';
  this.loader = options.loader || ROS3D.COLLADA_LOADER_2;

  // Markers that are displayed (Map ns+id--Marker)
  this.markers = {};

  // subscribe to MarkerArray topic
  var arrayTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'visualization_msgs/MarkerArray',
    compression : 'png'
  });
  
  arrayTopic.subscribe(function(arrayMessage) {
    
    arrayMessage.markers.forEach(function(message) {
      if(message.action === 0) {
        var updated = false;
        if(message.ns + message.id in that.markers) { // MODIFY
          updated = that.markers[message.ns + message.id].children[0].update(message);
          if(!updated) { // REMOVE
              that.rootObject.remove(that.markers[message.ns + message.id]);
          }
        }
        if(!updated) { // "ADD"
          var newMarker = new ROS3D.Marker({
            message : message,
            path : that.path,
            loader : that.loader
          });
          that.markers[message.ns + message.id] = new ROS3D.SceneNode({
            frameID : message.header.frame_id,
            tfClient : that.tfClient,
            object : newMarker
          });
          that.rootObject.add(that.markers[message.ns + message.id]);
        }
      }
      else if(message.action === 1) { // "DEPRECATED"
        console.warn('Received marker message with deprecated action identifier "1"');
      }
      else if(message.action === 2) { // "DELETE"
        that.rootObject.remove(that.markers[message.ns + message.id]);
        delete that.markers[message.ns + message.id];
      }
      else if(message.action === 3) { // "DELETE ALL"
        for (var m in that.markers){
          that.rootObject.remove(m);
        }
        that.markers = {};
      }
      else {
        console.warn('Received marker message with unknown action identifier "'+message.action+'"');
      }
    });
    
    that.emit('change');
  });
};
ROS3D.MarkerArrayClient.prototype.__proto__ = EventEmitter2.prototype;
