/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Nils Berg - berg.nils@gmail.com
 * @author Peter Soetens - peter@thesourceworks.com
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
  this.arrayTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'visualization_msgs/MarkerArray',
    compression : 'png',
    queue_length  : 2
  });
  
  this.arrayTopic.subscribe(function(arrayMessage) {

    arrayMessage.markers.forEach(function(message) {

      // We need to delete always to make sure no markers are overlaying on each other
      if ( that.markers[message.ns + message.id] ) {
        that.rootObject.remove(that.markers[message.ns + message.id]);
        that.markers[message.ns + message.id].removeTF();
      }

      if (message.action === 2 ) {
        // delete action, already deleted.
      } else if ( message.action === 0 ) {
	// create action

        var newMarker = new ROS3D.Marker({
          message : message,
          path : that.path,
          loader : that.loader
        });

        that.markers[message.ns + message.id] = new ROS3D.SceneNode({
          frameID : message.header.frame_id.trimLeft('/'),
          tfClient : that.tfClient,
          object : newMarker
        });
        that.rootObject.add(that.markers[message.ns + message.id]);
      }
    });
    
    that.emit('change');
  });

    this.removeArray = function() {
        var mac = this;
	mac.arrayTopic.unsubscribe();
        for (var key in mac.markers) {
            if (mac.markers.hasOwnProperty(key)) {
                mac.rootObject.remove( mac.markers[key] );
                mac.markers[key].removeTF();
            }
        }
    };
};
ROS3D.MarkerArrayClient.prototype.__proto__ = EventEmitter2.prototype;
