/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An occupancy grid client that listens to a given map topic.
 * 
 * Emits the following events:
 *  * 'change' - there was an update or change in the marker
 *  
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * rootObject (optional) - the root object to add this marker to
 */
ROS3D.OccupancyGridClient = function(options) {
  var that = this;
  var options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/map';
  this.continuous = options.continuous;
  this.rootObject = options.rootObject || new THREE.Object3D();

  // current grid that is displayed
  this.currentGrid = null;

  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'nav_msgs/OccupancyGrid',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {
    // check for an old map
    if (that.currentGrid) {
      that.rootObject.remove(that.currentGrid);
    }

    that.currentGrid = new ROS3D.OccupancyGrid({
      message : message
    });
    that.rootObject.add(that.currentGrid);

    that.emit('change');
    
    // check if we should unsubscribe
    if(!that.continuous) {
      rosTopic.unsubscribe();
    }
  });
};
ROS3D.OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;
