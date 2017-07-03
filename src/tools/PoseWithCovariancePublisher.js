/**
 * @author Jihoon Lee - lee@magazino.eu
 */


/**
 * A tool to publish pose with covariance
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * ros - a handle to the ROS connection
 *  * mapClient - occupancy gridmap client
 *  * topic (optional) - the topic to subscribe to, like '/basic_controls', if not provided use subscribe() to start message receiving
 *  * rootObject (optional) - the root object to add this marker to
 *  * covariance (optional) - covariance matrix to publish
 */
ROS3D.PoseWithCovariancePublisher = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  this.mapClient = options.mapClient;
  this.topicName = options.topic | 'pose_with_covariance';
  this.rootObject = options.rootObject || new THREE.Object3D();
  this.covariance = options.covariance || [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942];

  this.advertiseTopic(this.topicName);

  this.mapClient.on('change', this.updateMap.bind(this));

  this.map = this.mapClient.currentGrid;
  this.select = false;
  this.arrow = null;
  this.arrowOrigin = null;
  var headDiameter = 0.5;

  that.mousedown = function(event3D) {

    that.arrowOrigin = event3D.intersection.point.clone();
    that.normalOrigin = new THREE.Vector3(0,0,1);
  };

  that.mouseup = function(event3D) {
    if(that.arrow) {
      var intersection = ROS3D.intersectPlane(event3D.mouseRay, that.arrowOrigin, that.normalOrigin);
      var direction = new THREE.Vector3();
      direction.subVectors(intersection, that.arrowOrigin).normalize();

      var qt = that.arrow.quaternion.clone();


      // somehow you have to rotate it around 90 degree to match between scene and map frame...
      var q = new THREE.Quaternion();
      q.setFromAxisAngle(new THREE.Vector3(0,0,1), Math.PI / 2);
      qt.multiplyQuaternions(qt, q);

      that.publish(that.arrowOrigin, qt);
      //that.publish(that.arrowOrigin, qt);

      // that.cleanup();
    }
  };

  that.mousemove = function(event3D) {
    if(that.arrowOrigin) {
      // intersect mouse ray with plane
      var intersection = ROS3D.intersectPlane(event3D.mouseRay, that.arrowOrigin, that.normalOrigin);
      var direction = new THREE.Vector3();
      direction.subVectors(intersection, that.arrowOrigin);

      var len = direction.length();
      var headLength = len * 0.23;

      direction.normalize();
      if(that.arrow === null) {
        that.arrow = that.createArrow(that.arrowOrigin, direction, len, headLength, headDiameter);
        that.rootObject.add(that.arrow);
      }
      else {
        that.arrow.setDirection(direction);
        that.arrow.setLength(len);
      }
    }
  };

  var continuePropagation = function (event) {
    event.continuePropagation();
  };

  that.resetEvents = function() {
    if(that.select) {
      that.map.addEventListener('mousedown', that.mousedown);
      that.map.addEventListener('mouseup', that.mouseup);
      this.map.addEventListener('mousemove', that.mousemove);
      this.map.addEventListener('mouseover', continuePropagation);
      this.map.addEventListener('mouseout', continuePropagation);
    }
    else {
      this.map.removeEventListener('mousedown', that.mousedown);
      this.map.removeEventListener('mouseup', that.mouseup);
      this.map.removeEventListener('mousemove', that.mousemove);
      this.map.removeEventListener('mouseover', continuePropagation);
      this.map.removeEventListener('mouseout', continuePropagation);
    }
  };
};

ROS3D.PoseWithCovariancePublisher.prototype.__proto__ = THREE.Object3D.prototype;


/**
 * Advertise the pose with covariance
 *
 * @param topic - the topic to publish
 */
ROS3D.PoseWithCovariancePublisher.prototype.advertiseTopic = function(topic) {
  this.publisher = new ROSLIB.Topic({
    ros : this.ros,
    name : topic,
    messageType : 'geometry_msgs/PoseWithCovarianceStamped',
    compression : 'png'
  });
  this.publisher.advertise();
};

/**
 * Publish the given pose.
 *
 * @param msg - the message to publish
 */
ROS3D.PoseWithCovariancePublisher.prototype.publish = function(p, o) {
  var msg = new ROSLIB.Message({
    header: {
      frame_id: this.mapClient.sceneNode.frameID
    },
    pose: {
      pose: {
        position: {
          x: p.x,
          y: p.y,
          z: p.z
        },
        orientation: {
          x: o.x,
          y: o.y,
          z: o.z,
          w: o.w
        }
      },
      covariance: this.covariance
    },
  });
  this.publisher.publish(msg);
};

/**
 * listen to update of map and replace the object in root object
 */
ROS3D.PoseWithCovariancePublisher.prototype.updateMap = function() {
  this.rootObject.remove(this.map);
  this.map = this.mapClient.currentGrid;
};

ROS3D.PoseWithCovariancePublisher.prototype.selectable = function(enable) {
  this.cleanup();
  this.select = enable;
  this.resetEvents();
};

ROS3D.PoseWithCovariancePublisher.prototype.createArrow = function(origin, direction, len, headLength, headDiameter) {
  var arrow = new ROS3D.Arrow2({
                direction : direction,
                origin : origin,
                length : len,
                headLength : headLength,
                headDiameter : headDiameter
              });
  var color = new THREE.Color();
  arrow.setColor(0xff0000);
  return arrow;
};

ROS3D.PoseWithCovariancePublisher.prototype.cleanup = function() {

  if(this.arrow !== null) {
    this.rootObject.remove(this.arrow);
    this.arrow.dispose();
    this.arrow = null;
    this.arrowOrigin = null;
  }
};
