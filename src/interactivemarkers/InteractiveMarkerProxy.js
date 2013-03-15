(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['eventemitter2','./tfclient'], factory);
  }
  else {
    root.ImProxy = factory(root.EventEmitter2,root.TfClient);
  }
}(this, function (EventEmitter2, TfClient) {

  var ImProxy = {};

  var Client = ImProxy.Client = function(ros,tf) {
    this.ros = ros;
    this.tf = tf;
    this.intMarkerHandles = {};
    this.clientId = "improxy.js " + Math.round( Math.random() * 1000000 );
  };

  Client.prototype.__proto__ = EventEmitter2.prototype;

  Client.prototype.subscribe = function(topicName) {
    this.unsubscribe();

    this.markerUpdateTopic = new this.ros.Topic({
      name : topicName + '/tunneled/update',
      messageType : 'visualization_msgs/InteractiveMarkerUpdate'
    });
    this.markerUpdateTopic.subscribe(this.processUpdate.bind(this));

    this.feedbackTopic = new this.ros.Topic({
      name : topicName + '/feedback',
      messageType : 'visualization_msgs/InteractiveMarkerFeedback'
    });
    this.feedbackTopic.advertise();

    this.initService = new this.ros.Service({
        name        : topicName + '/tunneled/get_init'
      , serviceType : 'demo_interactive_markers/GetInit'
    });
    var request = new this.ros.ServiceRequest({});
    this.initService.callService(request, this.processInit.bind(this));
  };

  Client.prototype.unsubscribe = function() {
    if ( this.markerUpdateTopic ) {
      this.markerUpdateTopic.unsubscribe();
      this.feedbackTopic.unadvertise();
    }
    for( intMarkerName in this.intMarkerHandles ) {
      this.eraseIntMarker(intMarkerName);
    }
    this.intMarkerHandles = {};
  }

  Client.prototype.eraseIntMarker = function(intMarkerName) {
    if ( this.intMarkerHandles[intMarkerName] ) {
      this.emit('deleted_marker', intMarkerName);
      delete this.intMarkerHandles[intMarkerName];
    }
  }

  Client.prototype.processInit = function(initMessage) {
    var message = initMessage.msg;
    var client = this;
    message.erases = [];
    for( intMarkerName in this.intMarkerHandles ) {
      message.erases.push( intMarkerName );
    };
    message.poses = [];
    this.processUpdate(message);
  }

  Client.prototype.processUpdate = function(message) {
    var that = this;

    // Deletes markers
    message.erases.forEach(function(name) {
      var marker = that.intMarkerHandles[name];
      that.eraseIntMarker(name);
    });

    // Updates marker poses
    message.poses.forEach(function(poseMessage) {
      var marker = that.intMarkerHandles[poseMessage.name];
      if (marker) {
        marker.setPoseFromServer(poseMessage.pose);
      }
    });

    // Adds new markers
    message.markers.forEach(function(imMsg) {
      var oldIntMarkerHandle = that.intMarkerHandles[imMsg.name];
      if (oldIntMarkerHandle) {
        that.eraseIntMarker(oldIntMarkerHandle.name);
      }

      var intMarkerHandle = new ImProxy.IntMarkerHandle(imMsg, that.feedbackTopic, that.tf);
      that.intMarkerHandles[imMsg.name] = intMarkerHandle;
      that.emit('created_marker', intMarkerHandle);
      // this might trigger a transform update event immediately,
      // so we need to call it after emitting a created_marker event.
      intMarkerHandle.subscribeTf(imMsg);
    });
  };

  /* Handle with signals for a single interactive marker */

  var IntMarkerHandle = ImProxy.IntMarkerHandle = function(imMsg, feedbackTopic, tf) {
    this.feedbackTopic  = feedbackTopic;
    this.tf             = tf;
    this.name           = imMsg.name;
    this.header         = imMsg.header;
    this.controls       = imMsg.controls;
    this.menuEntries    = imMsg.menu_entries;
    this.dragging       = false;
    this.timeoutHandle  = null;
    this.tfTransform    = new TfClient.Transform();
    this.pose           = new TfClient.Pose();
    this.setPoseFromServer( imMsg.pose );
  };

  IntMarkerHandle.prototype.__proto__ = EventEmitter2.prototype;

  var KEEP_ALIVE = 0;
  var POSE_UPDATE = 1;
  var MENU_SELECT = 2;
  var BUTTON_CLICK = 3;
  var MOUSE_DOWN = 4;
  var MOUSE_UP = 5;

  IntMarkerHandle.prototype.subscribeTf = function(imMsg) {
    // subscribe to tf updates if frame-fixed
    if ( imMsg.header.stamp.secs === 0.0 && imMsg.header.stamp.nsecs === 0.0 ) {
      this.tf.subscribe( imMsg.header.frame_id, this.tfUpdate.bind(this) );
    }
  }

  IntMarkerHandle.prototype.emitServerPoseUpdate = function() {
    var poseTransformed = new TfClient.Pose( this.pose.position, this.pose.orientation );
    this.tfTransform.apply( poseTransformed );
    this.emit('server_updated_pose', poseTransformed );
  }

  IntMarkerHandle.prototype.setPoseFromServer = function(poseMsg) {
    this.pose.copy( poseMsg );
    this.emitServerPoseUpdate();
  };

  IntMarkerHandle.prototype.tfUpdate = function(transformMsg) {
    this.tfTransform.copy( transformMsg );
    this.emitServerPoseUpdate();
  };

  IntMarkerHandle.prototype.setPoseFromClient = function(event) {
    this.pose.copy(event);
    this.tfTransform.applyInverse( this.pose );
    this.emit('client_updated_pose', event);
    this.sendFeedback(POSE_UPDATE, undefined, 0, event.controlName);

    // keep sending pose feedback until the mouse goes up
    if ( this.dragging ) {
      if ( this.timeoutHandle ) {
        clearTimeout(this.timeoutHandle);
      }
      this.timeoutHandle = setTimeout( this.setPoseFromClient.bind(this,event), 250 );
    }
  };

  IntMarkerHandle.prototype.onButtonClick = function(event) {
    this.sendFeedback(BUTTON_CLICK, event.clickPosition, 0, event.controlName);
  };

  IntMarkerHandle.prototype.onMouseDown = function(event) {
    this.sendFeedback(MOUSE_DOWN, event.clickPosition, 0, event.controlName);
    this.dragging = true;
  }

  IntMarkerHandle.prototype.onMouseUp = function(event) {
    this.sendFeedback(MOUSE_UP, event.clickPosition, 0, event.controlName);
    this.dragging = false;
    if ( this.timeoutHandle ) {
      clearTimeout(this.timeoutHandle);
    }
  }

  IntMarkerHandle.prototype.onMenuSelect = function(event) {
    this.sendFeedback(MENU_SELECT, undefined, event.id, event.controlName);
  }

  IntMarkerHandle.prototype.sendFeedback = function(eventType, clickPosition, menu_entry_id, controlName) {

    var mouse_point_valid = clickPosition !== undefined;
    var clickPosition = clickPosition || {
      x : 0,
      y : 0,
      z : 0
    };

    var feedback = {
      header       : this.header,
      client_id    : this.client_id,
      marker_name  : this.name,
      control_name : controlName,
      event_type   : eventType,
      pose         : this.pose,
      mouse_point  : clickPosition,
      mouse_point_valid: mouse_point_valid,
      menu_entry_id: menu_entry_id
    }
    this.feedbackTopic.publish(feedback);
  };

  return ImProxy;
}));
