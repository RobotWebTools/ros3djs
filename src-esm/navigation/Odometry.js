import THREE from '../../shims/three/core.js';
import * as ROSLIB from 'roslib';

import { Arrow } from '../models/Arrow'
import { SceneNode } from '../visualization/SceneNode'

/**
 * @fileOverview
 * @author David V. Lu!! - davidvlu@gmail.com
 */

export class Odometry extends THREE.Object3D {

  /**
   * An Odometry client
   *
   * @constructor
   * @param options - object with following keys:
   *
   *  * ros - the ROSLIB.Ros connection handle
   *  * topic - the marker topic to listen to
   *  * tfClient - the TF client handle to use
   *  * rootObject (optional) - the root object to add this marker to
   *  * keep (optional) - number of markers to keep around (default: 1)
   *  * color (optional) - color for line (default: 0xcc00ff)
   *  * length (optional) - the length of the arrow (default: 1.0)
   *  * headLength (optional) - the head length of the arrow (default: 0.2)
   *  * shaftDiameter (optional) - the shaft diameter of the arrow (default: 0.05)
   *  * headDiameter (optional) - the head diameter of the arrow (default: 0.1)
   */
  constructor(options) {
    super();
    this.options = options || {};
    this.ros = options.ros;
    this.topicName = options.topic || '/particlecloud';
    this.tfClient = options.tfClient;
    this.color = options.color || 0xcc00ff;
    this.length = options.length || 1.0;
    this.rootObject = options.rootObject || new THREE.Object3D();
    this.keep = options.keep || 1;

    this.sns = [];

    this.rosTopic = undefined;
    this.subscribe();
  };


  unsubscribe(){
    if(this.rosTopic){
      this.rosTopic.unsubscribe(this.processMessage);
    }
  };

  subscribe(){
    this.unsubscribe();

    // subscribe to the topic
    this.rosTopic = new ROSLIB.Topic({
      ros : this.ros,
      name : this.topicName,
      queue_length : 1,
      messageType : 'nav_msgs/Odometry'
    });
    this.rosTopic.subscribe(this.processMessage.bind(this));
  };

  processMessage(message){
    if(this.sns.length >= this.keep) {
        this.sns[0].unsubscribeTf();
        this.rootObject.remove(this.sns[0]);
        this.sns.shift();
    }

    this.options.origin = new THREE.Vector3( message.pose.pose.position.x, message.pose.pose.position.y,
                                             message.pose.pose.position.z);

    var rot = new THREE.Quaternion(message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                                   message.pose.pose.orientation.z, message.pose.pose.orientation.w);
    this.options.direction = new THREE.Vector3(1,0,0);
    this.options.direction.applyQuaternion(rot);
    this.options.material = new THREE.MeshBasicMaterial({color: this.color});
    var arrow = new Arrow(this.options);

    this.sns.push(new SceneNode({
      frameID : message.header.frame_id,
      tfClient : this.tfClient,
      object : arrow
    }));

    this.rootObject.add(this.sns[ this.sns.length - 1]);
  };
}
