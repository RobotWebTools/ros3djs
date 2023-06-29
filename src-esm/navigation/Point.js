import THREE from '../../shims/three/core.js';
import * as ROSLIB from 'roslib';

import { SceneNode } from '../visualization/SceneNode'

/**
 * @fileOverview
 * @author David V. Lu!! - davidvlu@gmail.com
 */

export class Point extends THREE.Object3D {

  /**
   * A PointStamped client
   *
   * @constructor
   * @param options - object with following keys:
   *
   *  * ros - the ROSLIB.Ros connection handle
   *  * topic - the marker topic to listen to
   *  * tfClient - the TF client handle to use
   *  * rootObject (optional) - the root object to add this marker to
   *  * color (optional) - color for line (default: 0xcc00ff)
   *  * radius (optional) - radius of the point (default: 0.2)
   */
  constructor(options) {
    super();
    this.options = options || {};
    this.ros = options.ros;
    this.topicName = options.topic || '/point';
    this.tfClient = options.tfClient;
    this.color = options.color || 0xcc00ff;
    this.rootObject = options.rootObject || new THREE.Object3D();
    this.radius = options.radius || 0.2;

    this.sn = null;

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
        messageType : 'geometry_msgs/PointStamped'
    });
    this.rosTopic.subscribe(this.processMessage.bind(this));
  };

  processMessage(message){
    if(this.sn!==null){
        this.sn.unsubscribeTf();
        this.rootObject.remove(this.sn);
    }

    var sphereGeometry = new THREE.SphereGeometry( this.radius );
    var sphereMaterial = new THREE.MeshBasicMaterial( {color: this.color} );
    var sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    sphere.position.set(message.point.x, message.point.y, message.point.z);

    this.sn = new SceneNode({
        frameID : message.header.frame_id,
        tfClient : this.tfClient,
        object : sphere
    });

    this.rootObject.add(this.sn);
  };
}
