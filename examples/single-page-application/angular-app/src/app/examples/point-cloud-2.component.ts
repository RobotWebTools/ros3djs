import { Component, OnInit } from '@angular/core';
import * as ROS3D from 'ros3d';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-point-cloud-2',
  template: `
    <div>
      <h1>Simple PointCloud2 Example</h1>
      <p>Run the following commands in the terminal then refresh the page.</p>
      <ol>
        <li><code>roscore</code></li>
        <li><code>roslaunch rosbridge_server rosbridge_websocket.launch</code></li>
        <li><code>rosrun tf2_web_republisher tf2_web_republisher</code></li>
        <li><code>roslaunch openni_launch openni.launch depth_registration:=true</code></li>
      </ol>
      <div id="viewer"></div>
    </div>
  `,
  styles: [`
    h1, h2 {
      margin-top: 0;
      font-weight: normal;
    }
    ul {
      list-style-type: none;
    }
  `]
})
export class PointCloud2Component implements OnInit {

  constructor() { }

  /**
   * Setup all visualization elements when the page is loaded.
   */
  ngOnInit() {
    // Connect to ROS.
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    // Create the main viewer.
    const viewer = new ROS3D.Viewer({
      divID: 'viewer',
      width: 800,
      height: 600,
      antialias: true
    });

    // Setup a client to listen to TFs.
    const tfClient = new ROSLIB.TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: '/camera_link'
    });

    const cloudClient = new ROS3D.PointCloud2({
      ros: ros,
      tfClient: tfClient,
      rootObject: viewer.scene,
      topic: '/camera/depth_registered/points',
      material: { size: 0.05, color: 0xff00ff }
    });
  }
}
