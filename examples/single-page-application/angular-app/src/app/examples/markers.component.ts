import { Component, OnInit } from '@angular/core';
import * as ROS3D from 'ros3d';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-markers',
  template: `
    <div>
      <h1>Simple Marker Example</h1>
      <p>Run the following commands in the terminal then refresh this page.</p>
      <ol>
        <li><code>roscore</code></li>
        <li><code>rosrun visualization_marker_tutorials basic_shapes</code></li>
        <li><code>rosrun tf2_web_republisher tf2_web_republisher</code></li>
        <li><code>roslaunch rosbridge_server rosbridge_websocket.launch</code></li>
      </ol>
      <div id="markers"></div>
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
export class MarkersComponent implements OnInit {

  constructor() { }

  /**
   * Setup all visualization elements when the page is loaded.
   */
  ngOnInit() {
    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
      divID: 'markers',
      width: 400,
      height: 300,
      antialias: true
    });

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: '/my_frame'
    });

    // Setup the marker client.
    var markerClient = new ROS3D.MarkerClient({
      ros: ros,
      tfClient: tfClient,
      topic: '/visualization_marker',
      rootObject: viewer.scene
    });
  }
}
