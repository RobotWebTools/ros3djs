import { Component } from '@angular/core';
import * as ROS3D from 'ros3d';
import * as ROSLIB from 'roslib';

@Component({
  selector: 'app-root',
  template: `
    <div id="app">
      <p>This is an app meant to demonstrate how to use the ros3djs module in an Angular App</p>
      <p>Examples:</p>
      <ul>
        <li><a routerLink="examples/markers">Markers</a></li>
        <li><a routerLink="examples/point-cloud-2">PointCloud2</a></li>
      </ul>

      <div id="examples">
        <router-outlet></router-outlet>
      </div>
    </div>
  `,
  styles: [`
    #app {
      font-family: 'Avenir', Helvetica, Arial, sans-serif;
      -webkit-font-smoothing: antialiased;
      -moz-osx-font-smoothing: grayscale;
      text-align: left;
      color: #2c3e50;
      margin: 1em;
    }

    li {
      list-style-type: none;
    }

    #examples {
      box-shadow:
        rgba(10, 16, 20, 0.24) 0px 2px 2px,
        rgba(10, 16, 20, 0.12) 0px 0px 2px;
      padding: 1em;
    }
  `]
})
export class AppComponent {
}
