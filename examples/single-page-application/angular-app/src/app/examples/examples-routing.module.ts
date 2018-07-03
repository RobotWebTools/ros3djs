import { NgModule } from '@angular/core';
import { Routes, RouterModule } from '@angular/router';

import { MarkersComponent } from './markers.component';
import { PointCloud2Component } from './point-cloud-2.component';

const routes: Routes = [{
  path: 'examples',
  children: [
    { path: 'markers', component: MarkersComponent },
    { path: 'point-cloud-2', component: PointCloud2Component },
  ]
}];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule]
})
export class ExamplesRoutingModule { }
