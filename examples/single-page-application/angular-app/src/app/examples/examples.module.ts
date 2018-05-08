import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';

import { ExamplesRoutingModule } from './examples-routing.module';
import { MarkersComponent } from './markers.component';
import { PointCloud2Component } from './point-cloud-2.component';

@NgModule({
  imports: [
    CommonModule,
    ExamplesRoutingModule,
  ],
  declarations: [
    MarkersComponent,
    PointCloud2Component,
  ]
})
export class ExamplesModule { }
