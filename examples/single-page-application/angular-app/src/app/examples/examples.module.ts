import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';

import { ExamplesRoutingModule } from './examples-routing.module';
import { MarkersComponent } from './markers.component';

@NgModule({
  imports: [
    CommonModule,
    ExamplesRoutingModule
  ],
  declarations: [MarkersComponent]
})
export class ExamplesModule { }
