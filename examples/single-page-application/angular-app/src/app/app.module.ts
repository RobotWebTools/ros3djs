import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';

import { AppRoutingModule } from './app-routing.module';

import { AppComponent } from './app.component';
import { ExamplesModule } from './examples/examples.module';


@NgModule({
  declarations: [
    AppComponent
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    ExamplesModule
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
