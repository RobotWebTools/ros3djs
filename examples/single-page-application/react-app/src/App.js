import React, { Component } from 'react';
import { BrowserRouter as Router, Route, Link } from 'react-router-dom'

import './App.css';
import Markers from './examples/markers';

class App extends Component {
  render() {
    return (
      <Router>
        <div className="App">
          <p>This is an app meant to demonstrate how to use the ros3djs module in a Single Page Application</p>
          <p>Examples:</p>
          <ul>
            <li><Link to="/examples/markers">Markers</Link></li>
          </ul>

          <div id="examples">
              <div>
                <Route exact path="/examples/markers" component={Markers} />
              </div>
          </div>
        </div>
      </Router>
    );
  }
}

export default App;
