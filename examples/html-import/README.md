# Examples of ROS3D as an ECMAScript (ES) module in a static webpage

> Providing `type="module"` on a `script` element tells HTML5-compliant browsers to treat the script as an ECMAScript module. A second fallback script can be provided to older browsers by providing the `nomodule` attribute

> These examples must be run by a webserver as HTML imports can only load resources from the same (non-file) domain or domains that support CORS. (see [this issue](https://github.com/Polymer/polymer/issues/1535) for more info)

## Build Setup

``` bash
# 1. install dependencies
yarn install # or npm install

# 2. start simple express webserver at localhost:3000
yarn start # or npm start

# 3. open app and view examples
# http://localhost:3000
```
