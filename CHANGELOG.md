2015-03-27 - **0.15.0**
 * Fix never-set orientation of Interactive Markers [(rethink-rlinsalata)](https://github.com/rethink-rlinsalata/)
 * Properly implement MARKER_TEXT_VIEW_FACING [(T045T)](https://github.com/T045T/)

2015-03-09 - **0.14.0**
 * Make distance to near and far plane configurable [(daniel86)](https://github.com/daniel86/)
 * Use phong shading instead of lambert shading [(daniel86)](https://github.com/daniel86/)
 * Fix rotation of SceneNodes [(jakobs)](https://github.com/jakobs/)
 * Fixed STL loader [(jakobs)](https://github.com/jakobs/)
 * Added support for multiple visuals per link [(jakobs)](https://github.com/jakobs/)
 * Removes package:// from URI [(rctoris)](https://github.com/rctoris/)
 * Only adds material to a model that has none [(rctoris)](https://github.com/rctoris/)

2015-02-13 - **0.13.0**
 * Fixed memory leak of mesh marker [(asisbot)](https://github.com/asisbot/)
 * Support for different marker actions [(daniel86)](https://github.com/daniel86/)

 2015-02-04 - **0.12.0**
 * Apply touch event to interactive menu marker [(asisbot)](https://github.com/asisbot/)
 * Antialias fix [(daniel86)](https://github.com/daniel86/)
 * Updated depthcloud example to use web_video_server [(mitchellwills)](https://github.com/mitchellwills/)

2014-12-08 - **0.11.0**
 * Change linePrecision on raycaster [(mszarski)](https://github.com/mszarski/)
 * Added support for loading STLs as meshes [(mitchellwills)](https://github.com/mitchellwills/)
 * Compute normals after loading an STL model [(T045T)](https://github.com/T045T/)
 * Fixed depth cloud example [(mitchellwills)](https://github.com/mitchellwills/)
 * Fixed video method name [Akin Sisbot]
 * Fixed multi-touch operation for both Windows and Android tablet [Akin Sisbot]
 * Fix for ROS3D.Viewer clear color [Daniel Beßler]

2014-09-09 - **0.10.0**
 * Updated threejs library to r61 [(mitchellwills)](https://github.com/mitchellwills/)

2014-07-07 - **0.9.0**
 * Prevents empty text from being rendered [(rctoris)](https://github.com/rctoris/)
 * Added MarkerArrayClient [(T045T)](https://github.com/T045T/)
 * Use namespace for Markers - ID alone is not supposed to be unique [(T045T)](https://github.com/T045T/)
 * Use a separate TFClient to get the transform from InteractiveMarker to its sub-Markers [(T045T)](https://github.com/T045T/)
 * Interactivemarkers: get the correct transformation for sub-markers [(T045T)](https://github.com/T045T/)
 * Enable markdown parsing in JSDoc and add newlines to ensure proper parsing of lists [(T045T)](https://github.com/T045T/)

2014-06-11 - **0.8.0**
 * Added flag to change the type of collada loader [(rctoris)](https://github.com/rctoris/)
 * Fix typo in description of onMouseMove() [(T045T)](https://github.com/T045T/)
 * Use the total length of zoomDelta, not just the y component, for multi-touch zooming [(T045T)](https://github.com/T045T/)
 * Correction on measure unit for cellSize [(rbonghi)](https://github.com/rbonghi/)
 * Urdf shapes fix [(rbonghi)](https://github.com/rbonghi/)
 * Allow to change cellSize [(rk4an)](https://github.com/rk4an/)
 * Added LINE_STRIP and LINE_LIST support [(mszarski)](https://github.com/mszarski/)
 * Add tfPrefix for multi-robots support [(erkan)](https://github.com/erkan/)
 * Remove memory leak from MarkerClient [(knowrob)](https://github.com/knowrob/)
 * Changed MarkerClient to support multiple markers at a time [(knowrob)](https://github.com/knowrob/)

2013-08-23 - **r7**
 * Improved tablet support [(KazutoMurase)](https://github.com/KazutoMurase/)
 * Fix to cylinder marker type [(KazutoMurase)](https://github.com/KazutoMurase/)
 * Fix to IM transforms [(eratner)](https://github.com/eratner/)
 * Fix to colored mesh resources [(eratner)](https://github.com/eratner/)

2013-06-06 - **r6**
 * MARKER_CUBE_LIST added to Marker.js [(rctoris)](https://github.com/rctoris/)
 * URDF meshes are now scaled correctly [(rctoris)](https://github.com/rctoris/)

2013-05-29 - **r5**
 * Added DepthCloud class for point ploud streaming with ros_web_video and depthcloud_encoder [(jkammerl)](https://github.com/jkammerl/), [(dgossow)](https://github.com/dgossow/)

2013-04-15 - **r4**
 * Initial pose now set in SceneNode [(rctoris)](https://github.com/rctoris/)
 * Texture method for OccupancyGrid changed to canvas to allow for very large maps [(rctoris)](https://github.com/rctoris/)
 * OccupancyGrid origin now in corner instead of center of image [(rctoris)](https://github.com/rctoris/)
 * Optional TF client added to occupancy grid client [(rctoris)](https://github.com/rctoris/)
 * Interactive markers now removed correctly [(rctoris)](https://github.com/rctoris/)
 * Viewer now takes an optional initial camera position [(rctoris)](https://github.com/rctoris/)
 * Orbit controls fixed to prevent flicker during camera pan [(rctoris)](https://github.com/rctoris/)
 * Scale (units) now work in Collada files [(rctoris)](https://github.com/rctoris/)
 * Grunt files added and code cleanup for linter [(rctoris)](https://github.com/rctoris/)

2013-04-03 - **r3**
 * ColladaLoader2 added as third-party module [(rctoris)](https://github.com/rctoris/)
 * ColladaLoader2 replaces THREE.ColladaLoader [(rctoris)](https://github.com/rctoris/)
 * Viewer now takes an option intensity for the lighting [(rctoris)](https://github.com/rctoris/)

2013-04-03 - **r2**
 * Examples now use CDN version of DAE files [(rctoris)](https://github.com/rctoris/)
 * Maps module added with OccupancyGrid and client [(rctoris)](https://github.com/rctoris/)
 * SceneNode now takes an optional pose [(rctoris)](https://github.com/rctoris/)
 * Bugfix in InteractiveMarkerClient path option [(rctoris)](https://github.com/rctoris/)
 * Bugfix in interactive marker feedback [(rctoris)](https://github.com/rctoris/)

2013-03-17 - **r1**
 * Initial development of ROS3D [(rctoris)](https://github.com/rctoris/)
