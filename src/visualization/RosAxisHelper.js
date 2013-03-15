/**
 * @author dgossow@willowgarage.com
 */
(function (root, factory) {
  if (typeof define === 'function' && define.amd) {
    define(['three'], factory);
  }
  else {
    root.RosAxisHelper = factory(root.THREE);
  }
}(this, function (THREE) {

  //THREE.Axes = function ( options ) 
  var Axes = function ( options ) 
  {
    THREE.Object3D.call( this );
    
    var that = this;
    
    options = options || {};
    var shaftRadius = options.shaftRadius !== undefined ? options.shaftRadius : 0.008;
    var headRadius = options.headRadius !== undefined ? options.headRadius : 0.023;
    var headLength = options.headLength !== undefined ? options.headLength : 0.1;
    
    var longAxes = options.longAxes;

    this.line_geom = new THREE.CylinderGeometry( shaftRadius, shaftRadius, 1.0 - headLength );
    this.head_geom = new THREE.CylinderGeometry( 0, headRadius, headLength );

    function addAxis( axis )
    {
      var color = new THREE.Color;
      color.setRGB( axis.x, axis.y, axis.z );
      var material = new THREE.MeshBasicMaterial( { color: color.getHex() } );
      
      var rot_axis = new THREE.Vector3;
      rot_axis.crossVectors( axis, new THREE.Vector3( 0, -1, 0 ) );

      var rot = new THREE.Quaternion;
      rot.setFromAxisAngle( rot_axis, 0.5*Math.PI );

      var arrow = new THREE.Mesh( that.head_geom, material );
      arrow.position = axis.clone();
      arrow.position.multiplyScalar( 0.95 );
      arrow.useQuaternion = true;
      arrow.quaternion = rot;
      arrow.updateMatrix();
      that.add( arrow );

      var line = new THREE.Mesh( that.line_geom, material );
      line.position = axis.clone();
      line.position.multiplyScalar( 0.45 );
      line.useQuaternion = true;
      line.quaternion = rot;
      line.updateMatrix();
      that.add( line );        
    }
    
    addAxis( new THREE.Vector3(1,0,0) );
    addAxis( new THREE.Vector3(0,1,0) );
    addAxis( new THREE.Vector3(0,0,1) );
  };

  Axes.prototype = Object.create( THREE.Object3D.prototype );

  return Axes;
}));
