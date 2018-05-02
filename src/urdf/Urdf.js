import THREE from '../../shims/three/core.js';
import * as ROSLIB from 'roslib';

import { makeColorMaterial } from '../Ros3D';
import { MeshResource } from '../models/MeshResource';
import { SceneNode } from '../visualization/SceneNode';

/**
 * @author Jihoon Lee - jihoonlee.in@gmail.com
 * @author Russell Toris - rctoris@wpi.edu
 */

export class Urdf extends THREE.Object3D {

  /**
   * A URDF can be used to load a ROSLIB.UrdfModel and its associated models into a 3D object.
   *
   * @constructor
   * @param options - object with following keys:
   *
   *   * urdfModel - the ROSLIB.UrdfModel to load
   *   * tfClient - the TF client handle to use
   *   * path (optional) - the base path to the associated Collada models that will be loaded
   *   * tfPrefix (optional) - the TF prefix to used for multi-robots
   *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER)
   */
  constructor(options) {
    options = options || {};
    var urdfModel = options.urdfModel;
    var path = options.path || '/';
    var tfClient = options.tfClient;
    var tfPrefix = options.tfPrefix || '';
    var loader = options.loader;

    super();

    // load all models
    var links = urdfModel.links;
    for ( var l in links) {
      var link = links[l];
      for( var i=0; i<link.visuals.length; i++ ) {
        var visual = link.visuals[i];
        if (visual && visual.geometry) {
          // Save frameID
          var frameID = tfPrefix + '/' + link.name;
          // Save color material
          var colorMaterial = null;
          if (visual.material && visual.material.color) {
            var color = visual.material && visual.material.color;
            colorMaterial = makeColorMaterial(color.r, color.g, color.b, color.a);
          }
          if (visual.geometry.type === ROSLIB.URDF_MESH) {
            var uri = visual.geometry.filename;
            // strips package://
            var tmpIndex = uri.indexOf('package://');
            if (tmpIndex !== -1) {
              uri = uri.substr(tmpIndex + ('package://').length);
            }
            var fileType = uri.substr(-4).toLowerCase();

            // ignore mesh files which are not in Collada or STL format
            if (fileType === '.dae' || fileType === '.stl') {
              // create the model
              var mesh = new MeshResource({
                path : path,
                resource : uri,
                loader : loader,
                material : colorMaterial
              });

              // check for a scale
              if(link.visuals[i].geometry.scale) {
                mesh.scale.copy(visual.geometry.scale);
              }

              // create a scene node with the model
              var sceneNode = new SceneNode({
                frameID : frameID,
                  pose : visual.origin,
                  tfClient : tfClient,
                  object : mesh
              });
              this.add(sceneNode);
            } else {
              console.warn('Could not load geometry mesh: '+uri);
            }
          } else {
            if (!colorMaterial) {
              colorMaterial = makeColorMaterial(0, 0, 0, 1);
            }
            var shapeMesh;
            // Create a shape
            switch (visual.geometry.type) {
              case ROSLIB.URDF_BOX:
                var dimension = visual.geometry.dimension;
                var cube = new THREE.BoxGeometry(dimension.x, dimension.y, dimension.z);
                shapeMesh = new THREE.Mesh(cube, colorMaterial);
                break;
              case ROSLIB.URDF_CYLINDER:
                var radius = visual.geometry.radius;
                var length = visual.geometry.length;
                var cylinder = new THREE.CylinderGeometry(radius, radius, length, 16, 1, false);
                shapeMesh = new THREE.Mesh(cylinder, colorMaterial);
                shapeMesh.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * 0.5);
                break;
              case ROSLIB.URDF_SPHERE:
                var sphere = new THREE.SphereGeometry(visual.geometry.radius, 16);
                shapeMesh = new THREE.Mesh(sphere, colorMaterial);
                break;
            }
            // Create a scene node with the shape
            var scene = new SceneNode({
              frameID: frameID,
                pose: visual.origin,
                tfClient: tfClient,
                object: shapeMesh
            });
            this.add(scene);
          }
        }
      }
    }
  };

  unsubscribeTf () {
    this.children.forEach(function(n) {
      if (typeof n.unsubscribeTf === 'function') { n.unsubscribeTf(); }
    });
  };
}
