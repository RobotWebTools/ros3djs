import THREE from '../shims/three/core.js';

import { makeColorMaterial, MARKER_ARROW, MARKER_CUBE, MARKER_SPHERE, MARKER_CYLINDER, MARKER_LINE_STRIP, MARKER_LINE_LIST, MARKER_CUBE_LIST, MARKER_SPHERE_LIST, MARKER_POINTS, MARKER_TEXT_VIEW_FACING, MARKER_MESH_RESOURCE, MARKER_TRIANGLE_LIST } from '../Ros3D.js';
import { Arrow } from '../models/Arrow.js';
import { MeshResource } from '../models/MeshResource.js';
import { TriangleList } from '../models/TriangleList.js';

/**
 * @author David Gossow - dgossow@willowgarage.com
 * @author Russell Toris - rctoris@wpi.edu
 */

export class Marker extends THREE.Object3D {

  /**
   * A Marker can convert a ROS marker message into a THREE object.
   *
   * @constructor
   * @param options - object with following keys:
   *
   *   * path - the base path or URL for any mesh files that will be loaded for this marker
   *   * message - the marker message
   */
  constructor(options) {
    super();

    options = options || {};
    var path = options.path || '/';
    var message = options.message;

    // check for a trailing '/'
    if (path.substr(path.length - 1) !== '/') {
      path += '/';
    }

    if(message.scale) {
      this.msgScale = [message.scale.x, message.scale.y, message.scale.z];
    }
    else {
      this.msgScale = [1,1,1];
    }
    this.msgColor = message.color;
    this.msgMesh = undefined;

    // set the pose and get the color
    this.setPose(message.pose);
    var colorMaterial = makeColorMaterial(this.msgColor.r,
        this.msgColor.g, this.msgColor.b, this.msgColor.a);

    // create the object based on the type
    switch (message.type) {
      case MARKER_ARROW:
        // get the sizes for the arrow
        var len = message.scale.x;
        var headLength = len * 0.23;
        var headDiameter = message.scale.y;
        var shaftDiameter = headDiameter * 0.5;

        // determine the points
        var direction, p1 = null;
        if (message.points.length === 2) {
          p1 = new THREE.Vector3(message.points[0].x, message.points[0].y, message.points[0].z);
          var p2 = new THREE.Vector3(message.points[1].x, message.points[1].y, message.points[1].z);
          direction = p1.clone().negate().add(p2);
          // direction = p2 - p1;
          len = direction.length();
          headDiameter = message.scale.y;
          shaftDiameter = message.scale.x;

          if (message.scale.z !== 0.0) {
            headLength = message.scale.z;
          }
        }

        // add the marker
        this.add(new Arrow({
          direction : direction,
          origin : p1,
          length : len,
          headLength : headLength,
          shaftDiameter : shaftDiameter,
          headDiameter : headDiameter,
          material : colorMaterial
        }));
        break;
      case MARKER_CUBE:
        // set the cube dimensions
        var cubeGeom = new THREE.BoxGeometry(message.scale.x, message.scale.y, message.scale.z);
        this.add(new THREE.Mesh(cubeGeom, colorMaterial));
        break;
      case MARKER_SPHERE:
        // set the sphere dimensions
        var sphereGeom = new THREE.SphereGeometry(0.5);
        var sphereMesh = new THREE.Mesh(sphereGeom, colorMaterial);
        sphereMesh.scale.x = message.scale.x;
        sphereMesh.scale.y = message.scale.y;
        sphereMesh.scale.z = message.scale.z;
        this.add(sphereMesh);
        break;
      case MARKER_CYLINDER:
        // set the cylinder dimensions
        var cylinderGeom = new THREE.CylinderGeometry(0.5, 0.5, 1, 16, 1, false);
        var cylinderMesh = new THREE.Mesh(cylinderGeom, colorMaterial);
        cylinderMesh.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * 0.5);
        cylinderMesh.scale.set(message.scale.x, message.scale.z, message.scale.y);
        this.add(cylinderMesh);
        break;
      case MARKER_LINE_STRIP:
        var lineStripGeom = new THREE.BufferGeometry()
        var vertices = []
        var colors = []
        var lineStripMaterial = new THREE.LineBasicMaterial({
          size : message.scale.x
        });

        // add the points
        var j;
        for ( j = 0; j < message.points.length; j++) {
          vertices.push(message.points[j].x, message.points[j].y, message.points[j].z);
        }

        lineStripGeom.setAttribute( 'position', new THREE.Float32BufferAttribute( vertices, 3 ) );

        // determine the colors for each
        if (message.colors.length === message.points.length) {
          lineStripMaterial.vertexColors = true;
          for ( j = 0; j < message.points.length; j++) {
            colors.push(message.colors[j].r, message.colors[j].g, message.colors[j].b);
          }

          lineStripGeom.setAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );
        } else {
          lineStripMaterial.color.setRGB(message.color.r, message.color.g, message.color.b);
        }

        // add the line
        this.add(new THREE.Line(lineStripGeom, lineStripMaterial));
        break;
      case MARKER_LINE_LIST:
        var lineListGeom = new THREE.BufferGeometry()
        var vertices = []
        var colors = []
        var lineListMaterial = new THREE.LineBasicMaterial({
          size : message.scale.x
        });

        // add the points
        var k;
        for ( k = 0; k < message.points.length; k++) {
          vertices.push(message.points[k].x, message.points[k].y, message.points[k].z);
        }

        lineListGeom.setAttribute( 'position', new THREE.Float32BufferAttribute( vertices, 3 ) );

        // determine the colors for each
        if (message.colors.length === message.points.length) {
          lineListMaterial.vertexColors = true;
          for ( k = 0; k < message.points.length; k++) {
            colors.push(message.colors[k].r, message.colors[k].g, message.colors[k].b);
          }

          lineListGeom.setAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );
        } else {
          lineListMaterial.color.setRGB(message.color.r, message.color.g, message.color.b);
        }

        // add the line
        this.add(new THREE.Line(lineListGeom, lineListMaterial,THREE.LinePieces));
        break;
      case MARKER_CUBE_LIST:
        // holds the main object
        var object = new THREE.Object3D();

        // check if custom colors should be used
        var numPoints = message.points.length;
        var createColors = (numPoints === message.colors.length);
        // do not render giant lists
        var stepSize = Math.ceil(numPoints / 1250);

        // add the points
        var p, cube, curColor, newMesh;
        for (p = 0; p < numPoints; p+=stepSize) {
          cube = new THREE.BoxGeometry(message.scale.x, message.scale.y, message.scale.z);

          // check the color
          if(createColors) {
            curColor = makeColorMaterial(message.colors[p].r, message.colors[p].g, message.colors[p].b, message.colors[p].a);
          } else {
            curColor = colorMaterial;
          }

          newMesh = new THREE.Mesh(cube, curColor);
          newMesh.position.x = message.points[p].x;
          newMesh.position.y = message.points[p].y;
          newMesh.position.z = message.points[p].z;
          object.add(newMesh);
        }

        this.add(object);
        break;
      case MARKER_SPHERE_LIST:
        // holds the main object
        var sphereObject = new THREE.Object3D();

        // check if custom colors should be used
        var numSpherePoints = message.points.length;
        var createSphereColors = (numSpherePoints === message.colors.length);
        // do not render giant lists
        var sphereStepSize = Math.ceil(numSpherePoints / 1250);

        // add the points
        var q, sphere, curSphereColor, newSphereMesh;
        for (q = 0; q < numSpherePoints; q+=sphereStepSize) {
          sphere = new THREE.SphereGeometry(0.5, 8, 8);

          // check the color
          if(createSphereColors) {
            curSphereColor = makeColorMaterial(message.colors[q].r, message.colors[q].g, message.colors[q].b, message.colors[q].a);
          } else {
            curSphereColor = colorMaterial;
          }

          newSphereMesh = new THREE.Mesh(sphere, curSphereColor);
          newSphereMesh.scale.x = message.scale.x;
          newSphereMesh.scale.y = message.scale.y;
          newSphereMesh.scale.z = message.scale.z;
          newSphereMesh.position.x = message.points[q].x;
          newSphereMesh.position.y = message.points[q].y;
          newSphereMesh.position.z = message.points[q].z;
          sphereObject.add(newSphereMesh);
        }
        this.add(sphereObject);
        break;
      case MARKER_POINTS:
        // for now, use a particle system for the lists
        
        var geometry = new THREE.BufferGeometry()
        var vertices = []
        var colors = []
        var material = new THREE.ParticleBasicMaterial({
          size : message.scale.x
        });

        // add the points
        var i;
        for ( i = 0; i < message.points.length; i++) {
          vertices.push(message.points[i].x, message.points[i].y, message.points[i].z);
        }

        geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( vertices, 3 ) );

        // determine the colors for each
        if (message.colors.length === message.points.length) {
          material.vertexColors = true;
          for ( i = 0; i < message.points.length; i++) {
            colors.push(message.colors[i].r, message.colors[i].g, message.colors[i].b);
          }

          geometry.setAttribute( 'color', new THREE.Float32BufferAttribute( colors, 3 ) );

        } else {
          material.color.setRGB(message.color.r, message.color.g, message.color.b);
        }

        

        // add the particle system
        this.add(new THREE.ParticleSystem(geometry, material));
        break;
      case MARKER_TEXT_VIEW_FACING:
        // only work on non-empty text
        if (message.text.length > 0) {
          // Use a THREE.Sprite to always be view-facing
          // ( code from http://stackoverflow.com/a/27348780 )
          var textColor = this.msgColor;

          var canvas = document.createElement('canvas');
          var context = canvas.getContext('2d');
          var textHeight = 100;
          var fontString = 'normal ' + textHeight + 'px sans-serif';
          context.font = fontString;
          var metrics = context.measureText( message.text );
          var textWidth = metrics.width;

          canvas.width = textWidth;
          // To account for overhang (like the letter 'g'), make the canvas bigger
          // The non-text portion is transparent anyway
          canvas.height = 1.5 * textHeight;

          // this does need to be set again
          context.font = fontString;
          context.fillStyle = 'rgba('
            + Math.round(255 * textColor.r) + ', '
            + Math.round(255 * textColor.g) + ', '
            + Math.round(255 * textColor.b) + ', '
            + textColor.a + ')';
          context.textAlign = 'left';
          context.textBaseline = 'middle';
          context.fillText( message.text, 0, canvas.height/2);

          var texture = new THREE.Texture(canvas);
          texture.needsUpdate = true;

          var spriteMaterial = new THREE.SpriteMaterial({
            map: texture,
            // NOTE: This is needed for THREE.js r61, unused in r70
            useScreenCoordinates: false });
          var sprite = new THREE.Sprite( spriteMaterial );
          var textSize = message.scale.x;
          sprite.scale.set(textWidth / canvas.height * textSize, textSize, 1);

          this.add(sprite);      }
        break;
      case MARKER_MESH_RESOURCE:
        // load and add the mesh
        var meshColorMaterial = null;
        if(message.color.r !== 0 || message.color.g !== 0 ||
           message.color.b !== 0 || message.color.a !== 0) {
          meshColorMaterial = colorMaterial;
        }
        this.msgMesh = message.mesh_resource.substr(10);
        var meshResource = new MeshResource({
          path : path,
          resource :  this.msgMesh,
          material : meshColorMaterial,
        });
        this.add(meshResource);
        break;
      case MARKER_TRIANGLE_LIST:
        // create the list of triangles
        var tri = new TriangleList({
          material : colorMaterial,
          vertices : message.points,
          colors : message.colors
        });
        tri.scale.set(message.scale.x, message.scale.y, message.scale.z);
        this.add(tri);
        break;
      default:
        console.error('Currently unsupported marker type: ' + message.type);
        break;
    }
  };

  /**
   * Set the pose of this marker to the given values.
   *
   * @param pose - the pose to set for this marker
   */
  setPose(pose) {
    // set position information
    this.position.x = pose.position.x;
    this.position.y = pose.position.y;
    this.position.z = pose.position.z;

    // set the rotation
    this.quaternion.set(pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w);
    this.quaternion.normalize();

    // update the world
    this.updateMatrixWorld();
  };

  /**
   * Update this marker.
   *
   * @param message - the marker message
   * @return true on success otherwhise false is returned
   */
  update(message) {
    // set the pose and get the color
    this.setPose(message.pose);

    // Update color
    if(message.color.r !== this.msgColor.r ||
       message.color.g !== this.msgColor.g ||
       message.color.b !== this.msgColor.b ||
       message.color.a !== this.msgColor.a)
    {
        var colorMaterial = makeColorMaterial(
            message.color.r, message.color.g,
            message.color.b, message.color.a);

        switch (message.type) {
        case MARKER_LINE_STRIP:
        case MARKER_LINE_LIST:
        case MARKER_POINTS:
            break;
        case MARKER_ARROW:
        case MARKER_CUBE:
        case MARKER_SPHERE:
        case MARKER_CYLINDER:
        case MARKER_TRIANGLE_LIST:
        case MARKER_TEXT_VIEW_FACING:
            this.traverse (function (child){
                if (child instanceof THREE.Mesh) {
                    child.material = colorMaterial;
                }
            });
            break;
        case MARKER_MESH_RESOURCE:
            var meshColorMaterial = null;
            if(message.color.r !== 0 || message.color.g !== 0 ||
               message.color.b !== 0 || message.color.a !== 0) {
                meshColorMaterial = this.colorMaterial;
            }
            this.traverse (function (child){
                if (child instanceof THREE.Mesh) {
                    child.material = meshColorMaterial;
                }
            });
            break;
        case MARKER_CUBE_LIST:
        case MARKER_SPHERE_LIST:
            // TODO Support to update color for MARKER_CUBE_LIST & MARKER_SPHERE_LIST
            return false;
        default:
            return false;
        }

        this.msgColor = message.color;
    }

    // Update geometry
    var scaleChanged =
          Math.abs(this.msgScale[0] - message.scale.x) > 1.0e-6 ||
          Math.abs(this.msgScale[1] - message.scale.y) > 1.0e-6 ||
          Math.abs(this.msgScale[2] - message.scale.z) > 1.0e-6;
    this.msgScale = [message.scale.x, message.scale.y, message.scale.z];

    switch (message.type) {
      case MARKER_CUBE:
      case MARKER_SPHERE:
      case MARKER_CYLINDER:
          if(scaleChanged) {
              return false;
          }
          break;
      case MARKER_TEXT_VIEW_FACING:
          if(scaleChanged || this.text !== message.text) {
              return false;
          }
          break;
      case MARKER_MESH_RESOURCE:
          var meshResource = message.mesh_resource.substr(10);
          if(meshResource !== this.msgMesh) {
              return false;
          }
          if(scaleChanged) {
              return false;
          }
          break;
      case MARKER_ARROW:
      case MARKER_LINE_STRIP:
      case MARKER_LINE_LIST:
      case MARKER_CUBE_LIST:
      case MARKER_SPHERE_LIST:
      case MARKER_POINTS:
      case MARKER_TRIANGLE_LIST:
          // TODO: Check if geometry changed
          return false;
      default:
          break;
    }

    return true;
  };

  /*
   * Free memory of elements in this marker.
   */
  dispose() {
    this.children.forEach(function(element) {
      if (element instanceof MeshResource) {
        element.children.forEach(function(scene) {
          if (scene.material !== undefined) {
            scene.material.dispose();
          }
          scene.children.forEach(function(mesh) {
            if (mesh.geometry !== undefined) {
              mesh.geometry.dispose();
            }
            if (mesh.material !== undefined) {
              mesh.material.dispose();
            }
            scene.remove(mesh);
          });
          element.remove(scene);
        });
      } else {
        if (element.geometry !== undefined) {
            element.geometry.dispose();
        }
        if (element.material !== undefined) {
            element.material.dispose();
        }
      }
      element.parent.remove(element);
    });
  };
}
