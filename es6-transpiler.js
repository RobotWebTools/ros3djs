const debug = require('debug')
const path = require('path')

const colors = {
  GRAY: 0,
  RED: 1,
  GREEN: 2,
  YELLOW: 3,
  BLUE: 4,
  PURPLE: 5,
  CYAN: 6,
  WHITE: 7,
}

const stringifyObjects = (...args) => args
  .map(arg => typeof arg === 'object'
    ? JSON.stringify(arg, null, 2)
    : arg)

const logError = debug('ES6Transpiler:Error')
logError.log = (...args) => console.error(...stringifyObjects(...args))
logError.color = colors.RED

const logWarning = debug('ES6Transpiler:Warning')
logWarning.log = (...args) => console.log(...stringifyObjects(...args))
logWarning.color = colors.YELLOW

const logInfo = debug('ES6Transpiler:Info')
logInfo.log = (...args) => console.log(...stringifyObjects(...args))
logInfo.color = colors.BLUE

const getColorCodes = (c, bold) => {
  const boldCode = bold ? 1 : 5
  const colorCode = c % 10
  return {
    start: `\u001b[3${c};${boldCode}m`,
    finish: '\u001b[0m',
  }
}

const logHeader = (color) => (...args) => {
  const { start, finish } = getColorCodes(color)
  console.log(start, ...args, finish)
}

const debugRules = {
  logClasses: true,
  logConstructors: false,
  logDepsAsFound: true,
  logDepsAtEnd: true,
  logExportsAsFound: true,
  logExtensions: false,
  logFunctions: false,
  logImportMarking: true,
  logInjectedImports:true,
  logSuperConstructorCalls: false,
  logSuperMethodCalls: false,
}

const indent = '  '

// Track dependencies for validating transpilation output
const dependencies = {
  internal: {},
  track(file, dependency) {
    // ignore self-dependencies
    if (path.basename(file, '.js') === dependency) {
      return
    }

    const deps = dependencies.internal

    if (!deps[file]) {
      deps[file] = []
    }
    if (!deps[file].includes(dependency)) {
      if (debugRules.logDepsAsFound) {
        logInfo('depends on', dependency)
      }
      deps[file].push(dependency)
    }
  },
  toString() {
    return JSON.stringify(dependencies.internal, null, 2)
  },
  forFile(file) {
    const deps = dependencies.internal
    return deps[file]
  }
}

// Track inheritance for class generation
const inheritance = {
  index: {},
  isClass(prop) {
    return inheritance.index[prop] !== undefined
  },
  getParent(child) {
    return inheritance.index[child]
  },
  track(child, parent) {
    if (parent && !inheritance.index[child]) {
      if (debugRules.logExtensions) {
        logInfo(child, 'extends', parent)
      }
      inheritance.index[child] = parent
    } else {
      if (!inheritance.isClass(child)) {
        if (debugRules.logClasses) {
          logInfo(child, 'is a class')
        }
        inheritance.index[child] = null
      }
    }
  },
  toString() {
    return JSON.stringify(inheritance.index, null, 2)
  },
}

const getModule = (filepath = '') => filepath
  .split(path.sep)
  .slice(1)
  .join(path.sep)
  .split('.js')[0]

// Track exported properties for import injection
const exported = {
  propIndex: {}, // exporter -> prop
  track(exporter, prop) {
    const props = exported.propIndex
    const exporters = exported.exporterIndex

    if (!props[exporter]) {
      props[exporter] = []
    }
    if (props[exporter].includes(prop)) {
      logError('property exported multiple times', { exporter, prop })
      return
    }

    if (debugRules.logExportsAsFound) {
      logInfo('exports', prop)
    }

    props[exporter].push(prop)

    if (exporters[prop]) {
      logError('prop', prop, 'already exported by', exporter)
      return
    }

    exporters[prop] = exporter
  },
  exporterIndex: {}, // prop -> exporter
  getExporter(prop) {
    const exporters = exported.exporterIndex
    const exporter = exporters[prop]

    if (!exporter) {
      logError('no export found for', prop, { exporters })
    }
    return exporter
  },
}

const getFileClass = (filepath) => path.basename(filepath, '.js')
const isFileClass = (filepath, className) => getFileClass(filepath) === className

const transpile = {
  // Replace initial ROS3D assignment
  initialROS3DAssignment: [
    // from
    /var ROS3D = ROS3D \|\| \{\n  REVISION \: '0.18.0'\n\};/m,
    // to
    `export const REVISION = '0.18.0';`,
  ],
  // Replace mutations with exported properties
  exportedProperites: (filepath) => [
    // from:
    // ROS3D.MARKER_ARROW = 0;
    /\nROS3D\.(.*)\s+?=\s+?(.*)/g,
    // to:
    // export const MARKER_ARROW = 0;
    (match, $1, $2) => {
      const prop = $1
      const rhs = $2

      exported.track(filepath, prop)

      return `\nexport const ${prop} = ${rhs}`
    },
  ],
  // Remove ROS3D prefix on internal dependencies
  internalDependencies: (filepath) => [
    // from:
    // return ROS3D.findClosestPoint(axisRay, mpRay);
    /(.*)ROS3D\.(\w+)(?!.*=.*)/g,
    // to:
    // return findClosestPoint(axisRay, mpRay);
    (match, $1, $2) => {
      const preStuffs = $1
      const dep = $2

      if (/^\s*(?:\*|\/\/)/.test(preStuffs)) {
        // we're in a block comment, bail
        return match
      }

      // track dependency on $1
      dependencies.track(filepath, dep)

      return `${preStuffs}${dep}`
    }
  ],
  // Replace __proto__ mutation with class extension
  buildInheritanceIndexViaProto: [
    // from:
    // ROS3D.PoseWithCovariance.prototype.__proto__ = THREE.Object3D.prototype;
    /ROS3D.(\w+).prototype.__proto__ = (.*).prototype;[\r\n]?/g,
    // to:
    // set PoseWithCovariance to subclass from THREE.Object3D in inheritance index
    (match, $1, $2) => {
      // track $1 extends $2
      inheritance.track($1, $2)
      // remove it
      return ''
    }
  ],
  // Replace __proto__ mutation with class extension
  buildInheritanceIndexViaObjectAssign: [
    // from:
    // Object.assign(InteractiveMarker.prototype, THREE.EventDispatcher.prototype);
    /Object.assign\((\w+).prototype, (.*).prototype\);/g,
    // to:
    // set InteractiveMarker to subclass from THREE.EventDispatcher in inheritance index
    (match, $1, $2) => {
      // track $1 extends $2
      inheritance.track($1, $2)
      // remove it
      return ''
    }
  ],
  // Refactor methods
  methods: [
    // from:
    // ROS3D.Arrow2.prototype.dispose = function() { ... };
    /ROS3D.(\w+).prototype.(\w+) = function|function\s+(\w+)/g,
    // to:
    // dispose() { ... };
    (match, $1, $2, $3) => {
      if ($1) {
        // matches $1 and $2, replace with method
        const $class = $1
        const $method = $2

        // bail, not our responsibility
        if ($method === '__proto__') {
          return match
        }

        return $method
      } else {
        // matches $3, log skeptically, but don't replace
        if (debugRules.logFunctions) {
          logInfo('found function declaration', { $3 })
        }
        return match
      }
    }
  ],
  // Refactor constructor functions
  constructors: (filepath, state = { foundConstructor: false }) => [
    // from:
    // ROS3D.Arrow2 = function(options) { ... };
    /ROS3D.(\w+)\s*=\s*function/g,
    // to:
    // constructor(options) { ... };
    (match, $1) => {
      const isClass = isFileClass(filepath, $1)
      // if (isClass1 !== isClass2) {
      //   logWarning('class mismatch', {
      //     filepath,
      //     $1,
      //     isInheritanceClass: isClass1,
      //     isFileClass: isClass2,
      //   })
      // }
      if (isClass) {
        if (state.foundConstructor) {
          logError('already found a constructor in this file...', { match, $1 })
        }
        state.foundConstructor = true
        if (debugRules.logConstructors) {
          logInfo('found constructor', { match, $1 })
        }
        return 'constructor'
      } else {
        return match
      }
    }
  ],
  // Refactor super calls
  superCalls: (filepath, state = { superMethodCalls: {} }) => [
    // from:
    // constructor(options) {
    //   ...
    //   THREE.ArrowHelper.call(this, direction, origin, length, 0xff0000);
    //   ...
    // };
    /[\r\n]([\s]*)([\w.]+)\.call\((?:this|that)(.*)/g,
    // to:
    // constructor(options) {
    //   ...
    //   super(direction, origin, length, 0xff0000);
    //   ...
    // }
    (match, indent, $1, $2) => {
      const child = getFileClass(filepath)
      const parent = inheritance.getParent(child)
      const callee = $1
      const args = $2
        .split(/,|\)/)  // split args from leading comma and ending paren
        .slice(1, -1)   // slice out just the args
        .join(',')      // put them back into a string
        .trim()         // trim whitespace

      if (callee === parent) {
        // we got a super constructor call
        if (debugRules.logSuperConstructorCalls) {
          logInfo('found super constructor call', { match, child, parent, args })
        }
        return `\n${indent}super(${args});`
      } else {
        // we got a super method call
        const isMultipart = callee.includes('.')
        const calleeParts = callee.split('.')
        const hasTooManyParts = isMultipart && calleeParts.length > 3
        const partsMatch = isMultipart && calleeParts[0] === child && calleeParts[1] === 'prototype'

        if (hasTooManyParts) {
          logError('super method call invalid, too many parts', { match, child, callee, args })
          return match
        }

        if (!partsMatch) {
          logError('super method call invalid, parts dont match', { match, child, callee, args })
          return match
        }

        const method = isMultipart
          ? calleeParts[2]
          : callee

        if (debugRules.logSuperMethodCalls) {
          logInfo('found super method call', { match, child, parent, method, args })
        }

        if (state.superMethodCalls[method]) {
          logWarning('multiple calls found to super method, refactor may be necessary', { child, method })
        }

        state.superMethodCalls[method] = true

        return `\n${indent}super.${method}(${args});`
      }
    }
  ],
  // Generate class wrappers using inheritance index
  classes: (filepath, state = { isInClass: false, matchedFirstComment: false }) => [
    // from:
    // constructor(options) {
    //   ...
    // }
    //
    // dispose() {
    //   ...
    // }
    // /.*(\*\/).*|[\r\n]+$(?:[\r\n]+$)+((?![\r\n]+))|.*/gm,
    // /(\/\*\*(?:$|[.\r\n])*\*\/(?:$|[\s\r\n])*constructor\(.*)|[\r\n]+$(?:[\r\n]+$)+((?![\r\n]+))|.*/gm,
    /((?:\/\*\*(?:(?:\*[^/]|[^*])+?)\*\/)(?:[\s\r\n])*constructor\(.*)|$(?:[\r\n]$)*((?![\r\n]))|.+/gm,
    // to:
    // export class Arrow2 extends THREE.ArrowHelper {
    //   constructor(options) {
    //     ...
    //   }
    //
    //   dispose() {
    //     ...
    //   }
    // }
    (match, $1, $2) => {
      // $1 matches '/**' + anything not '*/' + '*/' + 'constructor('- aka a block comment followed by a constructor
      // $2 matches the end of a line that isn't followed by a newline char - aka EOF
      const isStart = $1 !== undefined && !state.matchedFirstComment
      const isFinish = $2 !== undefined
      const className = getFileClass(filepath)
      const parent = inheritance.getParent(className)

      if (isStart) {
        const indentedMatch = indent + match.replace(/[\r\n]/g, `\n${indent}`)
        state.matchedFirstComment = true
        state.isInClass = true

        // Track this as an export
        exported.track(filepath, className)

        if (parent) {
          return `export class ${className} extends ${parent} {\n\n${indentedMatch}`
        } else {
          return `export class ${className} {\n\n${indentedMatch}`
        }
      }
      if (state.isInClass) {
        if (!isFinish) {
          return `${indent}${match}`
        } else {
          state.isInClass = false
          return `\n}\n`
        }
      }

      return match
    }
  ],
  imports: (filepath, state = { isStart: false }) => [
    /^/m,
    (match) => {
      const deps = dependencies.forFile(filepath)
      const exporters = {}

      if (deps) {
        // build map of exporter -> dep
        deps.forEach(dep => {
          const exporter = exported.getExporter(dep)
          if (!exporter) {
            logError('cant find exporter for', dep, { filepath })
            return
          }

          // skip self exports
          if (exporter === filepath) {
            return
          }

          // build collection of deps for each exporter
          if (!exporters[exporter]) {
            exporters[exporter] = []
          }
          exporters[exporter].push(dep)

          if (debugRules.logImportMarking) {
            logInfo('marking import of', dep, 'from', exporter)
          }
        })

        // inject imports grouped by exporter
        const importStrings = Object.keys(exporters).map(exporter => {
          const props = exporters[exporter]
          const joinedProps = props.join(', ')
          const modulePath = path
            .relative(path.dirname(filepath), exporter)
            .split(/\.js/)[0]
          const internalModulePath = modulePath.startsWith('.')
            ? modulePath
            : ['.', ...modulePath.split(path.sep)].join(path.sep)
          const importString = `import { ${joinedProps} } from '${internalModulePath}'`

          if (debugRules.logInjectedImports) {
            logInfo('injecting import statement:', importString)
          }

          return importString
        })

        return importStrings.join('\n') + '\n\n'
      }

      return ''
    }
  ]
}

// Transpiles current src to ES6
const transpileToEs6 = function (content, filepath, grunt) {
  logHeader(colors.YELLOW)('\nv -- Processing', filepath, '-- v')

  let transpiled = content

  // transpile content from current format to ES6
  if (filepath === 'src/Ros3D.js') {
    transpiled = transpiled
      .replace(...transpile.initialROS3DAssignment)
  }

  // give replace function access to filepath
  const transpileInternalDependencies = transpile.internalDependencies(filepath)
  const transpileConstructors = transpile.constructors(filepath)
  const transpileSuperCalls = transpile.superCalls(filepath)
  const transpileClasses = transpile.classes(filepath)
  const transpileExportedProperites= transpile.exportedProperites(filepath)

  return transpiled
    .replace(...transpileInternalDependencies)
    .replace(...transpile.buildInheritanceIndexViaProto)
    .replace(...transpile.buildInheritanceIndexViaObjectAssign)
    .replace(...transpile.methods)
    .replace(...transpileConstructors)
    .replace(...transpileSuperCalls)
    .replace(...transpileClasses)
    .replace(...transpileExportedProperites)
}

// Injects es6 imports based on dependency and export
// metadata gathered from transpilation
const injectImports = function (content, filepath, grunt) {
  logHeader(colors.CYAN)('\nv -- Finalizing', filepath, '-- v')

  const pathParts = filepath.split(path.sep)
  const sourceFilePath = ['src', ...pathParts.slice(1)].join(path.sep)

  let transpiled = content

  // give replace function access to sourceFilePath
  const transpileImports = transpile.imports(sourceFilePath)

  return transpiled
    .replace(...transpileImports)
}

module.exports = {
  debugRules,
  dependencies,
  transpileToEs6,
  injectImports,
}

