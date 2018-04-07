const {
  debugRules,
  dependencies,
  inheritance,
  injectImports,
  transpileToEs6
} = require('./es6-transpiler')

// Export Grunt config
module.exports = function(grunt) {

  grunt.initConfig({
    pkg: grunt.file.readJSON('package.json'),
    concat: {
      build: {
        src  : ['./src/*.js', './src/**/*.js'],
        dest : './build/ros3d.js'
      }
    },
    jshint: {
      options: {
        jshintrc: '.jshintrc'
      },
      files: [
        'Gruntfile.js',
        './build/ros3d.js',
        './tests/*.js'
      ]
    },
    karma: {
      build: {
        configFile: './test/karma.conf.js',
        singleRun: true,
        browsers: ['PhantomJS']
      }
    },
    uglify: {
      options: {
        report: 'min'
      },
      build: {
        src: './build/ros3d.js',
        dest: './build/ros3d.min.js'
      }
    },
    watch: {
      dev: {
        options: {
          interrupt: true
        },
        files: [
          './src/*.js',
          './src/**/*.js'
        ],
        tasks: ['concat']
      },
      build_and_watch: {
        options: {
          interrupt: true
        },
        files: [
          'Gruntfile.js',
          '.jshintrc',
          './src/*.js',
          './src/**/*.js'
        ],
        tasks: ['build']
      }
    },
    clean: {
      options: {
        force: true
      },
      doc: ['./doc']
    },
    jsdoc: {
      doc: {
        src: [
          './src/*.js',
          './src/**/*.js'
        ],
        options: {
          destination: './doc',
          configure: 'jsdoc_conf.json'
        }
      }
    },
    pipe: {
      transpile: {
        options: {
          process: transpileToEs6,
        },
        files: [{
          expand: true,
          cwd: 'src',
          src: [
            '*.js',
            '**/*.js',
          ],
          dest: 'src-esm/',
        }]
      },
      transpile_imports: {
        options: {
          process: injectImports,
        },
        files: [{
          expand: true,
          cwd: 'src-esm',
          src: [
            '*.js',
            '**/*.js',
          ],
          dest: 'src-esm/',
        }]
      },
      transpile_index: {
        files: [{
          expand: true,
          cwd: 'es6-support',
          src: [
            'index.js'
          ],
          dest: 'src-esm/'
        }]
      }
    },
    execute: {
      transpile: {
        call: (grunt, options) => {
          console.log()
          if (debugRules.logInternalDepsAtEnd) {
            console.log('Internal dependencies')
            console.log(dependencies.internalToString())
          }
          if (debugRules.logExternalDepsAtEnd) {
            console.log('External dependencies')
            console.log(dependencies.externalToString())
          }
          if (debugRules.logInheritanceAtEnd) {
            console.log('Inheritance hierarchy')
            console.log(inheritance.toString())
          }

          console.log()
        },
      }
    }
  });

  grunt.loadNpmTasks('grunt-contrib-concat');
  grunt.loadNpmTasks('grunt-contrib-jshint');
  grunt.loadNpmTasks('grunt-contrib-watch');
  grunt.loadNpmTasks('grunt-contrib-uglify');
  grunt.loadNpmTasks('grunt-contrib-clean');
  grunt.loadNpmTasks('grunt-jsdoc');
  grunt.loadNpmTasks('grunt-karma');
  grunt.loadNpmTasks('grunt-pipe');
  grunt.loadNpmTasks('grunt-execute');

  grunt.registerTask('dev', ['concat', 'watch']);
  grunt.registerTask('build', ['concat', 'jshint', 'uglify']);
  grunt.registerTask('build_and_watch', ['watch']);
  grunt.registerTask('doc', ['clean', 'jsdoc']);
  grunt.registerTask('transpile', ['pipe', 'execute']);
};
