const {
  debugRules,
  dependencies,
  inheritance,
  injectImports,
  transpileToEs6
} = require('./es6-transpiler');

// Export Grunt config
module.exports = function(grunt) {

  grunt.initConfig({
    pkg: grunt.file.readJSON('package.json'),
    eslint: {
      lint: {
        options: {
          configFile: '.eslintrc',
        },
        src: [
          'Gruntfile.js',
          './src/*.js',
          './src/**/*.js',
          './tests/*.js'
        ],
      },
      fix: {
        options: {
          configFile: '<%= eslint.lint.options.configFile  %>',
          fix: true
        },
        src: '<%= eslint.lint.src  %>',
      }
    },
    shell: {
      build: {
        command: 'rollup -c'
      }
    },
    karma: {
      build: {
        configFile: './test/karma.conf.js',
        singleRun: true,
        browsers: process.env.CI ? ['FirefoxHeadless'] : ['Firefox'] // eslint-disable-line
      }
    },
    watch: {
      build_and_watch: {
        options: {
          interrupt: true
        },
        files: [
          'Gruntfile.js',
          '.eslintrc',
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
          console.log();
          if (debugRules.logInternalDepsAtEnd) {
            console.log('Internal dependencies');
            console.log(dependencies.internalToString());
          }
          if (debugRules.logExternalDepsAtEnd) {
            console.log('External dependencies');
            console.log(dependencies.externalToString());
          }
          if (debugRules.logInheritanceAtEnd) {
            console.log('Inheritance hierarchy');
            console.log(inheritance.toString());
          }

          console.log();
        },
      }
    }
  });

  grunt.loadNpmTasks('grunt-contrib-watch');
  grunt.loadNpmTasks('grunt-contrib-clean');
  grunt.loadNpmTasks('grunt-jsdoc');
  grunt.loadNpmTasks('grunt-karma');
  grunt.loadNpmTasks('grunt-pipe');
  grunt.loadNpmTasks('grunt-execute');
  grunt.loadNpmTasks('grunt-shell');
  grunt.loadNpmTasks('gruntify-eslint');

  grunt.registerTask('transpile', ['pipe', 'execute']);
  grunt.registerTask('build', ['eslint:lint', 'pipe', 'shell']);
  grunt.registerTask('build_and_watch', ['build', 'watch']);
  grunt.registerTask('doc', ['clean', 'jsdoc']);
  grunt.registerTask('lint', ['eslint:lint',]);
  grunt.registerTask('lint-fix', ['eslint:fix',]);
  grunt.registerTask('test', ['karma',]);
};
