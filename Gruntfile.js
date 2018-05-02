
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
        browsers: ['PhantomJS']
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
    }
  });

  grunt.loadNpmTasks('grunt-contrib-concat');
  grunt.loadNpmTasks('grunt-contrib-watch');
  grunt.loadNpmTasks('grunt-contrib-clean');
  grunt.loadNpmTasks('grunt-jsdoc');
  grunt.loadNpmTasks('grunt-karma');
  grunt.loadNpmTasks('grunt-pipe');
  grunt.loadNpmTasks('grunt-execute');
  grunt.loadNpmTasks('grunt-shell');
  grunt.loadNpmTasks('gruntify-eslint');

  grunt.registerTask('dev', ['concat', 'watch']);
  grunt.registerTask('build', ['eslint:lint', 'shell']);
  grunt.registerTask('build_and_watch', ['watch']);
  grunt.registerTask('doc', ['clean', 'jsdoc']);
  grunt.registerTask('lint', ['eslint:lint',]);
  grunt.registerTask('lint-fix', ['eslint:fix',]);
};
