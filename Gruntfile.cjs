/* global module */
// Export Grunt config
module.exports = function(grunt) {

  grunt.initConfig({
    pkg: grunt.file.readJSON('package.json'),
    shell: {
      build: {
        command: 'rollup -c'
      },
      'test-esm': {
        command: 'webpack --config esm-test.webpack.config.js'
      },
      lint: {
        command: 'eslint Gruntfile.cjs ./src/*.js ./src/**/*.js ./test/**/*.test.js'
      },
    },
    karma: {
      build: {
        configFile: './test/karma.conf.cjs',
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
  });

  grunt.loadNpmTasks('grunt-contrib-watch');
  grunt.loadNpmTasks('grunt-contrib-clean');
  grunt.loadNpmTasks('grunt-jsdoc');
  grunt.loadNpmTasks('grunt-karma');
  grunt.loadNpmTasks('grunt-shell');

  grunt.registerTask('build', ['shell:lint', 'shell:build']);
  grunt.registerTask('build_and_watch', ['build', 'watch']);
  grunt.registerTask('doc', ['clean', 'jsdoc']);
  grunt.registerTask('lint', ['shell:lint',]);
  grunt.registerTask('lint-fix', ['shell:lint-fix',]);
  grunt.registerTask('test', ['test-esm', 'karma',]);
  grunt.registerTask('test-esm', ['shell:test-esm']);
};
