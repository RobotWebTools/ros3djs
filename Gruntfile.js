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
  });

  grunt.loadNpmTasks('grunt-contrib-watch');
  grunt.loadNpmTasks('grunt-contrib-clean');
  grunt.loadNpmTasks('grunt-jsdoc');
  grunt.loadNpmTasks('grunt-karma');
  grunt.loadNpmTasks('grunt-shell');
  grunt.loadNpmTasks('gruntify-eslint');

  grunt.registerTask('build', [
    // 'eslint:lint', // TODO restore this after updating ESLint to make it compatible with class fields syntax.
    'shell'
  ]);
  grunt.registerTask('build_and_watch', ['build', 'watch']);
  grunt.registerTask('doc', ['clean', 'jsdoc']);
  grunt.registerTask('lint', ['eslint:lint',]);
  grunt.registerTask('lint-fix', ['eslint:fix',]);
  grunt.registerTask('test', ['karma',]);
};
