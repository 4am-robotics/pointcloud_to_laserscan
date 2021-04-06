^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointcloud_to_laserscan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.5 (2021-04-06)
------------------
* Merge pull request `#22 <https://github.com/ipa320/pointcloud_to_laserscan/issues/22>`_ from fmessmer/fix/TF_REPEATED_DATA
  update stamp to avoid TF_REPEATED_DATA
* only check stamp
* only publish on updated transform to avoid TF_REPEATED_DATA
* update stamp to avoid TF_REPEATED_DATA
* update travis config
* Merge pull request `#21 <https://github.com/ipa320/pointcloud_to_laserscan/issues/21>`_ from fmessmer/fix_catkin_lint
  fix catkin_lint
* fix catkin_lint
* Contributors: Felix Messmer, fmessmer

1.3.4 (2020-10-03)
------------------
* Merge pull request `#20 <https://github.com/ipa320/pointcloud_to_laserscan/issues/20>`_ from fmessmer/test_noetic
  test noetic
* add missing run_depend openni2_launch
* use DOCKER_IMAGE=ros:noetic-perception
* Bump CMake version to avoid CMP0048 warning
* add noetic jobs
* Contributors: Felix Messmer, fmessmer

1.3.3 (2020-03-18)
------------------
* Merge pull request `#19 <https://github.com/ipa320/pointcloud_to_laserscan/issues/19>`_ from lindemeier/fix/change_log_level_and_freq_avoiding_log_spam
  change log level and frequency
* consistent log level and catching TransformException
* catch each exception an handle them according to criticality
* change log level and frequency
* Merge pull request `#18 <https://github.com/ipa320/pointcloud_to_laserscan/issues/18>`_ from fmessmer/feature/python3_compatibility
  [ci_updates] pylint + Python3 compatibility
* activate pylint checks from feature branch
* Merge pull request `#17 <https://github.com/ipa320/pointcloud_to_laserscan/issues/17>`_ from fmessmer/ci_updates
  [travis] ci updates
* sort travis.yml
* add CATKIN_LINT=pedantic
* update travis.yml
* catkin_lint fixes
* Merge pull request `#15 <https://github.com/ipa320/pointcloud_to_laserscan/issues/15>`_ from fmessmer/feature/frame_publisher_extended_combined
  feature/frame publisher extended combined
* use ros::Time(0) for lookupTransform
* add debug info
* combine and harmonize functionality
* Added frame publisher that allows fixing rotations in transform from one frame to another
* Contributors: Felix Messmer, fmessmer, tsl

1.3.2 (2019-08-06)
------------------
* Merge pull request `#13 <https://github.com/ipa320/pointcloud_to_laserscan/issues/13>`_ from benmaidel/feature/melodify
  [Melodic] replaced deprecated pluginlib export macro
* update travis.yml, add kinetic and melodic checks
* replaced deprecated pluginlib export macro
* Contributors: Benjamin Maidel, Felix Messmer, fmessmer

1.3.1 (2019-03-14)
------------------
* Merge pull request `#12 <https://github.com/ipa320/pointcloud_to_laserscan/issues/12>`_ from ipa-fxm/review_frame_publisher
  review frame publisher
* frame_publisher tf2 migration
* fix tf lookup timestamps
* Merge pull request `#11 <https://github.com/ipa320/pointcloud_to_laserscan/issues/11>`_ from ipa320/fix_install_tags
  fix_install_tags
* fix_install_tags
* Merge pull request `#10 <https://github.com/ipa320/pointcloud_to_laserscan/issues/10>`_ from souravran/patch-1
  update to industrial_ci
* update to industrial_ci
* Merge pull request `#9 <https://github.com/ipa320/pointcloud_to_laserscan/issues/9>`_ from ipa-bnm/fix/output_flooding
  prevent console output flooding with errors
* added at least one pointcloud point to roi filter to prevent console output flooding with errors
* change logging level to debug
* Merge pull request `#6 <https://github.com/ipa320/pointcloud_to_laserscan/issues/6>`_ from ipa-fmw/travis
  Travis config
* Create .travis.rosinstall
* Create .travis.yml
* Merge pull request `#5 <https://github.com/ipa320/pointcloud_to_laserscan/issues/5>`_ from ipa-svn/indigo-devel
  New cleanedup verison of pointcloud to laserscan including roi filter and frame publisher
* Update package.xml
  Fixed tabs vs spaces
* Added README with description about the package functionality
* Cleanup indentation, function names, and license agreement.
* Merge pull request `#3 <https://github.com/ipa320/pointcloud_to_laserscan/issues/3>`_ from ipa-svn/feature/without_msgs_filter
  Merge clean and working version without message filter interface: Feature/without msgs filter
* Merge pull request `#2 <https://github.com/ipa320/pointcloud_to_laserscan/issues/2>`_ from ipa-fmw/feature/without_msgs_filter
  activate filter again
* activate filter again
* updated timestamp for published transform
* Changed log level from error log used for debug purposes.
* Removed message filter stuff since it seems to cause disfunction when the nodelets run for a long time
* cleanup and added test cb to debug crach in long time test.
* Cleanup code and set debug logging to error to see something on the screen also if started from other pc: Should be removed once craches are fixed.
* relax time restriction at lookup
* cleanup, removed unused parts and declare variables outside loop.
* Cleanup: removed function already commented out, functionality is in seperate files/nodelets
* Changed max range assignement to a bit less than max range since the assignement of scan does not work otherwise; value < max_range required
* Removed the uggly hack with statistical filtering from within the p2l nodelet sinse this functionality is not provided seperately by the nodelet roi_outlier_removal_nodelet added in the last commit. Keep code commented out until tested.
* Moved roi filter fo separate nodelet. The new nodelet provides the functionality of reducing a pointcloud to the region of interest in the same way as done within the pointcloud to laserscan, but without creating the scan.
* Added frame publisher that should not stay in this package. The frame publisher publishes a new frame with the origin and rotation based on two other frames accoring to settings.
* Uggly fix: split the pc reduction into pre and past filtering parts. The pre filtering part uses the fix z_min = 0 in base link in order to keep the floor redings. No floor redings -> no good filter tuning. This should be done by a pointcloud reduction node combined with filtering befor the pc2laserscan
* Moved static filtering option to the ipa_pointcloud_to_laserscan nodelet to quickly use logging. + Bugfixes
* added possibility to include a pcl statistical outlier filter to the reduced pointclud.
* Merge pull request `#3 <https://github.com/ipa320/pointcloud_to_laserscan/issues/3>`_ from ipa-svn/indigo-devel
  P2L without parameter lookup in callback
* Adapted opening angle of scan
* use cost declarations for paramerters not modified by the function.
* Use debug stream instead of info stream logging in cloud cb function
* cleanup
* Moved outlier filter to separate files and removed parameter lookup in callback
* Fixed wrong time in tf lookup, now using pointcloud time. Added processing time infor logging.
* Added general launch file for nodelet.
* Moved cob parameter settings to separate files.
* Merge pull request `#1 <https://github.com/ipa320/pointcloud_to_laserscan/issues/1>`_ from ipa-svn/indigo-devel
  ipa changes moved from navigation package
* Added setting for max noise cluster distance. All clusters further away are not considered to be noise clusters.
* Added missing msgs dependencies
* Added missing geometry msgs dependencies
* use camera arg consistently and assign parameters for the ipa scna filter.
* updated author and node name
* added ipa nodelet to xml
* build both ipa and original verison
* add own class for the ipa nodelet to be able to build both ipa and original version at the same time
* added ipa test launch files
* Added scan outlier filter for removal of strange noise clusters in the pointcloud
* Reduce computational time by  - determining which points out of the pointcloud to use in the pointcloud source frame instead of in the target frame (the borders are transformed to the pointcloud frame instead of other way around). - calculate the scan point out of the original point cloud.
* remove leading / of frame id if present which makes it possible to use messges with non-tf2-compatible fram notation
* build ipa source files instead.
* added node class for ipa nodelet
* added copy of nodlet for ipa changes
* Contributors: Benjamin Maidel, Felix Messmer, Florian Weisshardt, Sourav Senapati, ipa-fxm, ipa-svn, ips-svn, msh, svn

1.3.0 (2015-06-09)
------------------
* Fix pointcloud to laserscan transform tolerance issues
* Move pointcloud_to_laserscan to new repository
* Contributors: Paul Bovbel

1.2.7 (2015-06-08)
------------------

* Cleanup pointcloud_to_laserscan launch files
* Contributors: Paul Bovbel

1.2.6 (2015-02-04)
------------------
* Fix default value for concurrency
* Fix multithreaded lazy pub sub
* Contributors: Paul Bovbel

1.2.5 (2015-01-20)
------------------
* Switch to tf_sensor_msgs for transform
* Set parameters in sample launch files to default
* Add tolerance parameter
* Contributors: Paul Bovbel

1.2.4 (2015-01-15)
------------------
* Remove stray dependencies
* Refactor with tf2 and message filters
* Remove roslaunch check
* Fix regressions
* Refactor to allow debug messages from node and nodelet
* Contributors: Paul Bovbel

1.2.3 (2015-01-10)
------------------
* add launch tests
* refactor naming and fix nodelet export
* set default target frame to empty
* clean up package.xml
* Contributors: Paul Bovbel

1.2.2 (2014-10-25)
------------------
* clean up package.xml
* Fix header reference
* Fix flow
* Fix pointer assertion
* Finalize pointcloud to laserscan
* Initial pointcloud to laserscan commit
* Contributors: Paul Bovbel
