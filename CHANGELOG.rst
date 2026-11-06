^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package duatic_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-11-06)
------------------
* feat(maps): generate world maps by slicing visual geometry (#34)
* feat(worlds): lower the bag sorting walls to 0.3 m (#32)
* feat(worlds): add klt lifting warehouse world (#30)
* feat(worlds): bag lifting and bag sorting worlds with their maps, and welding models together on
  demand (#24)
* feat(worlds): add the duatic_office world (#7)
* feat: find an overlay's worlds/ and models/ without gz_models_path in every launch file;
  gz_models_path stays as an optional extra
* feat: support workspaces built with --merge-install
* feat: a bridge for compressed image transport, and parameters for the Gazebo resource paths (#14)
* feat: add duatic_gazebo.urdf.xacro
* feat: a warehouse startup test, with robustness fixes (#4)
* fix(gazebo): give unpause_sim the robot's namespace (#26)
* fix: load the collision meshes (#17)
* fix: the xacro include guard and dependencies
* fix: a relative path to docker-compose instead of an absolute one
* fix(package.xml): change the maintainer email
* refactor(launch): accommodate forwarding of the namespace (#6)
* refactor: use duatic_helpers instead of ros2_testing (#12, #15)
* docs: fix the README and the contribution guide
