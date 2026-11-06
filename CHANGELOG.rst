0.1.0 (2026-11-06)
------------------
* feat(maps): generate world maps by slicing visual geometry (#34)
* feat(worlds): lower the bag sorting walls to 0.3 m (#32)
* feat(worlds): add klt lifting warehouse world (#30)
* feat: Added 'gz_models_path' back as optional, and including overlay 'worlds/' and 'models/' paths
* feat: Removed need to specify manual 'gz_models_path' in each sim launch file
* feat(ci): migrate to reusable orchestrator with gist badges (#22)
* feat(ci): update CI workflows (#20)
* feat(): bridge for compressed image transport & parameters for changing gazebo resource paths
  (#14)
* feat(ci): added build and launch testing (#8)
* fix(gazebo): give unpause_sim the robot's namespace (#26)
* fix: add duatic_ros2control repository to repos.list (#25)
* fix(README): update badge links (#13)
* fix(package.xml): changed maintainer email
* refactor(launch): accomodate forwarding of namespace (#6)
* revert:"chore: SIGINT stop without leaving a process running" (#36)
* Added pytest for 'join_env_paths'
* Added support for merged-install workspace paths.
* Bag-lifting and bag-sorting worlds, their maps, and on-demand welding (#24)
* CI & README standardization (#21)
* Bump actions/checkout from 6 to 7 (#18)
* Bump actions/cache from 5 to 6 (#19)
* Gazebo: Failed to load collision meshes (#17)
* Using the refactored duatic_helpers package (#15)
* Move from ros2_testing to duatic_helpers (#12)
* fix xacro include guard and deps
* fix readme and contribution guide
* Next ci fix
* ci fix
* remove timeout
* missed one ci file
* make ci workflow public again
* added ci timeouts
* fix deps
* added repos.list!
* Added duatic_gazebo.urdf.xacro, fixed ci
* rename to duatic_gazebo
* Added duatic_office world (#7)
* Replaced absolute path with relative path to docker-compose
* Robustness update  (#4)
