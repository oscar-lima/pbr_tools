This package contains a gazebo_world of the ICO office environemnt, and launch files to start gazebo and spawn Tiago in it.

To start the ICO environment and spawn Tiago, you can use the launch-file launch/tiago_ico.launch.
Before you start it the first time you have to copy the config-file config/pose.yaml to ~/.pal/pose.yaml.
This provides amcl-localization with Tiagos initial pose in that environment.

The 2d-map that the environment is based on can be found in config/ico.
