{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };
  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.simpleFlake {
      inherit self nixpkgs;
      name = "simple ros flake";
      overlay = nix-ros-overlay.overlays.default;
      shell =
        { pkgs }:
        pkgs.mkShell {
          packages = with pkgs; [
            colcon
            clang
            clang-tools
            ruff
            gcc
            # ... other non-ROS packages
            (
              with pkgs.rosPackages.jazzy;
              buildEnv {
                paths = [
                  ament-cmake-core
                  python-cmake-module
                  ros-core
                  turtlesim
                  demo-nodes-py
                  demo-nodes-cpp
                  rqt
                  rqt-common-plugins
                  rqt-graph
                  rosbag2
                  tf2-ros
                  tf2-ros-py
                  tf2-tools
                  # ... other ROS packages
                ];
              }
            )
          ];
        };
    };
}
