{ cell, inputs }:
let
  inherit (inputs) self std nixpkgs ros2nix;
  godot = inputs.godot.godot.packages.godot-master;
  # godot = inputs.godot.godot.packages.godot-master.overrideAttrs (oldAttrs: {
  #   src = inputs.godot-patch;
  # });
  rosPkgs = ros2nix.humble.packages;
  rosSetupHook = ros2nix.common.packages.setupHook;
in
rec {
  default = rosPkgs.mkRosWorkspace {
    pkgs = [
      godot
      godot.godot-cpp
      rosPkgs.rclcpp
      rosPkgs.std_msgs
      rosPkgs.geometry_msgs
      rosPkgs.sensor_msgs
      rosPkgs.nav_msgs
      rosPkgs.ros2cli
      rosPkgs.ros2run
      rosPkgs.ros2topic
      rosPkgs.ros2node
      rosPkgs.rviz2
    ];
    nativeBuildInputs = with nixpkgs; [
      pkg-config
      meson
      ninja
    ];
  };
}
