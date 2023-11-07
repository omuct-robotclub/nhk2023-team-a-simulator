{ cell, inputs }:
let
  inherit (inputs) self std nixpkgs ros2nix;
  godot = inputs.godot.godot.packages.godot-master;
  rosPkgs = ros2nix.humble.packages;
  rosSetupHook = ros2nix.common.packages.setupHook;
in
rec {
  default = godot-ros;

  godot-ros =
    nixpkgs.stdenv.mkDerivation {
      pname = "godot-ros";
      version = "0.1.0";

      src = std.incl self [
        (self + "/src")
        (self + "/meson.build")
      ];

      buildInputs = [
        godot
        godot.godot-cpp
        rosPkgs.rclcpp
        rosPkgs.std_msgs
        rosPkgs.geometry_msgs
      ];

      postInstall = ''
        substitute ${./godot-ros.gdextension.in} $out/godot-ros.gdextension \
          --replace @libPath@ $out/lib/libgodot-ros.so
      '';

      nativeBuildInputs = with nixpkgs; [
        pkg-config
        meson
        ninja
      ];
    };

  test = godot.mkProject {
    name = "godot-ros-test";
    addons = [ godot-ros ];
  };
}
