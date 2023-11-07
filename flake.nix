{
  inputs = {
    std.url = "github:divnix/std";
    std.inputs.nixpkgs.follows = "nixpkgs";
    godot.url = "github:Pylgos/nix-godot";
    ros2nix.url = "github:Pylgos/ros2nix";
    nixpkgs.follows = "godot/nixpkgs";
  };

  outputs = { std, self, ... } @ inputs: std.growOn
    {
      inherit inputs;
      cellsFrom = ./nix;
      cellBlocks = with std.blockTypes; [
        (installables "packages")
        (devshells "devshells")
      ];
    }
    {
      packages = std.harvest self [ "godot-ros" "packages" ];
      devShells = std.harvest self [ "godot-ros" "devshells" ];
    };
}

