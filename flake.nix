{
  description = "Flake for C++/CMake development and SFML";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
    flake-utils.url = "github:numtide/flake-utils";
    old-nixpkgs.url = "github:NixOs/nixpkgs/nixos-23.11";
  };

  outputs = { self, nixpkgs, old-nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        oldPkgs = import old-nixpkgs { inherit system; };
      in {
        devShells.default = pkgs.mkShell {
          name = "cpp-sfml-2.5";
          
          packages = with pkgs; [
            cmake gcc gdb gnumake
            pkg-config clang-tools
            cmake-language-server
            ninja

          ];

          buildInputs = [ oldPkgs.sfml ];

          shellHook = ''
            export SFML_DIR="${oldPkgs.sfml}/lib/cmake/SFML"
            echo "Done. SFML available at $SFML_DIR ."
          '';
        };
      });
}

