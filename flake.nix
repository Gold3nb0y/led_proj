{
  description = "Led Project Flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs = { self, nixpkgs }: 
    let
        system = "x86_64-linux";
        pkgs = import nixpkgs { inherit system; };
    in {
        devShells.x86_64-linux.default = pkgs.mkShell {
            buildInputs = with pkgs; [
                gcc-arm-embedded
                openocd
                libtool
                automake
                autoconf
                zsh
                libusb1
            ];
        };
    };
}
