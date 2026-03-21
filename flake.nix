{
  description = "FRC robotics development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    frc-nix.url = "github:frc4451/frc-nix";
  };

  outputs =
    {
      self,
      nixpkgs,
      frc-nix,
    }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
    in
    {
      devShells.${system}.default =
        (pkgs.buildFHSEnv {
          name = "robotics-dev";
          targetPkgs =
            pkgs:
            (with pkgs; [
              nil
              nixd
              nixfmt
              javaPackages.compiler.temurin-bin.jdk-17
              gradle
              jdt-language-server
              nodejs
              prettier
              bun
              openssl
              gcc
              frc-nix.packages.${system}.glass
            ]);
          multiPkgs =
            pkgs:
            (with pkgs; [
              openssl
              gcc
              libxkbcommon
              libx11
              libXt
              libXinerama

            ]);
          runScript = "bash";
        }).env;
    };
}
