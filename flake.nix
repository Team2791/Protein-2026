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

              # --- graphics/display stack for sim GUI ---
              xorg.libX11
              xorg.libXext
              xorg.libXrender
              xorg.libXrandr
              xorg.libXi
              xorg.libXcursor
              xorg.libXfixes
              xorg.libXinerama
              mesa
              libGL
              libGLU

              # Wayland (needed if you're on a Wayland compositor)
              wayland
              libxkbcommon

              # GTK/font rendering (WPILib uses Java AWT/Swing or JavaFX)
              gtk3
              glib
              pango
              cairo
              gdk-pixbuf
              fontconfig
              freetype
            ]);
          multiPkgs =
            pkgs:
            (with pkgs; [
              openssl
              gcc
              util-linux
              harfbuzz
              atk
              libepoxy
            ]);
          runScript = "bash";
        }).env;
    };
}
