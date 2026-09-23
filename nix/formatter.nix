{ inputs, pkgs }:
inputs.treefmt-nix.lib.mkWrapper pkgs {
  imports = [ inputs.pedantix.treefmtModules.default ];

  projectRootFile = "flake.nix";

  programs = {
    black.enable = true;
    clang-format = {
      enable = true;
      package = pkgs.clang-tools_19;
    };
    pedantix.enable = true;
    shfmt.enable = true;
  };

  settings.formatter = {
    clang-format = {
      options = [
        "-i"
        "-style=file:${inputs.zephyr-nix.inputs.zephyr}/.clang-format"
      ];
      excludes = [
        "build/*"
        "twister-out*/*"
      ];
    };
  };
}
