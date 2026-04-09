{ pkgs, inputs }:
inputs.treefmt-nix.lib.mkWrapper pkgs {
  projectRootFile = "flake.nix";

  programs = {
    black.enable = true;
    clang-format = {
      enable = true;
      package = pkgs.clang-tools_19;
    };
    nixpkgs-fmt.enable = true;
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
