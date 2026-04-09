{ pkgs, perSystem, ... }:
let
  zephyr = perSystem.zephyr-nix;
in
pkgs.mkShell {
  packages = [
    (zephyr.sdk-1_0_0.override {
      targets = [
        "arm-zephyr-eabi"
      ];
    })

    (zephyr.pythonEnv.override {
      extraPackages = ps: [
        ps.stringcase
      ];
    })

    zephyr.hosttools

    pkgs.cmake
    pkgs.ninja

    perSystem.self.formatter
  ];

  shellHook = ''
    export LOCALE_ARCHIVE="${pkgs.glibcLocales}/lib/locale/locale-archive";
    export LC_ALL="C.UTF-8";
    export LANG="C.UTF-8";
  '';
}
