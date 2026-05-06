{ pkgs, perSystem, ... }:
let
  zephyr = perSystem.zephyr-nix;
  pythonEnv = zephyr.pythonEnv.override {
    extraPackages = ps: [
      ps.stringcase
    ];
  };

  # esptool depends on ecdsa; some nixpkgs revisions mark that dependency as insecure (CVE-2024-23342).
  pkgsForEsptool = import pkgs.path {
    inherit (pkgs) system;
    config.permittedInsecurePackages = [
      "python3.12-ecdsa-0.19.1"
    ];
  };
  esptool = pkgsForEsptool.esptool;
in
pkgs.mkShell {
  packages = [
    # Minimal SDK: ARM (e.g. STM32 NUCLEO) + Xtensa ESP32-S3 (e.g. esp32s3_devkitc).
    (zephyr.sdk-1_0_0.override {
      targets = [
        "arm-zephyr-eabi"
        "xtensa-espressif_esp32s3_zephyr-elf"
      ];
    })

    pythonEnv

    zephyr.hosttools

    pkgs.cmake
    pkgs.ninja

    esptool

    perSystem.self.formatter
  ];

  shellHook = ''
    export LOCALE_ARCHIVE="${pkgs.glibcLocales}/lib/locale/locale-archive";
    export LC_ALL="C.UTF-8";
    export LANG="C.UTF-8";
    # Force Zephyr/west to use the Nix Python (≥3.12), not /usr/bin/python3.
    export WEST_PYTHON="${pythonEnv}/bin/python3"
    # Prefer Nix cmake/ninja and Zephyr hosttools over ~/.local.
    export PATH="${pkgs.lib.makeBinPath [ pythonEnv pkgs.cmake pkgs.ninja zephyr.hosttools ]}:$PATH"
  '';
}
