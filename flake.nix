{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";

    blueprint.url = "github:numtide/blueprint";

    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    zephyr-nix = {
      url = "github:nix-community/zephyr-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = inputs:
    inputs.blueprint {
      inherit inputs;
      prefix = "nix";
    };

  nixConfig = {
    extra-substituters = [
      "https://cache.freedom.ind.br"
    ];
    extra-trusted-public-keys = [
      "cache.freedom.ind.br:4+Tt+AZreSw+P7xP0d6eHtIHhSAlkFbSa/9ugOkiMSM="
    ];
  };
}
