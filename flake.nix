{
  description = "RBDyn provides a set of classes and functions to model the dynamics of rigid body systems.";

  inputs.mc-rtc-nix.url = "github:mc-rtc/nixpkgs";

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        pyOverrideAttrs.rbdyn =
          { pkgs-final, drv-prev, ... }:
          {
            src = lib.cleanSource ./.;
            nativeBuildInputs = drv-prev.nativeBuildInputs ++ [ pkgs-final.jrl-cmakemodules ];
          };
      }
    );
}
