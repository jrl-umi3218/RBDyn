{
  description = "RBDyn provides a set of classes and functions to model the dynamics of rigid body systems.";

  inputs.mc-rtc-nix.url = "github:mc-rtc/nixpkgs";

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.rbdyn = {
          src = lib.cleanSource ./.;
        };
      }
    );
}
