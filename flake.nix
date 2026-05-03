{
  description = "RBDyn provides a set of classes and functions to model the dynamics of rigid body systems.";

  inputs.mc-rtc-nix.url = "github:mc-rtc/nixpkgs";

  inputs.spacevecalg.url = "github:jrl-umi3218/SpaceVecAlg/pull/67/head";

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        extraDevPyPackages = [ "rbdyn" ];

        overlays = [
          (pkgs-final: pkgs-prev: {
            spacevecalg = inputs.spacevecalg.packages.${pkgs-final.system}.spacevecalg;
          })
        ];

        overrideAttrs.rbdyn = 
        { pkgs-final, drv-prev, ... }:
        {
          src = lib.cleanSource ./.;
          cmakeFlags = [
            (lib.cmakeBool "PYTHON_BINDINGS" false)
            (lib.cmakeBool "NANOBIND_BINDINGS" true)
          ];
          nativeBuildInputs = with pkgs-final; [
            python3Packages.python
          ] ++ drv-prev.nativeBuildInputs;
          propagatedBuildInputs = with pkgs-final; [
            python3Packages.nanoeigenpy
            python3Packages.nanobind
          ] ++ drv-prev.propagatedBuildInputs;
        };

        pyPackages = {
          rbdyn =
            {
              pkgs,
              toPythonModule,
            }:
            (toPythonModule (pkgs.rbdyn.override { }));
        };
      }
    );
}
