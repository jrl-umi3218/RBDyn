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
          inputs.spacevecalg.overlays.flakoboros
        ];

        # overrides.rbdyn = {pkgs-final, ...}:
        # {
        #   spacevecalg = inputs.spacevecalg.packages.${pkgs-final.system}.spacevecalg;
        # };

        overrideAttrs.rbdyn =
          { pkgs-final, drv-prev, ... }:
          {
            pname = "rbdyn-nanobind";
            src = lib.cleanSource ./.;
            cmakeFlags = [
              (lib.cmakeBool "PYTHON_BINDINGS" false)
              (lib.cmakeBool "NANOBIND_BINDINGS" true)
              # FIXME: {
              # without this fixupPhase fails during patchelf phase because rpath has a reference to /build/ but I could not figure out why exactly
              (lib.cmakeBool "CMAKE_SKIP_BUILD_RPATH" true)
              (lib.cmakeBool "CMAKE_BUILD_WITH_INSTALL_RPATH" true)
              (lib.cmakeBool "CMAKE_INSTALL_RPATH_USE_LINK_PATH" true)
              "-DCMAKE_INSTALL_RPATH=$ORIGIN"
              # } FIXME
            ];
            nativeBuildInputs =
              with pkgs-final;
              [
                python3Packages.python
                jrl-cmakemodulesv2
              ]
              ++ drv-prev.nativeBuildInputs;
            propagatedBuildInputs =
              with pkgs-final;
              [
                python3Packages.nanoeigenpy
                python3Packages.nanobind
              ]
              ++ drv-prev.propagatedBuildInputs;
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
