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
            ];
            nativeCheckInputs = with pkgs-final; [ python3Packages.pythonImportsCheckHook ];
            pythonImportsCheck = [ "rbdyn" ];
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
            # XXX: Without this fixupPhase fails due to RPATHS references to /build/
            preFixup = ''
              patchelf --shrink-rpath --allowed-rpath-prefixes "$NIX_STORE" $out/${pkgs-final.python3Packages.python.sitePackages}/rbdyn/_rbdyn.*.so
            '';
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
