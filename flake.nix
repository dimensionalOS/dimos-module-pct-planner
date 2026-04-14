{
  description = "DimOS PCT (Point Cloud Tomography) planner module";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    gtsam-extended = {
      url = "github:jeff-hykin/gtsam-extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = { self, nixpkgs, flake-utils, lcm-extended, dimos-lcm, gtsam-extended, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.lcm;
        gtsam = gtsam-extended.packages.${system}.gtsam-cpp;

        # PCT's vendored smoothing lib targets OSQP 0.6.x API
        # (OSQPWorkspace/c_malloc/OSQPData). nixpkgs#osqp is 1.0.0 with a
        # breaking API. Pin 0.6.3 here so `osqp_interface.cc` compiles
        # unmodified.
        osqp06 = pkgs.stdenv.mkDerivation rec {
          pname = "osqp";
          version = "0.6.3";
          src = pkgs.fetchFromGitHub {
            owner = "osqp";
            repo = "osqp";
            rev = "v${version}";
            fetchSubmodules = true;
            sha256 = "sha256-3eVgAn4UkEvjVacOV9adgY9bHa/zjJJbwSmjLU+RZUw=";
          };
          nativeBuildInputs = [ pkgs.cmake ];
          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DUNITTESTS=OFF"
          ];
        };
      in {
        packages.default = pkgs.stdenv.mkDerivation {
          pname = "dimos-module-pct-planner";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            lcm
            gtsam
            pkgs.eigen
            osqp06
            pkgs.boost
            pkgs.tbb
          ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
          ];
        };
      });
}
