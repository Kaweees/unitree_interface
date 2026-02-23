{
  pkgs ? import <nixpkgs> { config.allowUnfree = true; },
}:

let
  python = pkgs.python312;
in
pkgs.mkShell {
  buildInputs = with pkgs; [
    python
    uv
    nixfmt
    just
    cyclonedds
  ];

  shellHook = ''
    export TMPDIR=/tmp
    export UV_PYTHON="${python}/bin/python"
    export CYCLONEDDS_HOME="${pkgs.cyclonedds}"
    export CMAKE_PREFIX_PATH="${pkgs.cyclonedds}:$CMAKE_PREFIX_PATH"
    export LD_LIBRARY_PATH="${pkgs.stdenv.cc.cc.lib}/lib:${pkgs.zlib}/lib:${pkgs.cyclonedds}/lib:$LD_LIBRARY_PATH"
    just install
  '';
}
