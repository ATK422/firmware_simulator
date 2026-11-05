{
  description = "firmware_simulator: an RP2040 emulator for UWRT's robot boards";

  # The team's environment (https://github.com/atk422/uwrt_nix): ARM GCC, the Pico SDK, the micro-ROS library,
  # Rust with thumbv6m-none-eabi, ROS 2 Humble and the riptide messages, the same versions on every machine
  inputs.uwrt.url = "github:atk422/uwrt_nix";

  outputs =
    { uwrt, ... }:
    {
      devShells = uwrt.lib.forAllSystems (
        pkgs:
        let
          system = pkgs.stdenv.hostPlatform.system;
          firmware = uwrt.devShells.${system}.firmware;
        in
        {
          # The emulator, both firmware repositories and the host side (ROS, CAN tools) in one shell
          default = pkgs.mkShell {
            inputsFrom = [
              firmware
              uwrt.devShells.${system}.ros
            ];
            packages = pkgs.lib.optionals pkgs.stdenv.hostPlatform.isLinux [
              # candump, cansend and friends, and `ip` for scripts/vcan_up.sh (Linux only)
              pkgs.can-utils
              pkgs.iproute2
            ];
            # inputsFrom brings the other shells' packages but not their variables
            inherit (firmware)
              PICO_SDK_PATH
              PICO_TOOLCHAIN_PATH
              CMAKE_POLICY_VERSION_MINIMUM
              MICRO_ROS_LIB_DIR
              ;
          };

          # Only the firmware tools: enough to build and test the emulator and the firmware images
          inherit firmware;
          inherit (uwrt.devShells.${system}) ros;
        }
      );
    };
}
