# Rebuilds the hand-written test programs. The ELFs are committed so the tests do not need an ARM toolchain.
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
PROGRAMS = blink watchdog sysreset lockup profile coverage embassy_blinky

all: $(PROGRAMS:=.elf)

%.elf: %.S link.ld
	$(AS) -mcpu=cortex-m0plus -o $*.o $<
	$(LD) -T link.ld -o $@ $*.o
	rm -f $*.o

# With a line table (the coverage tests map instructions to its lines)
coverage.elf: coverage.S link.ld
	$(AS) -g -mcpu=cortex-m0plus -o coverage.o $<
	$(LD) -T link.ld -o $@ coverage.o
	rm -f coverage.o

clean:
	rm -f *.elf *.o

# The embassy test firmware needs the thumbv6m-none-eabi Rust target; its Cargo.lock matches titan_firmware_rs.
embassy_blinky.elf: embassy_blinky/src/main.rs embassy_blinky/Cargo.toml
	cd embassy_blinky && cargo build --release
	arm-none-eabi-strip --strip-debug -o $@ embassy_blinky/target/thumbv6m-none-eabi/release/embassy_blinky

# pico-sdk conformance programs (CMake, against titan_firmware's pico-sdk)
pico:
	cmake -G Ninja -S pico -B ../build/pico
	ninja -C ../build/pico
	for p in hello_uart hello_multicore flash_program hello_usb hello_usb_ram unique_id; do arm-none-eabi-strip --strip-debug -o pico/$$p.elf ../build/pico/$$p.elf; done

# titan_firmware's dual CDC stdio library in a small program (CMake, against titan_firmware)
c_dual_usb:
	cmake -G Ninja -S c_dual_usb -B ../build/c/c_dual_usb
	ninja -C ../build/c/c_dual_usb
	arm-none-eabi-strip --strip-debug -o c_dual_usb/c_dual_usb.elf ../build/c/c_dual_usb/c_dual_usb.elf

# The C IVCBoard2 modem without micro-ROS (CMake, against titan_firmware; its sources from the submodule's
# blake/ivc branch, extracted into build/c/src without touching the submodule's checkout)
IVC_REV ?= a3463bd9d745f57eb7cf0e8a4120df0a6f957e7f
c_ivc_modem:
	mkdir -p ../build/c/src
	git -C ../external/titan_firmware archive $(IVC_REV) talos/IVCBoard2 | tar -x -C ../build/c/src
	cmake -G Ninja -S c_ivc_modem -B ../build/c/c_ivc_modem
	ninja -C ../build/c/c_ivc_modem
	arm-none-eabi-strip --strip-debug -o c_ivc_modem/c_ivc_modem.elf ../build/c/c_ivc_modem/c_ivc_modem.elf

# The C IVCBoard2 in 25 kHz pinger mode, from the same sources
c_ivc_pinger:
	mkdir -p ../build/c/src
	git -C ../external/titan_firmware archive $(IVC_REV) talos/IVCBoard2 | tar -x -C ../build/c/src
	cmake -G Ninja -S c_ivc_pinger -B ../build/c/c_ivc_pinger
	ninja -C ../build/c/c_ivc_pinger
	arm-none-eabi-strip --strip-debug -o c_ivc_pinger/c_ivc_pinger.elf ../build/c/c_ivc_pinger/c_ivc_pinger.elf

# osu-uwrt/nano_ros's libnanoros in a small board program (CMake, against titan_firmware and a nano_ros checkout,
# neither modified): nano_ros_codegen (built in the checkout: cargo build --release in nano_ros_codegen) generates
# the topic code from ros.toml into ../build/c/nanoros_led/gen, with the emulator's .msg files as the ROS install.
# REPO_DIR (optional) is a titan_firmware checkout with its pico-sdk, by default ../external/titan_firmware.
NANO_ROS_DIR ?= $(HOME)/nano_ros
NANO_ROS_CODEGEN ?= $(NANO_ROS_DIR)/nano_ros_codegen/target/release/nano_ros_codegen
NANOROS_GEN = $(abspath ../build/c/nanoros_led/gen)
nanoros_led:
	mkdir -p $(NANOROS_GEN)
	cp nanoros_led/ros.toml $(NANOROS_GEN)/
	$(NANO_ROS_CODEGEN) $(NANOROS_GEN)/ros.toml --ros-prefix ../crates/sim_host/nanoros_msgs --user-msgs ../crates/sim_host/nanoros_msgs
	cmake -G Ninja -S nanoros_led -B ../build/c/nanoros_led/build -DNANO_ROS_DIR=$(abspath $(NANO_ROS_DIR)) \
		-DNANO_ROS_GEN_DIR=$(NANOROS_GEN)/generated $(if $(REPO_DIR),-DREPO_DIR=$(abspath $(REPO_DIR)))
	ninja -C ../build/c/nanoros_led/build
	arm-none-eabi-strip --strip-debug -o nanoros_led/nanoros_led.elf ../build/c/nanoros_led/build/nanoros_led.elf

.PHONY: all clean pico c_dual_usb c_ivc_modem c_ivc_pinger nanoros_led
