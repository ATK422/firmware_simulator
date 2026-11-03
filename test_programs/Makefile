# Rebuilds the hand-written test programs. The ELFs are committed so the tests do not need an ARM toolchain.
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
PROGRAMS = blink watchdog sysreset lockup profile embassy_blinky

all: $(PROGRAMS:=.elf)

%.elf: %.S link.ld
	$(AS) -mcpu=cortex-m0plus -o $*.o $<
	$(LD) -T link.ld -o $@ $*.o
	rm -f $*.o

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

.PHONY: all clean pico c_dual_usb c_ivc_modem c_ivc_pinger
