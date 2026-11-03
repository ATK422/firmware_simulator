# Rebuilds the hand-written test programs. The ELFs are committed so the tests do not need an ARM toolchain.
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
PROGRAMS = blink watchdog sysreset profile embassy_blinky

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

.PHONY: all clean pico c_dual_usb
