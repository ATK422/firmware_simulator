# Rebuilds the hand-written test programs. The ELFs are committed so the tests do not need an ARM toolchain.
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
PROGRAMS = blink watchdog embassy_blinky

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
