.PHONY: all build flash monitor clean

all: flash

# Compile code
build:
	pio run

# Compile and flash code
flash:
	pio run --target upload

# Open serial monitor
monitor:
	pio device monitor

# Delete buildable files
clean:
	pio run --target clean
