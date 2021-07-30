all:
	pio run

upload:
	pio run --target upload

clean:
	pio run --target clean

program:
	pio run --target program

monitor:
	pio run --target monitor

uploadfs:
	pio run --target uploadfs

update:
	pio update