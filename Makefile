BUILD = ../../uCee/build

all:
	mkdir -p build
	make -C ../micropython/teensy BUILD=$(BUILD) MEMZIP_DIR=../../uCee/memzip_files

upload:
	mkdir -p build
	make -C ../micropython/teensy BUILD=$(BUILD) MEMZIP_DIR=../../uCee/memzip_files upload

