obj-m := hyperpixel.o

# This should be the absolute path to a copy of the Raspberry Pi Linux kernel source
# for the version of Raspbian et. al. you are running.
#
KDIR ?= kernel
HOST ?= raspberrypi

default:
	ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- $(MAKE) -C $(KDIR) M=$$PWD

install:
	ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- $(MAKE) -C $(KDIR) M=$$PWD modules_install

push:
	-rm hyperpixel.ko

	# build the kernel module
	make

	# SCP into a running raspberrypi and get the files there
	scp hyperpixel.ko hyperpixel-touchscreen.dts root@$(HOST):

	# SSH into a running raspberrypi and put the built files into place
	ssh root@$(HOST) ' \
		[ -d /lib/modules/$(uname -r)/extra ] || mkdir -v /lib/modules/$(uname -r)/extra; \
		mv -v hyperpixel.ko /lib/modules/$(uname -r)/extra/hyperpixel.ko; \
		depmod; \
		dtc -@ -I dts -O dtb -o /boot/overlays/hyperpixel-touchscreen.dtbo hyperpixel-touchscreen.dts \
	'

clean:
	-rm *.cmd .*.cmd .tmp_versions
	-rm *.mod.*
	-rm *.o
	-rm *.ko
	-rm *.order *.symvers
