obj-m := hyperpixel.o

# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- $(MAKE) -C $(KDIR) M=$$PWD

install:
	ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- $(MAKE) -C $(KDIR) M=$$PWD modules_install

push:
	-rm hyperpixel.ko
	make
	scp hyperpixel.ko root@pi:hyperpixel-touch-driver/hyperpixel.ko
	ssh root@pi 'cd hyperpixel-touch-driver && KDIR=/root/raspi-linux make install'

clean:
	-rm *.cmd .*.cmd .tmp_versions
	-rm *.mod.*
	-rm *.o
	-rm *.ko
	-rm *.order *.symvers
