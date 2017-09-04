obj-m := hyperpixel.o

# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- $(MAKE) -C $(KDIR) M=$$PWD

install:
	ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- $(MAKE) -C $(KDIR) M=$$PWD modules_install

clean:
	-rm *.cmd .*.cmd .tmp_versions
	-rm *.mod.*
	-rm *.o
	-rm *.ko
	-rm *.order *.symvers
