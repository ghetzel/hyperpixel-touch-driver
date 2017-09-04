obj-m := hyperpixel.o

# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	-rm *.cmd .*.cmd .tmp_versions
	-rm *.mod.*
	-rm *.o
	-rm *.ko
	-rm *.order *.symvers
