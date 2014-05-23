export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabi-

obj-m := mpu_6000.o
KDIR := /home/jimmy/Development/BBBKernel/kernel/kernel
PWD := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean