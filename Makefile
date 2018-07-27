always	:= ./dt/lte800.dtb

ARCH            := arm
KERNEL_SRC_DIR  ?= /home/tyf/adi2/linux

CROSS_COMPILE  ?= arm-linux-gnueabihf-

vptty-y := zptty.o globalmem.o
obj-m := vptty.o 

all:
	make -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

clean:
	make -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean

