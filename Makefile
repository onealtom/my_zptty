ARCH            := arm
KERNEL_SRC_DIR  ?= /home/tyf/adi2/linux
ifeq ($(shell uname -m | sed -e s/arm.*/arm/),arm)
else
 CROSS_COMPILE  ?= arm-linux-gnueabihf-
endif

obj-m := zptty.o

all:
	make -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

clean:
	make -C $(KERNEL_SRC_DIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean

