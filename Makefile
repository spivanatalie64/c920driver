obj-m += c920virtual.o

KVER ?= $(shell uname -r)
KDIR := /lib/modules/$(KVER)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	sudo install -m 644 c920virtual.ko /lib/modules/$(KVER)/extra/
	sudo depmod -a
	sudo modprobe -r v4l2loopback || true
	sudo modprobe c920virtual
