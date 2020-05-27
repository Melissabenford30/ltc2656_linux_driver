.PHONY: install

obj-m += ltc2656.o
KDIR :=  ~/linux
all:
	make -C $(KDIR) M=$(shell pwd) modules

install:
	make -C $(KDIR) M=$(shell pwd) modules_install

clean:
	make -C $(KDIR) M=$(shell pwd) clean

