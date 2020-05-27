.PHONY: install

obj-m += rtc-ds3232.o
KDIR :=  ~/linux
all:
	make -C $(KDIR) M=$(shell pwd) modules

install:
	make -C $(KDIR) M=$(shell pwd) modules_install

clean:
	make -C $(KDIR) M=$(shell pwd) clean

