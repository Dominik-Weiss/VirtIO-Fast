obj-m += virtio_fast_single.o
obj-m += virtio_fast_single_ack.o
obj-m += virtio_fast_multi.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
