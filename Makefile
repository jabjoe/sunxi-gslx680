CROSS_COMPILE=arm-linux-gnueabihf-

obj-m += gslx680_ts.o

all:
	make -C ../build/a13_defconfig-linux ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} M=$(PWD) modules

clean:
	rm gslx680.o
	rm gslx680.ko
