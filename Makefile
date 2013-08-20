CROSS_COMPILE=arm-linux-gnueabihf-
KDIR=../build/a13_defconfig-linux

obj-m += gslx680_ts.o

all:
	make -C ${KDIR} ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} M=$(PWD) modules

clean:
	make -C ${KDIR} ARCH=arm CROSS_COMPILE=${CROSS_COMPILE} M=$(PWD) clean
