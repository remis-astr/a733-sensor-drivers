# Out-of-tree build of the IMX477 VIN sensor driver.
#   make KDIR=/usr/src/linux-headers-<release>          (VIN headers from ./bsp)
#   make KDIR=... BSP=<allwinner-bsp checkout>          (full BSP tree instead)
# The VIN headers are not shipped in linux-headers-*-aw2511; ./bsp holds the
# subset these drivers include (symlink to ../imx327/bsp, see SOURCE.txt; copy
# with cp -rL for DKMS). They must match the BSP the
# kernel was built from (struct sensor_info is shared with vin_io.ko).
# DKMS builds through this Makefile too (dkms.conf).

KDIR ?= /lib/modules/$(shell uname -r)/build
# Resolved in the first make pass and exported, so kbuild's second pass
# (run from the kernel tree, where CURDIR differs) sees the same path.
BSP  ?= $(CURDIR)/bsp
export BSP

obj-m := imx477_mipi.o

ccflags-y += -I$(BSP)/drivers/vin -I$(BSP)/drivers/vin/modules/sensor -I$(BSP)/include

# VIN symbols live in vin_io.ko / vin_v4l2.ko; tolerate them missing from
# Module.symvers when the headers package does not list them.
export KBUILD_MODPOST_WARN := 1

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
