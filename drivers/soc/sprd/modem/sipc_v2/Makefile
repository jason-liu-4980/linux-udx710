obj-$(CONFIG_SPRD_SIPC_MBOX_CLIENT) += udx710_smsg_mbox.o

obj-$(CONFIG_SPRD_SIPC)     += sipc.o smsg.o smem.o sbuf.o sblock.o sipc_debugfs.o

obj-$(CONFIG_SPRD_SIPC_SPIPE)	+= spipe.o

obj-$(CONFIG_SPRD_SIPC_SPOOL)	+= spool.o

obj-$(CONFIG_SPRD_SIPC_ZERO_COPY_SIPX) += sipx.o

# Standalone object validation only; ignored for normal in-tree builds.
ifneq ($(KBUILD_EXTMOD),)
ccflags-y += -I$(src)/../../../../../include -DCONFIG_SPRD_SIPC=1 -DCONFIG_SPRD_SIPC_V2=1 -DCONFIG_SPRD_SIPC_MBOX_CLIENT=1
endif
