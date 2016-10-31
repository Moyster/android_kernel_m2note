# WMT HAL driver for MT6735
ccflags-y += \
	-I$(src)/include \
	-I$(src)/../linux/include \
	-I$(src)/../include \
	-I$(src)/../../common_detect

ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/mach
ccflags-y += -I$(srctree)/include/mach
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(subst ",,$(CONFIG_ARCH_MTK_PROJECT)")/dct/dct

subdir-ccflags-y += -D MTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT

ifeq ($(CONFIG_MTK_CONN_LTE_IDC_SUPPORT),y)
	subdir-ccflags-y += -DWMT_IDC_SUPPORT=1
else
	subdir-ccflags-y += -DWMT_IDC_SUPPORT=0
endif

ccflags-y += -DCONFIG_MTK_GPIO_LEGACY=1

obj-y += mtk_wcn_consys_hw.o
obj-y += wmt_plat_alps.o
