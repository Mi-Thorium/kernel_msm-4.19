ifeq ($(shell test -d $(srctree)/arch/arm64/boot/dts/motorola-msm8937/.git; echo $$?),0)
MOTO8937_DT_GIT_VER := $(shell cd $(srctree)/arch/arm64/boot/dts/motorola-msm8937; /usr/bin/env PATH="$$PATH":/usr/bin:/usr/local/bin git rev-parse --short HEAD)
$(info - Motorola MSM8937 device tree git version: $(MOTO8937_DT_GIT_VER))
ccflags-y += -DMOTO8937_DT_GIT_VER=\"$(MOTO8937_DT_GIT_VER)\"
else
ccflags-y += -DMOTO8937_DT_GIT_VER=\"Unknown\"
endif

obj-$(CONFIG_MACH_MOTOROLA_MSM8937) += mach_detect.o
