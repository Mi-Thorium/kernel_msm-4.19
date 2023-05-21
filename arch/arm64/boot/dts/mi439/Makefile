dtbo-$(CONFIG_MACH_XIAOMI_SDM439) += \
	sdm439-qrd-olive-overlay.dtbo \
	sdm439-qrd-pine-overlay.dtbo

sdm439-qrd-olive-overlay.dtbo-base := sdm439-olive.dtb \
        msm8937-interposer-sdm439-olive.dtb

sdm439-qrd-pine-overlay.dtbo-base := sdm439-pine.dtb \
	msm8937-interposer-sdm439-pine.dtb

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
