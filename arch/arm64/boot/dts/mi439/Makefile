ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

dtbo-$(CONFIG_MACH_XIAOMI_SDM439) += \
	sdm439-qrd-olive-overlay.dtbo \
	sdm439-qrd-pine-overlay.dtbo

sdm439-qrd-olive-overlay.dtbo-base := sdm439-olive.dtb \
        msm8937-interposer-sdm439-olive.dtb

sdm439-qrd-pine-overlay.dtbo-base := sdm439-pine.dtb \
	msm8937-interposer-sdm439-pine.dtb

endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)

$(obj)/%.dtbo:$(src)/../../../../arm64/boot/dts/vendor/qcom/xiaomi/%.dts FORCE
	$(call if_changed_dep,dtc)
	$(call if_changed,dtbo_verify)

dtbs: $(addprefix $(obj)/,$(dtb-y)) $(addprefix $(obj)/,$(dtbo-y))
clean-files	:= *.dtb *.dtbo
