#include <linux/export.h>
#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <motorola-msm8937/mach.h>

typedef struct motorola_msm8937_mach_info {
	const char *codename;
} motorola_msm8937_mach_info_t;

static const motorola_msm8937_mach_info_t motorola_msm8937_mach_table[MOTOROLA_MSM8937_MACH_MAX] = {
	[MOTOROLA_MSM8937_MACH_CEDRIC] = {"cedric"},
	[MOTOROLA_MSM8937_MACH_HANNAH] = {"hannah"},
	[MOTOROLA_MSM8937_MACH_JAMES] = {"james"},
	[MOTOROLA_MSM8937_MACH_JETER] = {"jeter"},
	[MOTOROLA_MSM8937_MACH_MONTANA] = {"montana"},
	[MOTOROLA_MSM8937_MACH_NORA] = {"nora"},
	[MOTOROLA_MSM8937_MACH_OWENS] = {"owens"},
	[MOTOROLA_MSM8937_MACH_PERRY] = {"perry"},
};

const char *motorola_msm8937_variant_str = NULL;
static enum motorola_msm8937_mach_types saved_mach = MOTOROLA_MSM8937_MACH_UNKNOWN;
static struct kobject *motorola_msm8937_mach_kobj;

enum motorola_msm8937_mach_types motorola_msm8937_mach_get(void) {
	return saved_mach;
}
EXPORT_SYMBOL(motorola_msm8937_mach_get);

const char *motorola_msm8937_mach_get_variant_str(void) {
	if (!IS_ERR_OR_NULL(motorola_msm8937_variant_str))
		return motorola_msm8937_variant_str;
	else if (saved_mach)
		return motorola_msm8937_mach_table[saved_mach].codename;
	else
		return "Unknown";
}
EXPORT_SYMBOL(motorola_msm8937_mach_get_variant_str);

static ssize_t motorola_msm8937_mach_device_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", motorola_msm8937_mach_table[saved_mach].codename);
}

static struct kobj_attribute motorola_msm8937_mach_device_attr = {
	.attr = {
		.name = "device",
		.mode = 0444,
	},
	.show = motorola_msm8937_mach_device_show,
};

static ssize_t motorola_msm8937_mach_variant_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", motorola_msm8937_mach_get_variant_str());
}

static struct kobj_attribute motorola_msm8937_mach_variant_attr = {
	.attr = {
		.name = "variant",
		.mode = 0444,
	},
	.show = motorola_msm8937_mach_variant_show,
};

static bool motorola_msm8937_mach_detect(void) {
	int i, rc;
	char of_compatible[32];
	char of_compatible_prefix[14]; // length of "qcom,msm89XX-" + 1

	if (of_machine_is_compatible("qcom,msm8917-moto"))
		strlcpy(of_compatible_prefix, "qcom,msm8917-", sizeof(of_compatible_prefix));
	else if (of_machine_is_compatible("qcom,msm8937-moto"))
		strlcpy(of_compatible_prefix, "qcom,msm8937-", sizeof(of_compatible_prefix));
	else if (of_machine_is_compatible("qcom,msm8940-moto"))
		strlcpy(of_compatible_prefix, "qcom,msm8940-", sizeof(of_compatible_prefix));
	else {
		pr_err("%s: Not Motorola MSM8937 machine\n", __func__);
		return false;
	}

	for (i=1; i<MOTOROLA_MSM8937_MACH_MAX; ++i) {
		snprintf(of_compatible, sizeof(of_compatible), "%s%s", of_compatible_prefix, motorola_msm8937_mach_table[i].codename);
		rc = of_machine_is_compatible(of_compatible);
		if (rc) {
			saved_mach = i;
			return true;
		}
	}

	pr_err("%s: Unrecognized Motorola MSM8937 machine\n", __func__);
	return false;
}

static int __init motorola_msm8937_mach_detect_init(void) {
	int rc;

	// Print version
	pr_info("%s: Motorola MSM8937 device tree git version: %s\n", __func__, MOTO8937_DT_GIT_VER);

	// Detect
	rc = motorola_msm8937_mach_detect();
	if (!rc) {
		pr_err("%s: Could not detect Motorola MSM8937 machine\n", __func__);
		goto fail;
	}

	// Read variant string from devicetree
	rc = of_property_read_string(of_find_node_by_path("/"), "motorola,variant", &motorola_msm8937_variant_str);
	if (rc) {
		pr_err("%s: Could not read variant string from devicetree", __func__);
	}

	// Print the current machine
	pr_info("%s: Current machine: %s (%s)\n", __func__,
			motorola_msm8937_mach_table[saved_mach].codename,
			motorola_msm8937_variant_str);

	// Create sysfs dir
	motorola_msm8937_mach_kobj = kobject_create_and_add("motorola-msm8937-mach", NULL);
	if (!motorola_msm8937_mach_kobj) {
		pr_err("%s: Failed to create sysfs dir\n", __func__);
		goto fail;
	}

	// Create sysfs files
	rc = sysfs_create_file(motorola_msm8937_mach_kobj,
			&motorola_msm8937_mach_device_attr.attr);
	if (rc < 0)
		pr_err("%s: Failed to create sysfs file device, rc=%d\n", __func__, rc);

	rc = sysfs_create_file(motorola_msm8937_mach_kobj,
			&motorola_msm8937_mach_variant_attr.attr);
	if (rc < 0)
		pr_err("%s: Failed to create sysfs file variant, rc=%d\n", __func__, rc);

	return 0;
fail:
	return rc;
}
core_initcall(motorola_msm8937_mach_detect_init);
