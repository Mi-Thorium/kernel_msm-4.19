#ifndef _MOTOROLA_MSM8937_MACH_H
#define _MOTOROLA_MSM8937_MACH_H

enum motorola_msm8937_mach_types {
	MOTOROLA_MSM8937_MACH_UNKNOWN = 0,

	MOTOROLA_MSM8937_MACH_CEDRIC,
	MOTOROLA_MSM8937_MACH_HANNAH,
	MOTOROLA_MSM8937_MACH_JAMES,
	MOTOROLA_MSM8937_MACH_JETER,
	MOTOROLA_MSM8937_MACH_MONTANA,
	MOTOROLA_MSM8937_MACH_NORA,
	MOTOROLA_MSM8937_MACH_OWENS,
	MOTOROLA_MSM8937_MACH_PERRY,

	MOTOROLA_MSM8937_MACH_MAX,
};

#if IS_ENABLED(CONFIG_MACH_MOTOROLA_MSM8937)
extern const char *motorola_msm8937_mach_get_variant_str(void);
extern enum motorola_msm8937_mach_types motorola_msm8937_mach_get(void);
#else
static inline const char *motorola_msm8937_mach_get_variant_str(void) { return "Unknown" }
static inline enum motorola_msm8937_mach_types motorola_msm8937_mach_get(void) { return MOTOROLA_MSM8937_MACH_UNKNOWN; }
#endif

#endif
