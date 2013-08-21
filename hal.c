#include <hardware/gps.h>
#include "gps.h"

static const GpsInterface *module_get_gps_interface(struct gps_device_t *dev)
{
	return &hal_gps_interface;
}

extern struct hw_module_t HAL_MODULE_INFO_SYM; /* forward decl */

static struct gps_device_t module_device = {
	.common = {
		.tag = HARDWARE_DEVICE_TAG,
		.version = 0,
		.module = &HAL_MODULE_INFO_SYM,
	},
	.get_gps_interface = module_get_gps_interface,
};

static int module_open(
	const struct hw_module_t *module,
	char const *name,
	struct hw_device_t **device
)
{
    *device = (struct hw_device_t*)&module_device;
    return 0;
}

static struct hw_module_methods_t module_methods = {
	.open = module_open
};

struct hw_module_t HAL_MODULE_INFO_SYM = {
	.tag = HARDWARE_MODULE_TAG,
	.version_major = 1,
	.version_minor = 0,
	.id = GPS_HARDWARE_MODULE_ID,
	.name = "GPS Module for Qualcomm RPC",
	.author = "Christopher Lais <chris+android@zenthought.org>",
	.methods = &module_methods,
};
