#ifndef SVMP_COMMON_H_NDNXIOHL
#define SVMP_COMMON_H_NDNXIOHL
#include <linux/fb.h>
struct fbdata {
	int fd;
	struct fb_var_screeninfo var;
	unsigned char *base;
	size_t len; // size of mmaped area
};

#endif /* end of include guard: SVMP_COMMON_H_NDNXIOHL */

