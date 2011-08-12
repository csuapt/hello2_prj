/* mt9d131 Camera
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MT9D131_H__
#define __MT9D131_H__

#include <media/v4l2-int-device.h>

#define MT9D131_I2C_ADDR	0x48 //(0x90 >> 1)
//#define MT9D131_I2C_ADDR	0x5d //(0xBA >> 1)

#define MT9D131_CLK_MAX		(60000000) /* 40MHz */
#define MT9D131_CLK_MIN		(8000000)  /* 6Mhz */

#define MT9D131_FLAG_PCLK_RISING_EDGE	(1 << 0)
#define MT9D131_FLAG_DATAWIDTH_8		(1 << 1) /* default width is 10 */

struct mt9d131_platform_data {
	char *master;
	int (*power_set) (struct v4l2_int_device *s, enum v4l2_power on);
	int (*ifparm) (struct v4l2_ifparm *p);
	int (*priv_data_set) (void *);
	u32 (*set_xclk) (struct v4l2_int_device *s, u32 xclkfreq);
	u32 flags;
};

#endif /* __MT9D131_H__ */
