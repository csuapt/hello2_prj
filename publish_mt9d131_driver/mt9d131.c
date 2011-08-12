/*
 * drivers/media/video/mt9d131.c
 *
 * Aptina MT9D131/A2010 soc sensor driver
 *
 * Copyright (C) 2011 Aptina Imaging
 * 
 * Leverage mt9p031.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/videodev2.h>
#include <linux/sysfs.h>

#include <media/mt9d131.h>
#include <media/v4l2-int-device.h>
#include <media/v4l2-chip-ident.h>

#define MT9D131_DEBUG_REG_ACCESS
typedef enum {
	NO_DEBUG=0x0,
	DEBUG_PIXEL=(1<<1),
	DEBUG_FRAME_SIZE=(1<<2),
	DEBUG_ENUM_FRAME_SIZE=(1<<3),
	DEBUG_ENUM_FRAME_INTERVAL=(1<<4),
	DEBUG_INIT_CAM=(1<<5),
	DEBUG_FRAME_FORMAT=(1<<6),
}debug_levle_t;
static int debug_level;

/************************************************************************
			macro
************************************************************************/
// Macro to configure I2c level shifter. Use only for MT9D131 Headboards 
// from Aptina; not required for Leopard Imaging or elsewise. 
//#define MT9D131_HEADBOARD
#undef MT9D131_HEADBOARD

#define MT9D131_CHIP_ID						0x1519
#define MT9D131_MAX_WIDTH					1600
#define MT9D131_MAX_HEIGHT					1200
#define MT9D131_DEFAULT_WIDTH				800
#define MT9D131_DEFAULT_HEIGHT				600

/* FPS Capabilities */
#define MT9D131_MIN_FPS						2
#define MT9D131_DEF_FPS						30
#define MT9D131_MAX_FPS						50

#define MT9D131_XCLK_NOM_1					12000000
#define MT9D131_XCLK_NOM_2					24000000

/* Analog gain values */
#define MT9D131_EV_MIN_GAIN					0
#define MT9D131_EV_MAX_GAIN					47
#define MT9D131_EV_DEF_GAIN					24
#define MT9D131_EV_GAIN_STEP				1

/* Exposure time values */
#define MT9D131_MIN_EXPOSURE				15000
#define MT9D131_MAX_EXPOSURE				128000
#define MT9D131_DEF_EXPOSURE				33000
#define MT9D131_EXPOSURE_STEP				100

/************************************************************************
			Register Address
************************************************************************/
#define REG_MT9D131_CHIP_VERSION			0x00

struct mt9d131_frame_size {
	u16 width;
	u16 height;
};

struct mt9d131_priv {
	struct mt9d131_platform_data  *pdata;
	struct v4l2_int_device  *v4l2_int_device;
	struct i2c_client  *client;
	struct v4l2_pix_format  pix;
	struct v4l2_fract timeperframe;
	unsigned long xclk_current;
	int fps;
	int scaler;
	int ver;
	int  model;
	u32  flags;
	u16 mWidth;
	u16 mHeight;
/* for flags */
#define INIT_DONE  (1<<0)
};
struct mt9d131_priv sysPriv;

static const struct v4l2_fmtdesc mt9d131_formats[] = {
	{
		.description = "standard YUV 4:2:2",
		.pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
		.description = "standard UYVY 4:2:2",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
};

static const unsigned int mt9d131_num_formats = ARRAY_SIZE(mt9d131_formats);

/**************************supported sizes******************************/
const static struct mt9d131_frame_size   mt9d131_supported_framesizes[]={
	{  80,  60 },
	{  160, 120 },
	{  176, 144 },
	{  320, 240 },
	{  352, 288 },
	{  400, 300 },
	{  640, 480 },
	{  800, 600 },
	{ 1280, 720 },
	{ 1280, 960 },
	{ 1280, 1024},
	{ 1600, 1200 },
};

enum mt9d131_image_size {
	VGA_BIN_30FPS,
	HDV_720P_30FPS,
	//HDV_720P_60FPS,
	//HDV_720P_60FPS_LVB,
	HDV_1080P_30FPS,
	MT9D131_THREE_MP,
	MT9D131_FIVE_MP,
};

enum mt9d131_image_size mt9d131_current_size_index;

const struct v4l2_fract mt9d131_frameintervals_fast[] = {
	{ .numerator = 24, .denominator = 2 },
	{ .numerator = 24, .denominator = 2 },
	{ .numerator = 24, .denominator = 2 },
	{ .numerator = 24, .denominator = 2 },
	{ .numerator = 24, .denominator = 2 },
	{ .numerator = 24, .denominator = 2 },
	{ .numerator = 24, .denominator = 2 },
	{ .numerator = 24, .denominator = 2 },
	{ .numerator = 24, .denominator = 2 },
};

const struct v4l2_fract mt9d131_frameintervals_slow[] = {
	{ .numerator = 8, .denominator = 2 },
	{ .numerator = 8, .denominator = 2 },
	{ .numerator = 8, .denominator = 2 },
	{ .numerator = 8, .denominator = 2 },
	{ .numerator = 8, .denominator = 2 },
	{ .numerator = 8, .denominator = 2 },
	{ .numerator = 8, .denominator = 2 },
	{ .numerator = 8, .denominator = 2 },
	{ .numerator = 8, .denominator = 2 },
};

static struct vcontrol{
	struct v4l2_queryctrl qc;
	int current_value;
}mt9d131_video_control[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = MT9D131_MIN_EXPOSURE,
			.maximum = MT9D131_MAX_EXPOSURE,
			.step = MT9D131_EXPOSURE_STEP,
			.flags=V4L2_CTRL_FLAG_DISABLED,   //<--disabled for the initial version
			.default_value = MT9D131_DEF_EXPOSURE,
		},
		.current_value = MT9D131_DEF_EXPOSURE,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Analog Gain",
			.minimum = MT9D131_EV_MIN_GAIN,
			.maximum = MT9D131_EV_MAX_GAIN,
			.step = MT9D131_EV_GAIN_STEP,
			.flags=V4L2_CTRL_FLAG_DISABLED,   //<--disabled for the initial version
			.default_value = MT9D131_EV_DEF_GAIN,
		},
		.current_value = MT9D131_EV_DEF_GAIN,
	},
};

#ifdef MT9D131_HEADBOARD
/**
 * mt9d131_config_PCA9543A - configure on-board I2c level-shifter PCA9543A of MT9D131 Headboards from Aptina
 * @client: pointer to i2c client
 * Configures the level shifter to enable channel 0 
 */
static int 
mt9d131_config_PCA9543A(const struct i2c_client *client)
{
	struct	i2c_msg msg;
	int		ret;
	u8		buf= 0x21;
	
	msg.addr  = (0xE6 >> 1);	//slave address of PCA9543A
	msg.flags = 0;
	msg.len   = 1;
	msg.buf   = &buf;
	
	ret = i2c_transfer(client->adapter, &msg, 1);

	return 0;
		
}
#endif //MT9D131_HEADBOARD

/**
 * mt9d131_reg_read - read resgiter value
 * @client: pointer to i2c client
 * @command: register address
 */
static int 
mt9d131_reg_read(const struct i2c_client *client, u16 command, u16 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = command & 0xff;// 8-bit/ byte addressable register

	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 1;
	msg[0].buf   = buf ;
	ret = i2c_transfer(client->adapter, &msg[0], 1);
	
	if(ret >= 0) {
		msg[1].addr  = client->addr;
		msg[1].flags = I2C_M_RD; //1
		msg[1].len   = 2;
		msg[1].buf   = buf;
		ret = i2c_transfer(client->adapter, &msg[1], 1);
	}
	//if return value of this function is < 0, it mean error.
	//else, under 16bit is valid data. 
	if(ret<0) printk("mt9d131_reg_read() failed: ret=%d, reg_addr=0x%x, reg_val=0x%x\n",ret,command,*val);

	if(ret >= 0) {
		*val = 0;
		*val = buf[1] + (buf[0] << 8);
		return 0;
	}
	
	v4l_err(client, "read from offset 0x%x error %d", command, ret);
	return ret;
}

/**
 * mt9d131_reg_write - read resgiter value
 * @client: pointer to i2c client
 * @command: register address
 * @data: value to be written 
 */ 
static int 
mt9d131_reg_write(const struct i2c_client *client, u16 command, u16 data)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = command & 0xff;// 16-bit/ byte addressable register
	data = swab16(data);
	memcpy(buf + 1, &data,    2);

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = buf;
		
	//i2c_transfer return message length, but this function should return 0 if correct case
	ret = i2c_transfer(client->adapter, &msg, 1);

	if(ret<0) printk("mt9d131_reg_write() failed: ret=%d, reg_addr=0x%x, reg_val=0x%x\n",ret,command,data);

	if (ret >= 0) ret = 0;

	return ret;
}

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE) return -EDOM;

	for (i = (ARRAY_SIZE(mt9d131_video_control) - 1); i >= 0; i--)
		if (mt9d131_video_control[i].qc.id == id) break;

	if (i < 0) i = -EINVAL;

	return i;
}

/**
 * mt9d131_calc_size - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static int 
mt9d131_calc_size(unsigned int request_width, unsigned int request_height)
{
	int i;
	unsigned long requested_pixels = request_width * request_height;

	for(i = 0; i < ARRAY_SIZE(mt9d131_supported_framesizes); i++) {
		if(mt9d131_supported_framesizes[i].height * mt9d131_supported_framesizes[i].width >= requested_pixels) 
			return i;
	}
	//couldn't find a match, return the max size as a default
	return (ARRAY_SIZE(mt9d131_supported_framesizes) - 1);
}


/* mt9d131_set_output_size - set the output size upon request specifed by "width" and "height"
 * @c: i2c client driver structure
 * @width: width of output
 * @height: width of output*/
static int
mt9d131_setup_sensor_output(const struct i2c_client *client, u16 width, u16 height)
{
	int	ret=0;
	u16	reg_val;

	if(debug_level&DEBUG_INIT_CAM) printk("entering mt9d131_setup_sensor_output()... \n");

	ret |=mt9d131_reg_write(client, 0x00F0, 0x0001); ///change to page 1
	ret |=mt9d131_reg_write(client, 0x00C6, 0x2703);
	ret |=mt9d131_reg_write(client, 0x00C8, width);

	ret |=mt9d131_reg_write(client, 0x00C6, 0x2705);
	ret |=mt9d131_reg_write(client, 0x00C8, height);
	if(ret) printk("mt9d131_setup_sensor_output() failed at width and height setting: ret=%d\n",ret);

	if((width > MT9D131_DEFAULT_WIDTH) && (height > MT9D131_DEFAULT_HEIGHT)){ //disable binning
		if(debug_level&DEBUG_FRAME_SIZE) printk("disable binning for %dx%d>%dx%d\n",
												width, height,MT9D131_DEFAULT_WIDTH,MT9D131_DEFAULT_HEIGHT);
		ret |=mt9d131_reg_write(client, 0x00F0, 0x0000); ///change to page 0
		ret |=mt9d131_reg_write(client, 0x0021, 0x0400); //disable binning

		ret |=mt9d131_reg_write(client, 0x00F0, 0x0001); ///change to page 1

		ret |=mt9d131_reg_write(client, 0x00C6, 0x2729); //change the cropping to full size,i.e., 1600
		ret |=mt9d131_reg_write(client, 0x00C8, MT9D131_MAX_WIDTH);

		ret |=mt9d131_reg_write(client, 0x00C6, 0x272D); ///change the cropping to full size,i.e., 1200
		ret |=mt9d131_reg_write(client, 0x00C8, MT9D131_MAX_HEIGHT);
		if(ret) printk("mt9d131_setup_sensor_output() failed at disabling binning setting: ret=%d\n",ret);
	}else{ //enable binning
		if(debug_level&DEBUG_FRAME_SIZE) printk("enable binning for %dx%d<= %dx%d\n",
												width, height,MT9D131_DEFAULT_WIDTH,MT9D131_DEFAULT_HEIGHT);
		ret |=mt9d131_reg_write(client, 0x00F0, 0x0000); ///change to page 0
		ret |=mt9d131_reg_write(client, 0x0021, 0x8400); //enable binning

		ret |=mt9d131_reg_write(client, 0x00F0, 0x0001); ///change to page 1
		ret |=mt9d131_reg_write(client, 0x00C6, 0x2729); //change the cropping to default size,i.e., 800
		ret |=mt9d131_reg_write(client, 0x00C8, MT9D131_DEFAULT_WIDTH);

		ret |=mt9d131_reg_write(client, 0x00C6, 0x272D); ///change the cropping to default size,i.e., 600
		ret |=mt9d131_reg_write(client, 0x00C8, MT9D131_DEFAULT_HEIGHT);
		if(ret) printk("mt9d131_setup_sensor_output() failed at enabling binning setting: ret=%d\n",ret);
	}
	mdelay(200);  //addd some delay to let it setttle down
	ret |=mt9d131_reg_write(client, 0x00C6, 0xA103); //refresh
	ret |=mt9d131_reg_write(client, 0x00C8, 0x0005);

	if(debug_level&DEBUG_FRAME_SIZE){ 
		ret |=mt9d131_reg_write(client, 0x00F0, 0x0000); ///change to page 0
		ret |=mt9d131_reg_read(client, 0x21, &reg_val);
		printk("READ_MODE_A(0x21)=0x%x\n",reg_val);

		ret |=mt9d131_reg_write(client, 0x00F0, 0x0001); ///change to page 1
		ret |=mt9d131_reg_write(client, 0x00C6, 0x2729);
		ret |=mt9d131_reg_read(client, 0xC8, &reg_val);
		printk("MODE_CROP_X1_A=%d\n",reg_val);

		ret |=mt9d131_reg_write(client, 0x00C6, 0x272d);
		ret |=mt9d131_reg_read(client, 0xC8, &reg_val);
		printk("MODE_CROP_Y1_A=%d\n",reg_val);
	}

	if(debug_level&DEBUG_INIT_CAM) printk("finished mt9d131_setup_sensor_output():ret=%d\n",ret);

	return (ret>= 0? 0:-EIO);
}

/* mt9d131_init_camera - initialize camera settings in context A
 * @client: pointer to i2c client
 * Initialize camera settings */ 
static int 
mt9d131_init_camera(const struct i2c_client *client)
{
	int ret=0;
	struct mt9d131_priv		*priv=i2c_get_clientdata(client);
	struct v4l2_pix_format	*pix =&priv->pix;

	//reset the chip which defaults to context A
	ret |=mt9d131_reg_write(client, 0x00F0, 0x0);
	ret |=mt9d131_reg_write(client, 0x65,   0xA000);

	ret |=mt9d131_reg_write(client, 0x00F0, 0x1);
	ret |=mt9d131_reg_write(client, 0xC3,   0x0501);

	ret |=mt9d131_reg_write(client, 0x00F0, 0x0);
	ret |=mt9d131_reg_write(client, 0x0D,   0x0021);
	ret |=mt9d131_reg_write(client, 0x0D,   0x0000);

	ret |=mt9d131_reg_write(client, 0x00F0, 0x1);
	mdelay(100);
	switch(pix->pixelformat ){//set pixel format
	case V4L2_PIX_FMT_UYVY:
		if(debug_level&DEBUG_FRAME_FORMAT) printk("set ""MODE_OUTPUT_FORMAT_A"" TO YCbCr(Cb-Y-Cr-Y)\n");
		ret |=mt9d131_reg_write(client, 0x00C6, 0xA77D);
		ret |=mt9d131_reg_write(client, 0x00C8, 0x0000);
		ret |=mt9d131_reg_write(client, 0x00C6, 0xA103);
		ret |=mt9d131_reg_write(client, 0x00C8, 0x0005);
		break;
	case V4L2_PIX_FMT_YUYV:
	default:
		if(debug_level&DEBUG_FRAME_FORMAT) printk("set ""MODE_OUTPUT_FORMAT_A"" TO YCbCr(Y-Cb-Y-Cr)\n");
		ret |=mt9d131_reg_write(client, 0x00C6, 0xA77D);
		//ret |=mt9d131_reg_write(client, 0x00C8, 0x0000); //<--actually Cb-Y-Cr-Y
		//ret |=mt9d131_reg_write(client, 0x00C8, 0x0001); //<--actually Cr-Y-Cb-Y
		ret |=mt9d131_reg_write(client, 0x00C8, 0x0002); //<--actually Y-Cb-Y-Cr
		//ret |=mt9d131_reg_write(client, 0x00C8, 0x0003); //<--actually Y-Cr-Y-Cb
		ret |=mt9d131_reg_write(client, 0x00C6, 0xA103);
		ret |=mt9d131_reg_write(client, 0x00C8, 0x0005);
		break;
	}

	//Enable PLL 24Mhz Ext/80Mhz Out read-back
	ret |=mt9d131_reg_write(client, 0x00F0, 0x00);
	ret |=mt9d131_reg_write(client, 0x66, 0x500B);
	ret |=mt9d131_reg_write(client, 0x67, 0x200);
	ret |=mt9d131_reg_write(client, 0x65, 0xA000);
	ret |=mt9d131_reg_write(client, 0x65, 0x2000);

	priv->mWidth=0;
	priv->mHeight=0;
	ret|=mt9d131_setup_sensor_output(client,(u16)MT9D131_DEFAULT_WIDTH,(u16)MT9D131_DEFAULT_HEIGHT); 

	priv->mWidth =(u16)MT9D131_DEFAULT_WIDTH;
	priv->mHeight=(u16)MT9D131_DEFAULT_HEIGHT;
	return ret;
}

/**
 * mt9d131_detect - Detect if an mt9d131 is present, and if so which revision
 * @client: pointer to the i2c client driver structure
 *
 * Returns a negative error number if no device is detected
 */
static int 
mt9d131_detect(struct i2c_client *client)
{
	struct mt9d131_priv *priv = i2c_get_clientdata(client);
	const char			*devname;
	u16					 chipid;
	
	if (!client) return -ENODEV;
	
	if(mt9d131_reg_read(client, REG_MT9D131_CHIP_VERSION, &chipid)) return -ENODEV;
		
	if(chipid == MT9D131_CHIP_ID) {
		devname = "mt9d131";
		priv->model = V4L2_IDENT_MT9D131;
		dev_info(&client->dev, "Sensor %s is detected(chipID=0x%04x)\n",devname,chipid);
		return 0;
	}
			
	dev_err(&client->dev, "Product ID error %04x\n", chipid); return -ENODEV;
}

/**
 * mt9d131_set_exposure_time - sets exposure time per input value
 * @exp_time: exposure time to be set on device
 * @client: pointer to standard i2c client
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 *
 * If the requested exposure time is within the allowed limits, the HW
 * is configured to use the new exposure time, and the video_controls
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
static int 
mt9d131_set_exposure_time(u32 exp_time, struct i2c_client *client, struct vcontrol *lvc)
{
	int ret = -EINVAL;
	
	printk("mt9d131_set_exposure_time() is not needed for SOC sensor when AE/AWB is on\n");
	return ret;	
}

/**
 * mt9d131_set_gain - sets sensor analog gain per input value
 * @lineargain: analog gain value index to be set on device
 * @client: pointer to standard i2c client
 * @lvc: pointer to V4L2 analog gain entry in video_controls array
 *
 * If the requested analog gain is within the allowed limits, the HW
 * is configured to use the new gain value, and the video_controls
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
static int 
mt9d131_set_gain(u16 lineargain, struct i2c_client *client, struct vcontrol *lvc)
{
	int ret= -EINVAL;
		
	printk("mt9d131_set_gain() is not needed for SOC sensor when AE/AWB is on\n");
	return ret;
}

/************************************************************************
			v4l2_ioctls
************************************************************************/
/**
 * mt9d131_v4l2_int_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int 
mt9d131_v4l2_int_s_power(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct mt9d131_priv *priv = s->priv;
	struct i2c_client *client = priv->client;
	static int sensor_is_ready=0;
	
	int ret;

	switch (power) {
	case V4L2_POWER_STANDBY:
		/* FALLTHROUGH */
		if(debug_level&DEBUG_INIT_CAM) printk("mt9d131_v4l2_int_s_power()-->V4L2_POWER_STANDBY is granted! sensor_is_ready=%d\n",
												sensor_is_ready);
		break;

	case V4L2_POWER_OFF:
		if(debug_level&DEBUG_INIT_CAM) printk("mt9d131_v4l2_int_s_power()-->V4L2_POWER_OFF is trying.....\n");
		if(--sensor_is_ready >0){
			if(debug_level&DEBUG_INIT_CAM) printk("mt9d131_v4l2_int_s_power()-->V4L2_POWER_OFF is not granted! sensor_is_ready=%d\n",
												sensor_is_ready);
			break;	
		}
		ret = priv->pdata->power_set(s, power);
		if (ret < 0) {
			dev_err(&client->dev, "Unable to set target board power" " state (OFF/STANDBY)\n");
			if(debug_level&DEBUG_INIT_CAM) printk("mt9d131_v4l2_int_s_power()-->V4L2_POWER_OFF is failed,sensor_is_ready=%d\n",
												sensor_is_ready);
			return ret;
		}else{
			if(debug_level&DEBUG_INIT_CAM) printk("mt9d131_v4l2_int_s_power()-->V4L2_POWER_OFF is successful,sensor_is_ready=%d\n",
													sensor_is_ready);
		}
		break;
	case V4L2_POWER_ON:
		if(debug_level&DEBUG_INIT_CAM) printk("mt9d131_v4l2_int_s_power()-->V4L2_POWER_ON is trying.....sensor_is_ready=%d\n",
											sensor_is_ready);

		if(sensor_is_ready) {
			sensor_is_ready=1;
			if(debug_level&DEBUG_INIT_CAM) printk("mt9d131_v4l2_int_s_power()-->V4L2_POWER_ON is not needed since it is ready!.\n");
			break;
		}

		ret = priv->pdata->power_set(s, power);
		if (ret < 0) {
			dev_err(&client->dev, "Unable to set target board power " "state (ON)\n");
			return ret;
		}
		if(debug_level&DEBUG_INIT_CAM) printk("mt9d131_v4l2_int_s_power()-->V4L2_POWER_ON is successful\n");

#ifdef MT9D131_HEADBOARD
		//configure i2c level shifter on mt9d131 head-board, no need for Leopard module
		mt9d131_config_PCA9543A(client);	
#endif	
		if (!(priv->flags & INIT_DONE)) {
			ret = mt9d131_detect(client);
			if (ret < 0) {
				dev_err(&client->dev, "Unable to detect sensor\n");
				return ret;
			}
			priv->flags |= INIT_DONE;
		}

		ret = mt9d131_init_camera(client);
		if (ret < 0) {
			dev_err(&client->dev, "Unable to initialize sensor\n");
			return ret;
		}else{
			dev_info(&client->dev, "Aptina MT9D131/A2010SOC is initialized and ready to go!\n");
			sensor_is_ready=1; 
		} 
		break;
	default:
		dev_err(&client->dev, " mt9d131_v4l2_int_s_power(): unknow 'power' value: %d\n",power);
		break;
	}
	
	return 0;
}

/**
 * mt9d131_v4l2_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 * ----->Note, this function is not active in this release.<------
 */
static int mt9d131_v4l2_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = -EINVAL;
	struct vcontrol *lvc;
	struct mt9d131_priv *priv = s->priv;
	struct i2c_client *client = priv->client;
	
	int i = find_vctrl(vc->id);
	if (i < 0) return -EINVAL;

	lvc = &mt9d131_video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		retval = mt9d131_set_exposure_time(vc->value, client, lvc);
		break;
	case V4L2_CID_GAIN:
		retval = mt9d131_set_gain(vc->value, client, lvc);
		break;
	default:
		dev_err(&client->dev, "mt9d131_v4l2_int_s_ctrl(): unknow cid:%d\n",vc->id);
		break;
	}
	return retval;
}

/**
 * mt9d131_v4l2_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 * ----->Note, this function is not active in this release.<------
 */
static int 
mt9d131_v4l2_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct vcontrol *lvc;

	int i = find_vctrl(vc->id);
	if (i < 0) return -EINVAL;

	lvc = &mt9d131_video_control[i];

	return -EINVAL; //It's not applicable for SOC sensor when AE/AWB is on. 
					//this line shall be removed in future when v4l2_s_ctrl is supported. 

	/*switch (vc->id) {
	case  V4L2_CID_EXPOSURE:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	default:
		return -EINVAL; //<--out of range
	}

	return 0;*/
}

/**
 * mt9d131_v4l2_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int 
mt9d131_v4l2_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL) qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0) return -EINVAL;

	*qc = mt9d131_video_control[i].qc;
	return 0;
}

/**
 * mt9d131_v4l2_int_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int 
mt9d131_v4l2_int_enum_fmt_cap(struct v4l2_int_device *s,struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if(debug_level&DEBUG_FRAME_FORMAT) printk("fmt->index =%d, fmt->type=V4L2_BUF_TYPE_VIDEO_CAPTURE\n",fmt->index);
		if(index >= ARRAY_SIZE(mt9d131_formats)) 
			return -EINVAL;

        break;
	default:
		printk(KERN_ERR KBUILD_MODNAME "mt9d131_v4l2_int_enum_fmt_cap() failed, return -EINVAL\n");
		return -EINVAL;
	}

	strlcpy(fmt->description, mt9d131_formats[index].description, sizeof(fmt->description));
	fmt->pixelformat = mt9d131_formats[index].pixelformat;

	return 0;
}

/**
 * mt9d131_v4l2_int_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int 
mt9d131_v4l2_int_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	enum mt9d131_image_size isize;
	int ifmt,ret=0;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct mt9d131_priv *priv = s->priv;
	struct v4l2_pix_format *pix2 = &priv->pix;

	if((pix->width==priv->mWidth) && (pix->height==priv->mHeight)) return ret;  //do nothing if it's a repeated request

	if(debug_level&DEBUG_FRAME_FORMAT) printk("request format from input:  pix->pixelformat=0x%x\n",pix->pixelformat);

	isize = mt9d131_calc_size(pix->width, pix->height);
	mt9d131_current_size_index = isize;
	if(debug_level&DEBUG_FRAME_FORMAT){ 
		printk("isize=%d for given pix->width=%d,pix->height=%d\n",isize,pix->width,pix->height);
	}
	priv->mWidth=pix->width;
	priv->mHeight=pix->height;

	for(ifmt = 0; ifmt < ARRAY_SIZE(mt9d131_formats); ifmt++) {
		if (pix->pixelformat == mt9d131_formats[ifmt].pixelformat){
			if(debug_level&DEBUG_FRAME_FORMAT) 
				printk("found match@ifmt=%d:pix->pixelformat=0x%x,mt9d131_formats[ifmt].pixelformat=0x%x\n",
						ifmt,pix->pixelformat,mt9d131_formats[ifmt].pixelformat); 
			break;
		}
	}
	if(ifmt == ARRAY_SIZE(mt9d131_formats)){
		printk("couldn't find any matched pixel formats, use the 1st entry in the table!\n");
		return -1;
	}

	pix->pixelformat = mt9d131_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	*pix2 = *pix;

	return 0;
}

/**
 * mt9d131_v4l2_int_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int 
mt9d131_v4l2_int_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct mt9d131_priv		*priv = s->priv;
	struct v4l2_pix_format	*pix = &f->fmt.pix;
	int						 ret;
	
	if(debug_level&DEBUG_FRAME_FORMAT) printk("before s_fmt:input format=0x%x,width=%d,height=%d\n",
												pix->pixelformat,pix->width, pix->height);
	ret = mt9d131_v4l2_int_try_fmt_cap(s, f);
	if (!ret){ 
		ret=mt9d131_setup_sensor_output(priv->client, pix->width, pix->height);
	}
		
	return ret;
}

/**
 * mt9d131_v4l2_int_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int 
mt9d131_v4l2_int_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct mt9d131_priv *priv = s->priv;
	
	f->fmt.pix.width		= priv->pix.width;
	f->fmt.pix.height		= priv->pix.height;
	f->fmt.pix.pixelformat	= V4L2_COLORSPACE_JPEG;
	f->fmt.pix.pixelformat	= priv->pix.pixelformat;
	f->fmt.pix.field		= V4L2_FIELD_NONE;
	return 0;
}

#if 0 //Will be implemented later
/**
 * mt9d131_calc_xclk - Calculate the required xclk frequency
 * @c: i2c client driver structure
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate and return the required xclk frequency
 */
static unsigned long 
mt9d131_calc_xclk(struct i2c_client *c)
{
	struct mt9d131_priv *priv = i2c_get_clientdata(c);
	struct v4l2_fract *timeperframe = &priv->timeperframe;

	if (timeperframe->numerator == 0 ||
	    timeperframe->denominator == 0) {
		/* supply a default nominal_timeperframe */
		timeperframe->numerator = 15;
		timeperframe->denominator = MT9D131_DEF_FPS;
	}

	priv->fps = timeperframe->denominator / timeperframe->numerator;
	if (priv->fps < MT9D131_MIN_FPS)
		priv->fps = MT9D131_MIN_FPS;
	else if (priv->fps > MT9D131_MAX_FPS)
		priv->fps = MT9D131_MAX_FPS;

	timeperframe->numerator = 15;
	timeperframe->denominator = priv->fps;

	return MT9D131_XCLK_NOM_1;
}
#endif

/**
 * mt9d131_v4l2_int_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 * ----->Note, this function is not active in this release.<------
 */
static int mt9d131_v4l2_int_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	/*struct mt9d131_priv *priv		  = s->priv;
	struct i2c_client	*client		  = priv->client;
	struct v4l2_fract	*timeperframe = &a->parm.capture.timeperframe;


	priv->timeperframe = *timeperframe;
	priv->xclk_current = mt9d131_calc_xclk(client);
	*timeperframe = priv->timeperframe;
	return 0;*/
	
	//Not yet available.  Will be implemented later
	return 0;
}

/**
 * mt9d131_v4l2_int_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int 
mt9d131_v4l2_int_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct mt9d131_priv *priv		= s->priv;
	struct v4l2_captureparm *cparm	= &a->parm.capture;

	if(a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = priv->timeperframe;
	if(priv->mWidth<=MT9D131_DEFAULT_WIDTH){
		cparm->timeperframe.numerator  = mt9d131_frameintervals_fast[0].numerator;
		cparm->timeperframe.denominator= mt9d131_frameintervals_fast[0].denominator;
	}else{
		cparm->timeperframe.numerator  = mt9d131_frameintervals_slow[0].numerator;
		cparm->timeperframe.denominator= mt9d131_frameintervals_slow[0].denominator;
	}
	return 0;
}

/**
 * mt9d131_v4l2_int_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int 
mt9d131_v4l2_int_g_priv(struct v4l2_int_device *s, void *p)
{
	struct mt9d131_priv *priv = s->priv;

	return priv->pdata->priv_data_set(p);
}

/**
 * mt9d131_v4l2_int_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's ifparm
 *
 * Returns device's (sensor's) ifparm in p parameter
 */
static int mt9d131_v4l2_int_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct mt9d131_priv *priv = s->priv;
	int rval;

	if (p == NULL)
		return -EINVAL;

	if (!priv->pdata->ifparm)
		return -EINVAL;

	rval = priv->pdata->ifparm(p);
	if (rval) {
		v4l_err(priv->client, "g_ifparm.Err[%d]\n", rval);
		return rval;
	}

	return 0;
}

/**
 * mt9d131_v4l2_int_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 */
static int 
mt9d131_v4l2_int_enum_framesizes(struct v4l2_int_device *s, struct v4l2_frmsizeenum *frms)
{
	int ifmt;

	for (ifmt = 0; ifmt < ARRAY_SIZE(mt9d131_formats); ifmt++){
		if (mt9d131_formats[ifmt].pixelformat == frms->pixel_format){
			if(debug_level&DEBUG_ENUM_FRAME_SIZE) 
				printk("found a matched pixelformat:0x%x at table entry %d\n",mt9d131_formats[ifmt].pixelformat,ifmt);
			break;
		}
	}
	if (ifmt == ARRAY_SIZE(mt9d131_formats)){
		printk(KERN_ERR KBUILD_MODNAME "Couldn't find any matched for a given frame format:0x%x\n",frms->pixel_format);
	}

	if (frms->index >= ARRAY_SIZE(mt9d131_supported_framesizes)){
		if(debug_level&DEBUG_ENUM_FRAME_SIZE) 
			printk("We've already reached all discrete framesizes %d!\n", ARRAY_SIZE(mt9d131_supported_framesizes));
		frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frms->discrete.width = mt9d131_supported_framesizes[frms->index -1 ].width;
		frms->discrete.height = mt9d131_supported_framesizes[frms->index-1 ].height;
		return -EINVAL;
	}

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = mt9d131_supported_framesizes[frms->index].width;
	frms->discrete.height= mt9d131_supported_framesizes[frms->index].height;
	return 0;
}

/**
 * mt9d131_v4l2_int_enum_frameintervals - V4L2 sensor if handler for vidioc_int_enum_frameintervals
 * @s: pointer to standard V4L2 device structure
 * @frmi: pointer to standard V4L2 frameinterval enumeration structure
 *
 * Returns possible frameinterval numerator and denominator depending on choosen frame size
 */
static int 
mt9d131_v4l2_int_enum_frameintervals(struct v4l2_int_device *s, struct v4l2_frmivalenum *frmi)
{
	int ifmt;
	int max_size;

	for (ifmt = 0; ifmt < ARRAY_SIZE(mt9d131_formats); ifmt++)
		if (mt9d131_formats[ifmt].pixelformat == frmi->pixel_format) break;

	max_size = ARRAY_SIZE(mt9d131_supported_framesizes);
	if(frmi->index==max_size) return -EINVAL;
	
	for(ifmt = 0; ifmt < max_size; ifmt++) {
		if(frmi->width <= mt9d131_supported_framesizes[ifmt].width) {
			frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
			if(frmi->width<=MT9D131_DEFAULT_WIDTH){
				frmi->discrete.numerator   =mt9d131_frameintervals_fast[frmi->index].numerator;
				frmi->discrete.denominator =mt9d131_frameintervals_fast[frmi->index].denominator;
			}else{
				frmi->discrete.numerator   =mt9d131_frameintervals_slow[frmi->index].numerator;
				frmi->discrete.denominator =mt9d131_frameintervals_slow[frmi->index].denominator;
			}
			if(debug_level&DEBUG_ENUM_FRAME_INTERVAL) printk("ifmt=%d,frmi->index=%d,frmi->width=%d\n",
															ifmt,frmi->index,frmi->width);
			if(frmi->discrete.denominator <= mt9d131_frameintervals_fast[max_size - ifmt - 1].denominator) 
				return 0;
			else
				return -EINVAL;

			return 0;
		}
	}
	return 0;
}

static struct v4l2_int_ioctl_desc mt9d131_ioctl_desc[] = {
	{ .num = vidioc_int_enum_framesizes_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_enum_framesizes },
	{ .num = vidioc_int_enum_frameintervals_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_enum_frameintervals },
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_g_priv },
	{ .num = vidioc_int_g_ifparm_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_g_ifparm },
	{ .num = vidioc_int_enum_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_g_fmt_cap }, 
	{ .num = vidioc_int_s_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_g_parm },
	{ .num = vidioc_int_s_parm_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_int_s_parm },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_s_ctrl },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)mt9d131_v4l2_queryctrl },
};

#ifdef MT9D131_DEBUG_REG_ACCESS
/***********************************************************************************
 *                               Sysfs                                             *                               
************************************************************************************/
/* Basic register read write support */
static u16 mt9d131_attr_basic_addr  = 0x0000;

static ssize_t
mt9d131_basic_reg_addr_show( struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", mt9d131_attr_basic_addr);
}

static ssize_t
mt9d131_basic_reg_addr_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	u16 val;
	sscanf(buf, "%hx", &val);
	mt9d131_attr_basic_addr = (u16) val;
	return n;
}

static DEVICE_ATTR( basic_reg_addr, S_IRUGO|S_IWUSR, mt9d131_basic_reg_addr_show, mt9d131_basic_reg_addr_store);


static ssize_t
mt9d131_basic_reg_val_show( struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 val;
	int ret;

	ret = mt9d131_reg_read(sysPriv.client, mt9d131_attr_basic_addr, &val);
	if(ret < 0){        
		printk(KERN_INFO "mt9d131: Basic register read failed");
		return 1; // nothing processed
	} else {
		return sprintf(buf, "0x%x\n", val);
	}
}

static ssize_t
mt9d131_basic_reg_val_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	u32 val;
	sscanf(buf, "%x", &val);

	if (mt9d131_reg_write(sysPriv.client, mt9d131_attr_basic_addr, (u16)val)) {
		printk(KERN_INFO "mt9d131_basic_reg_val_store(): val=0x%x is written failed, return n=%d\n",val,n);
		return n; // nothing processed
	} else {
		return n;
	}
}
static DEVICE_ATTR( basic_reg_val, S_IRUGO|S_IWUSR, mt9d131_basic_reg_val_show, mt9d131_basic_reg_val_store);


/* Exposure time access support */
static ssize_t
mt9d131_exposure_val_show( struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 val;
	struct vcontrol *lvc;
	int i = find_vctrl(V4L2_CID_EXPOSURE);
	if (i < 0)
		return -EINVAL;
	lvc = &mt9d131_video_control[i];
	val = lvc->current_value;
	
	if(val < 0){        
		printk(KERN_INFO "mt9d131: Exposure value read failed");
		return 1; // nothing processed
	} else {
		return sprintf(buf, "%d\n", val);
	}
}

static ssize_t
mt9d131_exposure_val_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	u32 val;
	struct i2c_client *client;
	struct vcontrol *lvc;
	
	sscanf(buf, "%d", &val);
	client = sysPriv.client;
		
	lvc = &mt9d131_video_control[V4L2_CID_EXPOSURE];	

	if (mt9d131_set_exposure_time((u32)val, client, lvc)) {
		printk(KERN_INFO "mt9d131: Exposure write failed");
		return n; // nothing processed
	} else {
		return n;
    }
}

static DEVICE_ATTR( exposure_val, S_IRUGO|S_IWUSR, mt9d131_exposure_val_show, mt9d131_exposure_val_store);


/* Global Gain access support */
static ssize_t
mt9d131_gain_val_show( struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 val;
	struct vcontrol *lvc;
    
	int i = find_vctrl(V4L2_CID_GAIN);
	if (i < 0)
		return -EINVAL;
	lvc = &mt9d131_video_control[i];
	val = lvc->current_value;
      
	if(val < 0){        
		printk(KERN_INFO "mt9d131: Global Gain value read failed");
		return 1; // nothing processed
	} else {
		return sprintf(buf, "%d\n", val);
    }
}

static ssize_t
mt9d131_gain_val_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	u16 val;
	struct i2c_client *client;
	struct vcontrol *lvc;
	
	sscanf(buf, "%hd", &val);
	client = sysPriv.client;
		
	lvc = &mt9d131_video_control[V4L2_CID_GAIN];	
		
	if (mt9d131_set_gain(val, client, lvc)) {
		printk(KERN_INFO "mt9d131: Global gain write failed");
		return n; // nothing processed
	} else {
		return n;
	}
}

static DEVICE_ATTR( gain_val, S_IRUGO|S_IWUSR, mt9d131_gain_val_show, mt9d131_gain_val_store);

static struct attribute *mt9d131_sysfs_attr[] = {
	&dev_attr_basic_reg_addr.attr,
	&dev_attr_basic_reg_val.attr,
	&dev_attr_exposure_val.attr,
	&dev_attr_gain_val.attr,
};

static int 
mt9d131_sysfs_add(struct kobject *kobj)
{
	int i = ARRAY_SIZE(mt9d131_sysfs_attr);
	int rval = 0;
	
	do {
		rval = sysfs_create_file(kobj, mt9d131_sysfs_attr[--i]);
	} while((i > 0) && (rval == 0));
	return rval;
}

static int 
mt9d131_sysfs_rm(struct kobject *kobj)
{
	int i = ARRAY_SIZE(mt9d131_sysfs_attr);
	int rval = 0;

	do {
		sysfs_remove_file(kobj, mt9d131_sysfs_attr[--i]);
	} while(i > 0);
	return rval;
}
#endif	//MT9D131_DEBUG_REG_ACCESS

static struct v4l2_int_slave mt9d131_slave = {
	.ioctls = mt9d131_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9d131_ioctl_desc),
};

/**
 * mt9d131_probe - probing the mt9d131 soc sensor
 * @client: i2c client driver structure
 * @did:    device id of i2c_device_id structure
 *
 * Upon the given i2c client, the sensor's module id is to be retrieved.
 */
static int 
mt9d131_probe(struct i2c_client *client, const struct i2c_device_id *did)
{
	struct mt9d131_priv		*priv;
	struct v4l2_int_device	*v4l2_int_device;
	int						 ret;

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) return -ENOMEM;

	v4l2_int_device = kzalloc(sizeof(*v4l2_int_device), GFP_KERNEL);
	if (!v4l2_int_device) {
		kfree(priv);
		return -ENOMEM;
	}

	v4l2_int_device->module = THIS_MODULE;
	strncpy(v4l2_int_device->name, "mt9d131", sizeof(v4l2_int_device->name));
	
	v4l2_int_device->type = v4l2_int_type_slave;
	v4l2_int_device->u.slave = &mt9d131_slave;

	v4l2_int_device->priv = priv;
	priv->v4l2_int_device = v4l2_int_device;
	priv->client = client;
	priv->pdata = client->dev.platform_data;
	priv->pdata->flags = MT9D131_FLAG_PCLK_RISING_EDGE;
	
	/* Setting Pixel Values */
	priv->pix.width       = mt9d131_supported_framesizes[7].width;
	priv->pix.height      = mt9d131_supported_framesizes[7].height;
	priv->pix.pixelformat = mt9d131_formats[0].pixelformat;
	i2c_set_clientdata(client, priv);
	sysPriv.client = priv->client;

	ret = v4l2_int_device_register(priv->v4l2_int_device);
	if (ret) {
		i2c_set_clientdata(client, NULL);
		kfree(v4l2_int_device);
		kfree(priv);
	}
	
#ifdef MT9D131_DEBUG_REG_ACCESS
	mt9d131_sysfs_add(&client->dev.kobj);
#endif
	return ret;
}

/**
 * mt9d131_remove - remove the mt9d131 soc sensor driver module
 * @client: i2c client driver structure
 *
 * Upon the given i2c client, the sensor driver module is removed.
 */
static int 
mt9d131_remove(struct i2c_client *client)
{
	struct mt9d131_priv *priv = i2c_get_clientdata(client);

	v4l2_int_device_unregister(priv->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

#ifdef MT9D131_DEBUG_REG_ACCESS
	mt9d131_sysfs_rm(&client->dev.kobj);
#endif	
	
	kfree(priv->v4l2_int_device);
	kfree(priv);
	return 0;
}

static const struct i2c_device_id mt9d131_id[] = {
	{ "mt9d131", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9d131_id);

static struct i2c_driver mt9d131_i2c_driver = {
	.driver = {
		.name = "mt9d131",
	},
	.probe    = mt9d131_probe,
	.remove   = mt9d131_remove,
	.id_table = mt9d131_id,
};

/************************************************************************
			module function
************************************************************************/
static int __init mt9d131_module_init(void)
{
	return i2c_add_driver(&mt9d131_i2c_driver);
}

static void __exit mt9d131_module_exit(void)
{
	printk("debug_level=%d\n", debug_level);
	i2c_del_driver(&mt9d131_i2c_driver);
}

module_init(mt9d131_module_init);
module_exit(mt9d131_module_exit);

MODULE_DESCRIPTION("mt9d131 soc sensor(2 meg pixel) driver");
MODULE_AUTHOR("Aptina");
MODULE_LICENSE("GPL v2");
module_param(debug_level, int, 0644);
MODULE_PARM_DESC(debug_level, "Debug level");
