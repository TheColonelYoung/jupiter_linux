// SPDX-License-Identifier: GPL-2.0-only

/*
 * Sony IMX678 sensor driver
 *
 */

#include <asm/unaligned.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Streaming Mode */
#define IMX678_REG_MODE_SELECT	0x3000
#define IMX678_MODE_STANDBY	    0x01
#define IMX678_MODE_STREAMING	0x00

/* Chip ID */
#define IMX678_REG_ID		0x4d1c
#define IMX678_ID			0x2a6

/* Chip color */
#define IMX678_REG_COL		0x4d18
#define IMX678_COL			0x1

/* Input clock rate INCK*/
#define IMX678_INCLK_RATE	72000000

/* CSI2 HW configuration - defined by selected mode of sensor */
/* For 2-lane,  all-pixel, 10-bit mode is data rate 1440 Mbps/lane*/
#define IMX678_LINK_FREQ		1440000000
#define IMX678_NUM_DATA_LANES	2

#define IMX678_REG_MIN		0x00
#define IMX678_REG_MAX		0xfffff

/**
 * struct imx678_reg - imx678 sensor register
 * @address: Register address
 * @val: Register value
 */
struct imx678_reg {
	u16 address;
	u8 val;
};

/**
 * struct imx678_reg_list - imx678 sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct imx678_reg_list {
	u32 num_of_regs;
	const struct imx678_reg *regs;
};

/**
 * struct imx678_mode - imx678 sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @hblank: Horizontal blanking in lines
 * @vblank: Vertical blanking in lines
 * @vblank_min: Minimal vertical blanking in lines
 * @vblank_max: Maximum vertical blanking in lines
 * @pclk: Sensor pixel clock
 * @link_freq_idx: Link frequency index
 * @reg_list: Register list for sensor mode
 */
struct imx678_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 hblank;
	u32 vblank;
	u32 vblank_min;
	u32 vblank_max;
	u64 pclk;
	u32 link_freq_idx;
	struct imx678_reg_list reg_list;
};

/**
 * struct imx678 - imx678 sensor device structure
 * @dev: Pointer to generic device
 * @client: Pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @inclk: Sensor input clock
 * @ctrl_handler: V4L2 control handler
 * @link_freq_ctrl: Pointer to link frequency control
 * @pclk_ctrl: Pointer to pixel clock control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @vblank: Vertical blanking in lines
 * @cur_mode: Pointer to current selected sensor mode
 * @mutex: Mutex for serializing sensor controls
 * @streaming: Flag indicating streaming state
 */
struct imx678 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct clk *inclk;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
	};
	u32 vblank;
	const struct imx678_mode *cur_mode;
	struct mutex mutex;
	bool streaming;
};

static const s64 link_freq[] = {
	IMX678_LINK_FREQ,
};

/**
 * to_imx678() - imv678 V4L2 sub-device to imx678 device.
 * @subdev: pointer to imx678 V4L2 sub-device
 *
 * Return: pointer to imx678 device
 */
static inline struct imx678 *to_imx678(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct imx678, sd);
}

/**
 * imx678_read_reg() - Read registers.
 * @imx678: pointer to imx678 device
 * @reg: register address
 * @len: length of bytes to read. Max supported bytes is 4
 * @val: pointer to register value to be filled.
 *
 * Big endian register addresses with little endian values.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_read_reg(struct imx678 *imx678, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct i2c_msg msgs[2] = {0};
	u8 addr_buf[2] = {0};
	u8 data_buf[4] = {0};
	int ret;

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_le32(data_buf);

	return 0;
}

/**
 * imx678_write_reg() - Write register
 * @imx678: pointer to imx678 device
 * @reg: register address
 * @len: length of bytes. Max supported bytes is 4
 * @val: register value
 *
 * Big endian register addresses with little endian values.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_write_reg(struct imx678 *imx678, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	u8 buf[6] = {0};

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_le32(val, buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/**
 * imx678_write_regs() - Write a list of registers
 * @imx678: pointer to imx678 device
 * @regs: list of registers to be written
 * @len: length of registers array
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_write_regs(struct imx678 *imx678,
			     const struct imx678_reg *regs, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx678_write_reg(imx678, regs[i].address, 1, regs[i].val);
		if (ret)
			return ret;
	}

	return 0;
}


/**
 * imx678_power_on() - Sensor power on sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx678 *imx678 = to_imx678(sd);
	int ret;

	gpiod_set_value_cansleep(imx678->reset_gpio, 1);

	ret = clk_prepare_enable(imx678->inclk);
	if (ret) {
		dev_err(imx678->dev, "fail to enable inclk");
		goto error_reset;
	}

	usleep_range(18000, 20000);

	return 0;

error_reset:
	gpiod_set_value_cansleep(imx678->reset_gpio, 0);

	return ret;
}

/**
 * imx678_power_off() - Sensor power off sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx678 *imx678 = to_imx678(sd);

	gpiod_set_value_cansleep(imx678->reset_gpio, 0);

	clk_disable_unprepare(imx678->inclk);

	return 0;
}

/**
 * imx678_detect() - Detect imx678 sensor
 * @imx678: pointer to imx678 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int imx678_detect(struct imx678 *imx678)
{
	int ret;
	u32 val;

	ret = imx678_read_reg(imx678, IMX678_REG_ID, 2, &val);
	if (ret)
		return ret;

	if (val != IMX678_ID) {
		printk(KERN_INFO "[IMX678]: Probe - Sensor ID mismatch %x!=%x\n", IMX678_ID, val);
		dev_err(imx678->dev, "chip id mismatch: %x!=%x",
			IMX678_ID, val);
		return -ENXIO;
	}
	return 0;
}

/* V4l2 subdevice ops */

static const struct v4l2_subdev_video_ops imx678_video_ops = {
	//.s_stream = imx678_set_stream,
};

static const struct v4l2_subdev_pad_ops imx678_pad_ops = {
	//.init_cfg = imx678_init_pad_cfg,
	//.enum_mbus_code = imx678_enum_mbus_code,
	//.enum_frame_size = imx678_enum_frame_size,
	//.get_fmt = imx678_get_pad_format,
	//.set_fmt = imx678_set_pad_format,
};

static const struct v4l2_subdev_ops imx678_subdev_ops = {
	//.video = &imx678_video_ops,
	//.pad = &imx678_pad_ops,
};


/**
 * imx678_parse_hw_config() - Parse HW configuration and check if supported
 * @imx678: pointer to imx678 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_parse_hw_config(struct imx678 *imx678)
{
	struct fwnode_handle *fwnode = dev_fwnode(imx678->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	unsigned long rate;
	int ret;
	int i;

	if (!fwnode)
		return -ENXIO;

	/* Request optional reset pin */
	imx678->reset_gpio = devm_gpiod_get_optional(imx678->dev, "reset",
						     GPIOD_OUT_LOW);
	if (IS_ERR(imx678->reset_gpio)) {
		dev_err(imx678->dev, "failed to get reset gpio %ld",
			PTR_ERR(imx678->reset_gpio));

		return PTR_ERR(imx678->reset_gpio);
	}

	/* Get sensor input clock */
	imx678->inclk = devm_clk_get(imx678->dev, NULL);
	if (IS_ERR(imx678->inclk)) {
		dev_err(imx678->dev, "could not get inclk");
		return PTR_ERR(imx678->inclk);
	}

	rate = clk_get_rate(imx678->inclk);
	if (rate != IMX678_INCLK_RATE) {
		dev_err(imx678->dev, "inclk frequency mismatch");
		return -EINVAL;
	}

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX678_NUM_DATA_LANES) {
		dev_err(imx678->dev,
			"number of CSI2 data lanes %d is not supported",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(imx678->dev, "no link frequencies defined");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == IMX678_LINK_FREQ)
			goto done_endpoint_free;

	ret = -EINVAL;

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/**
 * imx678_probe() - I2C client device binding
 * @client: pointer to i2c client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_probe(struct i2c_client *client)
{
	struct imx678 *imx678;
	int ret;

	imx678 = devm_kzalloc(&client->dev, sizeof(*imx678), GFP_KERNEL);
	if (!imx678){
		return -ENOMEM;
	}

	imx678->dev = &client->dev;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx678->sd, client, &imx678_subdev_ops);

	ret = imx678_parse_hw_config(imx678);
	if (ret) {
		dev_err(imx678->dev, "HW configuration is not supported");
		return ret;
	}

	mutex_init(&imx678->mutex);
	/*
	printk(KERN_INFO "[IMX678]: Probe - Power on\n");
	ret = imx678_power_on(imx678->dev);
	if (ret) {
		dev_err(imx678->dev, "failed to power-on the sensor");
		printk(KERN_INFO "[IMX678]: Probe - Failed to power-on the sensor\n");
		goto error_mutex_destroy;
	}*/

	/* Check module identity */
	ret = imx678_detect(imx678);
	if (ret) {
		dev_err(imx678->dev, "failed to find sensor: %d", ret);
		goto error_power_off;
	} else {
		printk(KERN_INFO "[IMX678]: Probe - Sensor detected\n");
	}

	/* Set default mode to max resolution */
    /*
	imx678->cur_mode = &supported_mode;
	imx678->vblank = imx678->cur_mode->vblank;

	ret = imx678_init_controls(imx678);
	if (ret) {
		dev_err(imx678->dev, "failed to init controls: %d", ret);
		goto error_power_off;
	}*/

	/* Initialize subdev */
	//imx678->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	//imx678->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
    /*
	imx678->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx678->sd.entity, 1, &imx678->pad);
	if (ret) {
		dev_err(imx678->dev, "failed to init entity pads: %d", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx678->sd);
	if (ret < 0) {
		dev_err(imx678->dev,
			"failed to register async subdev: %d", ret);
		goto error_media_entity;
	}*/

	pm_runtime_set_active(imx678->dev);
	pm_runtime_enable(imx678->dev);
	pm_runtime_idle(imx678->dev);

	printk(KERN_INFO "[IMX678]: Probe - End\n");

	return 0;

error_media_entity:
	media_entity_cleanup(&imx678->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(imx678->sd.ctrl_handler);
error_power_off:
	imx678_power_off(imx678->dev);
error_mutex_destroy:
	mutex_destroy(&imx678->mutex);

	return ret;
}

/**
 * imx678_remove() - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx678_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	//v4l2_async_unregister_subdev(sd);
	//media_entity_cleanup(&sd->entity);
	//v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	pm_runtime_suspended(&client->dev);

	mutex_destroy(&imx678->mutex);

	return 0;
}

static const struct dev_pm_ops imx678_pm_ops = {
	SET_RUNTIME_PM_OPS(imx678_power_off, imx678_power_on, NULL)
};

static const struct of_device_id imx678_of_match[] = {
	{ .compatible = "sony,imx678" },
	{ }
};

MODULE_DEVICE_TABLE(of, imx678_of_match);

static struct i2c_driver imx678_driver = {
	.probe_new = imx678_probe,
	.remove = imx678_remove,
	.driver = {
		.name = "imx678",
		.pm = &imx678_pm_ops,
		.of_match_table = imx678_of_match,
	},
};

module_i2c_driver(imx678_driver);

MODULE_DESCRIPTION("Sony imx678 sensor driver");
MODULE_LICENSE("GPL");
