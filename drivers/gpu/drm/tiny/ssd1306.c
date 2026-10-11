// SPDX-License-Identifier: GPL-2.0
/*
 * DRM driver for Solomon SSD1306 OLED panels
 *
 * Copyright 2022 Noralf Trønnes
 * Copyright 2026 Soumyajyoti Sarkar
 *
 * Based on panel-mipi-dbi.c and fb_ssd1306.c
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_print.h>

#include <video/mipi_display.h>

static const u8 ssd1306_magic[15] = { 'M', 'I', 'P', 'I', ' ', 'D', 'B', 'I',
				      0, 0, 0, 0, 0, 0, 0 };

struct ssd1306_commands {
	const u8 *buf;
	size_t len;
};

static struct ssd1306_commands *
ssd1306_check_commands(struct device *dev, const struct firmware *fw)
{
	const u8 *data = fw->data;
	struct ssd1306_commands *cmds;
	size_t size = fw->size, cmds_len;
	unsigned int i = 0;

	if (size < sizeof(ssd1306_magic) + 1 + 2) {
		dev_err(dev, "firmware too small\n");
		return ERR_PTR(-EINVAL);
	}

	if (memcmp(data, ssd1306_magic, sizeof(ssd1306_magic))) {
		dev_err(dev, "bad magic: %15ph\n", data);
		return ERR_PTR(-EINVAL);
	}

	if (data[sizeof(ssd1306_magic)] != 1) {
		dev_err(dev, "unsupported version\n");
		return ERR_PTR(-EINVAL);
	}

	cmds_len = size - sizeof(ssd1306_magic) - 1;
	data += sizeof(ssd1306_magic) + 1;

	while ((i + 1) < cmds_len) {
        i++;  // skip command byte, we only validate structure
		u8 nparams = data[i++];
		i += nparams;
		if (i > cmds_len)
			return ERR_PTR(-EINVAL);
	}

	cmds = devm_kzalloc(dev, sizeof(*cmds), GFP_KERNEL);
	if (!cmds)
		return ERR_PTR(-ENOMEM);

	cmds->len = cmds_len;
	cmds->buf = devm_kmemdup(dev, data, cmds_len, GFP_KERNEL);
	if (!cmds->buf)
		return ERR_PTR(-ENOMEM);

	return cmds;
}

static struct ssd1306_commands *
ssd1306_commands_from_fw(struct device *dev)
{
	struct ssd1306_commands *cmds;
	const struct firmware *fw;
	const char *compatible;
	char fw_name[40];
	int ret;

	ret = of_property_read_string_index(dev->of_node, "compatible", 0, &compatible);
	if (ret)
		return ERR_PTR(ret);

	snprintf(fw_name, sizeof(fw_name), "%s.bin", compatible);
	ret = request_firmware(&fw, fw_name, dev);
	if (ret) {
		dev_err(dev, "firmware '%s' not found (%d)\n", fw_name, ret);
		return ERR_PTR(ret);
	}

	cmds = ssd1306_check_commands(dev, fw);
	release_firmware(fw);
	return cmds;
}

static void ssd1306_commands_execute(struct mipi_dbi *dbi,
				     struct ssd1306_commands *cmds)
{
	unsigned int i = 0;

	if (!cmds)
		return;

	while (i < cmds->len) {
		u8 cmd = cmds->buf[i++];
		u8 nparams = cmds->buf[i++];
		const u8 *params = &cmds->buf[i];

		if (cmd == 0x00 && nparams == 1)
			msleep(params[0]);
		else if (nparams)
			mipi_dbi_command_stackbuf(dbi, cmd, params, nparams);
		else
			mipi_dbi_command(dbi, cmd);

		i += nparams;
	}
}

struct ssd1306_device {
	struct mipi_dbi_dev dbidev;

	struct drm_plane plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
};

static struct ssd1306_device *to_ssd1306(struct drm_device *drm)
{
	return container_of(drm_to_mipi_dbi_dev(drm), struct ssd1306_device, dbidev);
}

static const u32 ssd1306_formats[] = {
	DRM_MIPI_DBI_PLANE_FORMATS,
};

static const u64 ssd1306_modifiers[] = {
	DRM_MIPI_DBI_PLANE_FORMAT_MODIFIERS,
};

static const struct drm_plane_helper_funcs ssd1306_plane_helper = {
	DRM_MIPI_DBI_PLANE_HELPER_FUNCS,
};

static const struct drm_plane_funcs ssd1306_plane_funcs = {
	DRM_MIPI_DBI_PLANE_FUNCS,
	.destroy = drm_plane_cleanup,
};

static void ssd1306_enable(struct drm_crtc *crtc, struct drm_atomic_commit *state)
{
	struct drm_device *drm = crtc->dev;
	struct ssd1306_device *s = to_ssd1306(drm);
	struct mipi_dbi_dev *dbidev = &s->dbidev;
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	int ret, idx;

	if (!drm_dev_enter(drm, &idx))
		return;

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out;
	if (!ret)
		ssd1306_commands_execute(dbi, dbidev->driver_private);

	/* Rotation: SSD1306 uses 0xA0/0xA1 (seg remap), 0xC0/0xC8 (COM scan) */
	switch (dbidev->rotation) {
	default:
		addr_mode = 0xA0; break;  /* A0 = normal */
	case 90:
		addr_mode = 0xA0; break;
	case 180:
		addr_mode = 0xA1 | 0xC8; break;
	case 270:
		addr_mode = 0xA1 | 0xC0; break;
	}
	mipi_dbi_command(dbi, addr_mode & 0xFF);
	if (addr_mode & 0x100)
		mipi_dbi_command(dbi, (addr_mode >> 8) & 0xFF);

	backlight_enable(dbidev->backlight);
out:
	drm_dev_exit(idx);
}

static const struct drm_crtc_helper_funcs ssd1306_crtc_helper = {
	DRM_MIPI_DBI_CRTC_HELPER_FUNCS,
	.atomic_enable = ssd1306_enable,
};

static const struct drm_crtc_funcs ssd1306_crtc_funcs = {
	DRM_MIPI_DBI_CRTC_FUNCS,
	.destroy = drm_crtc_cleanup,
};

static const struct drm_encoder_funcs ssd1306_enc_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_connector_helper_funcs ssd1306_conn_helper = {
	DRM_MIPI_DBI_CONNECTOR_HELPER_FUNCS,
};

static const struct drm_connector_funcs ssd1306_conn_funcs = {
	DRM_MIPI_DBI_CONNECTOR_FUNCS,
	.destroy = drm_connector_cleanup,
};

static const struct drm_mode_config_helper_funcs ssd1306_mode_helper = {
	DRM_MIPI_DBI_MODE_CONFIG_HELPER_FUNCS,
};

static const struct drm_mode_config_funcs ssd1306_mode_funcs = {
	DRM_MIPI_DBI_MODE_CONFIG_FUNCS,
};

static const struct drm_display_mode ssd1306_mode = {
	DRM_SIMPLE_MODE(128, 64, 37, 49),
};

DEFINE_DRM_GEM_DMA_FOPS(ssd1306_fops);

static const struct drm_driver ssd1306_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ssd1306_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "ssd1306",
	.desc			= "Solomon SSD1306",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id ssd1306_of_match[] = {
	{ .compatible = "solomon,ssd1306" },
	{ }
};
MODULE_DEVICE_TABLE(of, ssd1306_of_match);

static const struct spi_device_id ssd1306_id[] = {
	{ "ssd1306", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ssd1306_id);

static int ssd1306_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ssd1306_device *s;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	struct drm_plane *plane;
	struct drm_crtc *crtc;
	struct drm_encoder *enc;
	struct drm_connector *conn;
	u32 rotation = 0;
	int ret;

	s = devm_drm_dev_alloc(dev, &ssd1306_driver, struct ssd1306_device, dbidev.drm);
	if (IS_ERR(s))
		return PTR_ERR(s);

	dbidev = &s->dbidev;
	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "reset gpio failed\n");

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW | GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "dc gpio failed\n");

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	dbi->read_commands = NULL; /* SSD1306 typically write-only */

	dbidev->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(dbidev->regulator))
		return dev_err_probe(dev, PTR_ERR(dbidev->regulator), "power regulator failed\n");

	dbidev->io_regulator = devm_regulator_get(dev, "io");
	if (IS_ERR(dbidev->io_regulator))
		return dev_err_probe(dev, PTR_ERR(dbidev->io_regulator), "io regulator failed\n");

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	/* Load firmware */
	dbidev->driver_private = ssd1306_commands_from_fw(dev);
	if (IS_ERR(dbidev->driver_private))
		return PTR_ERR(dbidev->driver_private);

	ret = drm_mipi_dbi_dev_init(dbidev, &ssd1306_mode, DRM_FORMAT_RGB565, rotation, 0);
	if (ret)
		return ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = dbidev->mode.hdisplay;
	drm->mode_config.max_width = dbidev->mode.hdisplay;
	drm->mode_config.min_height = dbidev->mode.vdisplay;
	drm->mode_config.max_height = dbidev->mode.vdisplay;
	drm->mode_config.funcs = &ssd1306_mode_funcs;
	drm->mode_config.preferred_depth = 16;
	drm->mode_config.helper_private = &ssd1306_mode_helper;

	plane = &s->plane;
	ret = drm_universal_plane_init(drm, plane, 0, &ssd1306_plane_funcs,
				       ssd1306_formats, ARRAY_SIZE(ssd1306_formats),
				       ssd1306_modifiers, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(plane, &ssd1306_plane_helper);
	drm_plane_enable_fb_damage_clips(plane);

	crtc = &s->crtc;
	ret = drm_crtc_init_with_planes(drm, crtc, plane, NULL, &ssd1306_crtc_funcs, NULL);
	if (ret)
		return ret;
	drm_crtc_helper_add(crtc, &ssd1306_crtc_helper);

	enc = &s->encoder;
	ret = drm_encoder_init(drm, enc, &ssd1306_enc_funcs, DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ret;
	enc->possible_crtcs = drm_crtc_mask(crtc);

	conn = &s->connector;
	ret = drm_connector_init(drm, conn, &ssd1306_conn_funcs, DRM_MODE_CONNECTOR_SPI);
	if (ret)
		return ret;
	drm_connector_helper_add(conn, &ssd1306_conn_helper);

	ret = drm_connector_attach_encoder(conn, enc);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_client_setup(drm, NULL);

	return 0;
}

static void ssd1306_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);
	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void ssd1306_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver ssd1306_spi_driver = {
	.driver = {
		.name = "ssd1306",
		.of_match_table = ssd1306_of_match,
	},
	.id_table = ssd1306_id,
	.probe = ssd1306_probe,
	.remove = ssd1306_remove,
	.shutdown = ssd1306_shutdown,
};
module_spi_driver(ssd1306_spi_driver);

MODULE_DESCRIPTION("Solomon SSD1306 OLED DRM driver");
MODULE_AUTHOR("Soumyajyoti Sarkar");
MODULE_LICENSE("GPL");
