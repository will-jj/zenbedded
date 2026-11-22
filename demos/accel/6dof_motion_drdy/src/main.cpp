/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zenbedded_transport/generated/interface_data.h>

#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zenbedded_common/network.hpp>
#include <zenbedded_rcl/zenbedded_client.hpp>

LOG_MODULE_REGISTER(imu_tier_2, LOG_LEVEL_INF);

using IMUClient = ZenbeddedClient<RawCodec<zenbedded_state_t>, RawCodec<zenbedded_command_t>>;

static struct sensor_trigger data_trigger;

/* Flag set from IMU device irq handler */
static volatile int irq_from_device;
K_SEM_DEFINE(drdy_sem, 0, 1);

/*
 * Get a device structure from a devicetree node from alias
 * "6dof_motion_drdy0".
 */
static const struct device *get_6dof_motion_device(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(6dof_motion_drdy0));

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}


static void handle_6dof_motion_drdy(const struct device *dev, const struct sensor_trigger *trig)
{
	if (trig->type == SENSOR_TRIG_DATA_READY) {
		int rc = sensor_sample_fetch_chan(dev, trig->chan);

		if (rc < 0) {
			printf("sample fetch failed: %d\n", rc);
			printf("cancelling trigger due to failure: %d\n", rc);
			(void)sensor_trigger_set(dev, trig, NULL);
			return;
		} else if (rc == 0) {
			irq_from_device = 1;
			k_sem_give(&drdy_sem);
		}
	}
}

uint32_t loop_freq = 100;     // hz


int main(void)
{
	const struct device *dev = get_6dof_motion_device();
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value temperature;



	LOG_INF("Starting IMU Tier 2 App");
	if (int net_err = zenbedded::common::init_network(); net_err != 0)
	{
		LOG_ERR("Failed to initialize network interface: %d", net_err);
		return net_err;
	}

	static IMUClient client;

	int ret = -1;

	#if defined(CONFIG_ZENBEDDED_COMMON_USB_CDC)
	if (zenbedded::common::is_usbd_ready())
	{
		char usb_loc[32];
		if (
		zenbedded::common::get_usb_peer_locator(
			ZENBEDDED_DEFAULT_LOCATOR, usb_loc, sizeof(usb_loc)) == 0)
		{
		LOG_INF("Attempting Zenoh connection over USB CDC ECM (%s)...", usb_loc);
		ret = client.init(loop_freq, usb_loc);
		if (ret != 0)
		{
			LOG_WRN("Failed to connect via USB CDC ECM (%s): %d", usb_loc, ret);
		}
		}
	}
	#endif

	#if defined(CONFIG_ZENBEDDED_COMMON_WIFI)
	if (ret != 0 && zenbedded::common::is_wifi_ready())
	{
		LOG_INF("Attempting Zenoh connection over Wi-Fi (%s)...", ZENBEDDED_DEFAULT_LOCATOR);
		ret = client.init(loop_freq, ZENBEDDED_DEFAULT_LOCATOR);
	}
	#endif

	if (ret != 0)
	{
		ret = client.init(loop_freq, ZENBEDDED_DEFAULT_LOCATOR);
	}

	if (ret != 0)
	{
		LOG_ERR("ZenbeddedClient.init failed with %d, aborting", ret);
		return ret;
	}

	if (dev == NULL) {
	return 0;
	}

	data_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	if (sensor_trigger_set(dev, &data_trigger, handle_6dof_motion_drdy) < 0) {
		printf("Cannot configure data trigger!!!\n");
		return 0;
	}

	k_sleep(K_MSEC(1000));


	zenbedded_state_t state = {0};
    zenbedded_command_t cmd{};


	while (1) {
		k_sem_take(&drdy_sem, K_FOREVER);
		//if (irq_from_device) {
			sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
			sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
			sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
			state.imu_angular_velocity__x    = sensor_value_to_double(&gyro[0]);
			state.imu_angular_velocity__y    = sensor_value_to_double(&gyro[1]);
			state.imu_angular_velocity__z    = sensor_value_to_double(&gyro[2]);
			state.imu_linear_acceleration__x = sensor_value_to_double(&accel[0]);
			state.imu_linear_acceleration__y = sensor_value_to_double(&accel[1]);
			state.imu_linear_acceleration__z = sensor_value_to_double(&accel[2]);
			//k_sleep(K_MSEC(1000 / loop_freq));
			// printf("hello %.4f", state.imu_angular_velocity_x);
			client.write_state(state);
			irq_from_device = 0;
		//}
	}
	return 0;
}
