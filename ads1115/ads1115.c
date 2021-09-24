/*
 * Copyright (c) 2021 Jackychen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sgm_ads1115

#include <drivers/i2c.h>
#include <init.h>
#include <kernel.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <drivers/sensor.h>
#include <net/net_ip.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ads1115, LOG_LEVEL_INF);

#include <device.h>
#include <zephyr/types.h>

#define ADS1115_REG_CONVERSION	0x00
#define ADS1115_REG_CONFIG		0x01
#define ADS1115_REG_LO_TH		0x02
#define ADS1115_REG_HI_TH		0x03

#define ADS1115_AIN_INDEX_MAX	3

struct ads1115_data {
	struct k_timer 			*timer;
	struct k_work 			sample_worker;
	const  struct device 	*i2c_master;
	uint16_t 				i2c_slave_addr;

	uint16_t 				ain_value[4];
};

static int ads1115_init(const struct device *dev);
static int ads115_sample_fetch(const struct device *dev,enum sensor_channel chan);
static int ads1115_channel_get(const struct device *dev,enum sensor_channel chan,struct sensor_value *val);

static const struct sensor_driver_api ads1115_api_funcs = {
	.sample_fetch = ads115_sample_fetch,
	.channel_get = ads1115_channel_get,
};

#define ADS1115_DEV(idx) DT_NODELABEL(adccollector ## idx)

#define CREATE_COLLECTOR_DEVICE(idx)                                \
     static struct ads1115_data ads1115_data_##idx = {				\
			.i2c_slave_addr = DT_INST_REG_ADDR(idx),				\
	 };																\
     DEVICE_DT_DEFINE(ADS1115_DEV(idx),                             \
                     ads1115_init,                           		\
                     NULL,                                          \
                     &ads1115_data_##idx,                           \
                     NULL,                                  		\
                     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,      \
                     &ads1115_api_funcs);

#if DT_NODE_HAS_STATUS(ADS1115_DEV(0), okay)
CREATE_COLLECTOR_DEVICE(0)
#endif
#if DT_NODE_HAS_STATUS(ADS1115_DEV(1), okay)
CREATE_COLLECTOR_DEVICE(1)
#endif
#if DT_NODE_HAS_STATUS(ADS1115_DEV(2), okay)
CREATE_COLLECTOR_DEVICE(2)
#endif

static int ads1115_reg_write(struct ads1115_data *p_data, uint8_t reg, uint16_t *p_val)
{
	int err = 0;

	uint8_t wr_buff[3] = {0};
	uint16_t wr_value = *p_val;

	wr_buff[0] = reg;
	wr_buff[1] = (wr_value >> 8) & 0xff;
	wr_buff[2] = wr_value & 0xff;

	err = i2c_write(p_data->i2c_master, wr_buff, 3, p_data->i2c_slave_addr);

	if (err < 0) {
		LOG_ERR("0x%02x ads1115_reg_write reg 0x%02x error %d",p_data->i2c_slave_addr,reg,err);
		return err;
	}

	return 0;
}

static int ads1115_reg_read(struct ads1115_data *p_data, uint8_t reg, uint16_t *p_val)
{
	int err = 0;

	uint8_t wr_buff[2] = {0};
	uint8_t rd_buff[2] = {0};

	wr_buff[0] = reg;
	wr_buff[1] = (p_data->i2c_slave_addr << 1) | 0x01;

	err = i2c_write_read(p_data->i2c_master, p_data->i2c_slave_addr, wr_buff, 2, rd_buff, 2);
	if (err < 0) {
		LOG_ERR("0x%02x ads1115_reg_read reg 0x%02x error %d",p_data->i2c_slave_addr,reg,err);
		return err;
	}

	*p_val = (uint16_t)rd_buff[0] << 8;
	*p_val |= rd_buff[1];

	return 0;
}

static int ads1115_ainx_chan_change(struct ads1115_data *p_data, uint8_t ainx_index)
{
	int err = 0;
	uint16_t	config = 0;
	uint16_t    wr_value = 0;

	if(ainx_index > ADS1115_AIN_INDEX_MAX)
	{
		LOG_ERR("unvalid  ainx_index %d",ainx_index);
		return -EINVAL;
	}

	err = ads1115_reg_read(p_data, ADS1115_REG_CONFIG, &config);
	if(err < 0)
	{
		return -EINVAL;
	}

	config &= 0x8fff;
	wr_value = 0x04 + ainx_index;
	wr_value <<= 12;
	wr_value |= config;
	err = ads1115_reg_write(p_data, ADS1115_REG_CONFIG, &wr_value);
	if(err < 0)
	{
		return -EINVAL;
	}

	return 0;
}

static int ads1115_once_conversion_start(struct ads1115_data *p_data)
{
	int err = 0;
	uint16_t	config = 0;
	uint16_t    wr_value = 0;

	err = ads1115_reg_read(p_data, ADS1115_REG_CONFIG, &config);
	if(err < 0)
	{
		return -EINVAL;
	}

	wr_value = config | 0x8000;

	err = ads1115_reg_write(p_data, ADS1115_REG_CONFIG, &wr_value);
	if(err < 0)
	{
		return -EINVAL;
	}

	return 0;
}

static int ads1115_chan_change_with_start_conversion(struct ads1115_data *p_data,uint8_t ainx_index)
{
	int err = 0;
	uint16_t	config = 0;
	uint16_t    wr_value = 0;

	if(ainx_index > ADS1115_AIN_INDEX_MAX)
	{
		LOG_ERR("unvalid  ainx_index %d",ainx_index);
		return -EINVAL;
	}

	err = ads1115_reg_read(p_data, ADS1115_REG_CONFIG, &config);
	if(err < 0)
	{
		return -EINVAL;
	}

	config &= 0x8fff;
	wr_value = 0x04 + ainx_index;
	wr_value <<= 12;
	wr_value |= config;

	wr_value |= 0x8000;

	err = ads1115_reg_write(p_data, ADS1115_REG_CONFIG, &wr_value);
	if(err < 0)
	{
		return -EINVAL;
	}

	return 0;
}


static int ads1115_once_conversion_status_get(struct ads1115_data *p_data,uint16_t *p_status)
{
	int err = 0;

	err = ads1115_reg_read(p_data, ADS1115_REG_CONFIG, p_status);
	if(err < 0)
	{
		return -EINVAL;
	}

	*p_status &= 0x8000;

	return 0;
}

static int ads1115_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ads1115_data *p_ads1115_data = dev->data;

	switch (chan)
	{
		case SENSOR_CHAN_CH0:
			val->val1 = p_ads1115_data->ain_value[0];
			val->val2 = 0;
			break;

		case SENSOR_CHAN_CH1:
			val->val1 = p_ads1115_data->ain_value[1];
			val->val2 = 0;
			break;

		case SENSOR_CHAN_CH2:
			val->val1 = p_ads1115_data->ain_value[2];
			val->val2 = 0;
			break;

		case SENSOR_CHAN_CH3:
			val->val1 = p_ads1115_data->ain_value[3];
			val->val2 = 0;
			break;

		default:
			return -EINVAL;
			break;
	}

	return 0;
}

static int ads115_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
   	struct ads1115_data *p_ads1115_data = dev->data;
	uint8_t i = 0;
	uint16_t status = 0;
	uint8_t rty = 0;
	int err = 0;

	/* Collect each channel */
	for(i = 0 ; i < (ADS1115_AIN_INDEX_MAX + 1); i++)
	{
		ads1115_chan_change_with_start_conversion(p_ads1115_data, i);

		do
		{
			k_msleep(50);
			ads1115_once_conversion_status_get(p_ads1115_data , &status);
			//LOG_INF("try %d to get conversion status is 0x%04x",rty,status);
		}while( (!status) && (rty++ < 3) );

		rty = 0;
		if(status)
		{
			ads1115_reg_read(p_ads1115_data, ADS1115_REG_CONVERSION, &p_ads1115_data->ain_value[i]);
			LOG_INF("salver_addr 0x%02x get ain%d value 0x%04x",p_ads1115_data->i2c_slave_addr,i,p_ads1115_data->ain_value[i]);
		}
		else
		{
			p_ads1115_data->ain_value[i] = NULL;
			LOG_INF("salver_addr 0x%02x Can't get ain%d",p_ads1115_data->i2c_slave_addr,i);
		}
	}

   return 0;
}

static int ads1115_init(const struct device *dev)
{
	int err = 0;
	uint16_t	config = 0;
	uint16_t    wr_value = 0;

	struct ads1115_data *p_ads1115_data = dev->data;

	p_ads1115_data->i2c_master = device_get_binding(DT_INST_BUS_LABEL(0));
	if(!p_ads1115_data->i2c_master)
	{
		LOG_ERR("I2C master not found: %s",DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	// Try 100ms delay at the start
	k_msleep(100);

	err = ads1115_reg_read(p_ads1115_data, ADS1115_REG_CONFIG, &config);
	if(err < 0)
	{
		LOG_ERR("ads1115_init err!");
		return -EINVAL;
	}

	/* FSR = 6.144 */
	config &= 0xf1ff;
	wr_value = 0;
	wr_value <<= 9;
	wr_value |= config;
	wr_value &= 0x7fff;
	err = ads1115_reg_write(p_ads1115_data, ADS1115_REG_CONFIG, &wr_value);
	if(err < 0)
	{
		LOG_ERR("ads1115_init err!");
		return -EINVAL;
	}

	LOG_INF("ads1115_init 0x%04x finsh",p_ads1115_data->i2c_slave_addr);
	return err;
}




#if 0
static struct ads1115_data m_ads1115_data;

DEVICE_AND_API_INIT(ads1115, DT_INST_LABEL(0), ads1115_init, &m_ads1115_data,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &ads1115_api_funcs);
#endif


