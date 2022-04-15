/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/adc.h>

#define ADC_DEVICE_NAME	DT_LABEL(DT_INST(0, ite_it8xxx2_adc))
#define ADC_RESOLUTION	10
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_1ST_CHANNEL_ID	0
#define ADC_2ND_CHANNEL_ID	1
#define ADC_3RD_CHANNEL_ID	2
#define ADC_4TH_CHANNEL_ID	3
#define ADC_5TH_CHANNEL_ID	4
#define ADC_6TH_CHANNEL_ID	5
#define ADC_7TH_CHANNEL_ID	6
#define ADC_8TH_CHANNEL_ID	7

#define BUFFER_SIZE  4
static int16_t m_sample_buffer[BUFFER_SIZE];
static int16_t m_sample_buffer1[1];

static const struct device *init_adc(void)
{
	int i, ret;
	const struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

	for (i = 0; i < 8; i++) {

		if (i==0 || i==3 || i==6 || i==7)
			continue;

		struct adc_channel_cfg channel_cfg = {
			.gain			  = ADC_GAIN,
			.reference		  = ADC_REFERENCE,
			.acquisition_time = ADC_ACQUISITION_TIME,
			.channel_id 	  = i,
			.differential	  = 0,
		};

		ret = adc_channel_setup(adc_dev, &channel_cfg);
		if (ret) {
			printk("adc_dev%i setup fail\n",i);
		}
	}

	return adc_dev;
}

void main(void)
{
	int i, j, ret;

	const struct device *adc_dev = init_adc();
	if (!adc_dev) {
		printk("adc dev init fail\n");
	}

	for(j=0; j < 50; j++) {


		printk("=====TEST2:Read one by one with delay=====\n");
		for (i = 0; i < 8; i++) {

			if (i==0 || i==3 || i==6 || i==7)
				continue;

			const struct adc_sequence sequence1 = {
				.channels    = BIT(i),
				.buffer      = m_sample_buffer1,
				.buffer_size = sizeof(m_sample_buffer1),
				.resolution  = ADC_RESOLUTION,
			};
			ret = adc_read(adc_dev, &sequence1);

			int32_t sample_value = m_sample_buffer1[0];
			printk("sample_value%d=%d\n",i, sample_value);

		}
	
		printk("=====TEST1:Read all=====\n");
		const struct adc_sequence sequence = {
			.channels    = BIT(ADC_2ND_CHANNEL_ID) |
				       BIT(ADC_3RD_CHANNEL_ID) |
				       BIT(ADC_5TH_CHANNEL_ID) |
				       BIT(ADC_6TH_CHANNEL_ID),
			.buffer      = m_sample_buffer,
			.buffer_size = sizeof(m_sample_buffer),
			.resolution  = ADC_RESOLUTION,
		};

		ret = adc_read(adc_dev, &sequence);

		for (i = 0; i < BUFFER_SIZE; i++) {
			int32_t sample_value = m_sample_buffer[i];
			printk("sample_value %d\n", sample_value);
		}
	}
}
