/**
 ******************************************************************************
 * @addtogroup MiscTargets Misc Targets
 * @{
 * @addtogroup Naze32Pro Tau Labs Naze32Pro support files
 * @{
 *
 * @file       pios_board.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2014
 * @brief      The board specific initialization routines
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */

#include "board_hw_defs.c"

#include <pios.h>
#include <openpilot.h>
#include <uavobjectsinit.h>
#include "hwnaze32pro.h"
#include "manualcontrolsettings.h"
#include "modulesettings.h"

///////////////////////////////////////////////////////////////////////////////

/**
 * Configuration for the MPU6000 chip
 */
#if defined(PIOS_INCLUDE_MPU6000)
#include "pios_mpu6000.h"
static const struct pios_exti_cfg pios_exti_mpu6000_cfg __exti_config = {
	.vector = PIOS_MPU6000_IRQHandler,
	.line = EXTI_Line1,
	.pin = {
		.gpio = GPIOB,
		.init = {
			.GPIO_Pin   = GPIO_Pin_1,
			.GPIO_Speed = GPIO_Speed_50MHz,
			.GPIO_Mode  = GPIO_Mode_IN,
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_PuPd  = GPIO_PuPd_NOPULL,
		},
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = EXTI1_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.exti = {
		.init = {
			.EXTI_Line    = EXTI_Line1, // matches above GPIO pin
			.EXTI_Mode    = EXTI_Mode_Interrupt,
			.EXTI_Trigger = EXTI_Trigger_Rising,
			.EXTI_LineCmd = ENABLE,
		},
	},
};

static const struct pios_mpu60x0_cfg pios_mpu6000_cfg = {
	.exti_cfg           = &pios_exti_mpu6000_cfg,
	.default_samplerate = 666,
	.interrupt_cfg      = PIOS_MPU60X0_INT_CLR_ANYRD,
	.interrupt_en       = PIOS_MPU60X0_INTEN_DATA_RDY,
	.User_ctl           = PIOS_MPU60X0_USERCTL_DIS_I2C,
	.Pwr_mgmt_clk       = PIOS_MPU60X0_PWRMGMT_PLL_Z_CLK,
	.default_filter     = PIOS_MPU60X0_LOWPASS_256_HZ,
	.orientation        = PIOS_MPU60X0_TOP_270DEG
};
#endif /* PIOS_INCLUDE_MPU6000 */

///////////////////////////////////////////////////////////////////////////////

/**
 * Configuration of the internal HMC5983 chip
 */
#if defined(PIOS_INCLUDE_HMC5983)
#include "pios_hmc5983.h"
static const struct pios_exti_cfg pios_exti_hmc5983_cfg __exti_config = {
	.vector = PIOS_HMC5983_IRQHandler,
	.line = EXTI_Line0,
	.pin = {
		.gpio = GPIOB,
		.init = {
			.GPIO_Pin   = GPIO_Pin_0,
			.GPIO_Speed = GPIO_Speed_50MHz,
			.GPIO_Mode  = GPIO_Mode_IN,
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_PuPd  = GPIO_PuPd_NOPULL,
		},
	},
	.irq = {
		.init = {
			.NVIC_IRQChannel                   = EXTI0_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_LOW,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
		},
	},
	.exti = {
		.init = {
			.EXTI_Line    = EXTI_Line0, // matches above GPIO pin
			.EXTI_Mode    = EXTI_Mode_Interrupt,
			.EXTI_Trigger = EXTI_Trigger_Rising,
			.EXTI_LineCmd = ENABLE,
		},
	},
};

static const struct pios_hmc5983_cfg pios_hmc5983_cfg = {
	.exti_cfg    = &pios_exti_hmc5983_cfg,
	.Averaging   = PIOS_HMC5983_AVERAGING_8,
	.M_ODR       = PIOS_HMC5983_ODR_75,
	.Meas_Conf   = PIOS_HMC5983_MEASCONF_NORMAL,
	.Gain        = PIOS_HMC5983_GAIN_1_9,
	.Mode        = PIOS_HMC5983_MODE_CONTINUOUS,
	.orientation = PIOS_HMC5983_TOP_180DEG,
};

#endif /* PIOS_INCLUDE_HMC5983 */

///////////////////////////////////////////////////////////////////////////////

/**
 * Configuration for the external HMC5883 chip
 */
#if defined(PIOS_INCLUDE_HMC5883)
#include "pios_hmc5883_priv.h"
static const struct pios_hmc5883_cfg pios_hmc5883_external_cfg = {
	.exti_cfg            = NULL,
	.M_ODR               = PIOS_HMC5883_ODR_75,
	.Meas_Conf           = PIOS_HMC5883_MEASCONF_NORMAL,
	.Gain                = PIOS_HMC5883_GAIN_1_9,
	.Mode                = PIOS_HMC5883_MODE_SINGLE,
	.Default_Orientation = PIOS_HMC5883_TOP_0DEG,
};
#endif /* PIOS_INCLUDE_HMC5883 */

///////////////////////////////////////////////////////////////////////////////

/**
 * Configuration for the MS5611 chip
 */
#if defined(PIOS_INCLUDE_MS5611_SPI)
#include "pios_ms5611_priv.h"
static const struct pios_ms5611_cfg pios_ms5611_cfg = {
	.oversampling             = MS5611_OSR_4096,
	.temperature_interleaving = 1,
};
#endif /* PIOS_INCLUDE_MS5611_SPI */

///////////////////////////////////////////////////////////////////////////////

/* One slot per selectable receiver group.
 *  eg. PWM, PPM, GCS, SPEKTRUM1, SPEKTRUM2, SBUS
 * NOTE: No slot in this map for NONE.
 */
uintptr_t pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE];

#define PIOS_COM_TELEM_RF_RX_BUF_LEN 512
#define PIOS_COM_TELEM_RF_TX_BUF_LEN 512

#define PIOS_COM_GPS_RX_BUF_LEN 32
#define PIOS_COM_GPS_TX_BUF_LEN 16

#define PIOS_COM_TELEM_USB_RX_BUF_LEN 65
#define PIOS_COM_TELEM_USB_TX_BUF_LEN 65

#define PIOS_COM_CAN_RX_BUF_LEN 256
#define PIOS_COM_CAN_TX_BUF_LEN 256

#define PIOS_COM_BRIDGE_RX_BUF_LEN 65
#define PIOS_COM_BRIDGE_TX_BUF_LEN 12

#define PIOS_COM_MAVLINK_TX_BUF_LEN 32
#define PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN 19

#define PIOS_COM_HOTT_RX_BUF_LEN 16
#define PIOS_COM_HOTT_TX_BUF_LEN 16

#define PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN 128

#define PIOS_COM_FRSKYSPORT_TX_BUF_LEN 16
#define PIOS_COM_FRSKYSPORT_RX_BUF_LEN 16

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
#define PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN 40
uintptr_t pios_com_debug_id;
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */

uintptr_t pios_com_aux_id;
uintptr_t pios_com_gps_id;
uintptr_t pios_com_telem_usb_id;
uintptr_t pios_com_telem_rf_id;
uintptr_t pios_com_vcp_id;
uintptr_t pios_com_bridge_id;
uintptr_t pios_com_mavlink_id;
uintptr_t pios_com_hott_id;
uintptr_t pios_com_frsky_sensor_hub_id;
uintptr_t pios_com_lighttelemetry_id;
uintptr_t pios_com_can_id;
uintptr_t pios_com_frsky_sport_id;

uintptr_t pios_uavo_settings_fs_id;
uintptr_t pios_waypoints_settings_fs_id;

uintptr_t pios_internal_adc_id;

uintptr_t pios_can_id;

///////////////////////////////////////////////////////////////////////////////

/*
 * Setup a com port based on the passed cfg, driver and buffer sizes. tx size of -1 make the port rx only
 */
#if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
static void PIOS_Board_configure_com (const struct pios_usart_cfg *usart_port_cfg, size_t rx_buf_len, size_t tx_buf_len,
		const struct pios_com_driver *com_driver, uintptr_t *pios_com_id)
{
	uintptr_t pios_usart_id;
	if (PIOS_USART_Init(&pios_usart_id, usart_port_cfg)) {
		PIOS_Assert(0);
	}

	uint8_t * rx_buffer;
	if (rx_buf_len > 0) {
		rx_buffer = (uint8_t *) PIOS_malloc(rx_buf_len);
		PIOS_Assert(rx_buffer);
	} else {
		rx_buffer = NULL;
	}

	uint8_t * tx_buffer;
	if (tx_buf_len > 0) {
		tx_buffer = (uint8_t *) PIOS_malloc(tx_buf_len);
		PIOS_Assert(tx_buffer);
	} else {
		tx_buffer = NULL;
	}

	if (PIOS_COM_Init(pios_com_id, com_driver, pios_usart_id,
				rx_buffer, rx_buf_len,
				tx_buffer, tx_buf_len)) {
		PIOS_Assert(0);
	}
}
#endif	/* PIOS_INCLUDE_USART && PIOS_INCLUDE_COM */

///////////////////////////////////////////////////////////////////////////////

#ifdef PIOS_INCLUDE_DSM
static void PIOS_Board_configure_dsm(const struct pios_usart_cfg *pios_usart_dsm_cfg, const struct pios_dsm_cfg *pios_dsm_cfg,
		const struct pios_com_driver *pios_usart_com_driver,enum pios_dsm_proto *proto,
		ManualControlSettingsChannelGroupsOptions channelgroup,uint8_t *bind)
{
	uintptr_t pios_usart_dsm_id;
	if (PIOS_USART_Init(&pios_usart_dsm_id, pios_usart_dsm_cfg)) {
		PIOS_Assert(0);
	}

	uintptr_t pios_dsm_id;
	if (PIOS_DSM_Init(&pios_dsm_id, pios_dsm_cfg, pios_usart_com_driver,
			pios_usart_dsm_id, *proto, *bind)) {
		PIOS_Assert(0);
	}

	uintptr_t pios_dsm_rcvr_id;
	if (PIOS_RCVR_Init(&pios_dsm_rcvr_id, &pios_dsm_rcvr_driver, pios_dsm_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[channelgroup] = pios_dsm_rcvr_id;
}
#endif

///////////////////////////////////////////////////////////////////////////////

#ifdef PIOS_INCLUDE_HSUM
static void PIOS_Board_configure_hsum(const struct pios_usart_cfg *pios_usart_hsum_cfg,
		const struct pios_com_driver *pios_usart_com_driver,enum pios_hsum_proto *proto,
		ManualControlSettingsChannelGroupsOptions channelgroup)
{
	uintptr_t pios_usart_hsum_id;
	if (PIOS_USART_Init(&pios_usart_hsum_id, pios_usart_hsum_cfg)) {
		PIOS_Assert(0);
	}

	uintptr_t pios_hsum_id;
	if (PIOS_HSUM_Init(&pios_hsum_id, pios_usart_com_driver,
			  pios_usart_hsum_id, *proto)) {
		PIOS_Assert(0);
	}

	uintptr_t pios_hsum_rcvr_id;
	if (PIOS_RCVR_Init(&pios_hsum_rcvr_id, &pios_hsum_rcvr_driver, pios_hsum_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[channelgroup] = pios_hsum_rcvr_id;
}
#endif

///////////////////////////////////////////////////////////////////////////////

/**
 * Indicate a target-specific error code when a component fails to initialize
 *  1 pulse:  MPU6000 - PIOS_MPU6000_Init failed
 *  2 pulses: MPU6000 - PIOS_MPU6000_Test failed
 *  3 pulses: HMC5983 - PIOS_HMC5983_Init failed
 *  4 pulses: HMC5983 - PIOS_HMC5983_Test failed
 *  5 pulses: Flash   - PIOS_Flash_Internal_Init failed
 *                    - PIOS_FLASHFS_Logfs_Init failed (settings)
 *                    - PIOS_FLASHFS_Logfs_Init failed (waypoints)
 *  6 pulses: I2C     - External I2C bus locked
 *  7 pulses: HMC5883 - PIOS_HMC5883_Init failed
 *  8 pulses: HMC5883 - PIOS_HMC5883_Test failed
 *  9 pulses: ADC     - PIOS_INTERNAL_ADC_Init failed
 * 10 pulses: ADC     - PIOS_ADC_Init failed
 */

void panic(int32_t code)
{
	while(1)
	{
		for (int32_t i = 0; i < code; i++)
		{
			PIOS_WDG_Clear();
			PIOS_LED_Toggle(PIOS_LED_ALARM);
			PIOS_DELAY_WaitmS(200);
			PIOS_WDG_Clear();
			PIOS_LED_Toggle(PIOS_LED_ALARM);
			PIOS_DELAY_WaitmS(200);
		}

		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(200);
		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(200);
		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(100);
	}
}

///////////////////////////////////////////////////////////////////////////////

/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */

#include <pios_board_info.h>

void PIOS_Board_Init(void) {

	/* Delay system */
	PIOS_DELAY_Init();

	const struct pios_board_info * bdinfo = &pios_board_info_blob;

    ///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_LED)
	const struct pios_led_cfg * led_cfg = PIOS_BOARD_HW_DEFS_GetLedCfg(bdinfo->board_rev);
	PIOS_Assert(led_cfg);
	PIOS_LED_Init(led_cfg);
    #endif	/* PIOS_INCLUDE_LED */

    ///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_SPI)
	if (PIOS_SPI_Init(&pios_spi_internal_id, &pios_spi_internal_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
    #endif

    ///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_I2C)
    RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);  // Switch I2C clock from HSI to SYSCLK.  I2C1 is the external I2C port.
                                           // Might be better to make this call in the beginning of PIOS_I2C_Init,
                                           // but that would force changes into the other STM32F3 targets.

	if (PIOS_I2C_Init(&pios_i2c_external_id, &pios_i2c_external_cfg)) {
		PIOS_DEBUG_Assert(0);
	}
	if (PIOS_I2C_CheckClear(pios_i2c_external_id) != 0)
		panic(6);
    #endif

    ///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_FLASH)
	/* Inititialize all flash drivers */
	if (PIOS_Flash_Internal_Init(&pios_internal_flash_id, &flash_internal_cfg) != 0)
		panic(5);

	/* Register the partition table */
	const struct pios_flash_partition * flash_partition_table;
	uint32_t num_partitions;
	flash_partition_table = PIOS_BOARD_HW_DEFS_GetPartitionTable(bdinfo->board_rev, &num_partitions);
	PIOS_FLASH_register_partition_table(flash_partition_table, num_partitions);

	/* Mount all filesystems */
	if (PIOS_FLASHFS_Logfs_Init(&pios_uavo_settings_fs_id, &flashfs_internal_settings_cfg, FLASH_PARTITION_LABEL_SETTINGS) != 0)
		panic(5);
	if (PIOS_FLASHFS_Logfs_Init(&pios_waypoints_settings_fs_id, &flashfs_internal_waypoints_cfg, FLASH_PARTITION_LABEL_WAYPOINTS) != 0)
		panic(5);

    #endif	/* PIOS_INCLUDE_FLASH */

    ///////////////////////////////////////////////////////////////////////////

    /* Initialize the task monitor library */
	TaskMonitorInitialize();

	/* Initialize UAVObject libraries */
	EventDispatcherInitialize();
	UAVObjInitialize();

	/* Initialize the alarms library */
	AlarmsInitialize();

	HwNaze32ProInitialize();
	ModuleSettingsInitialize();

    ///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_RTC)
 	/* Initialize the real-time clock and its associated tick */
	PIOS_RTC_Init(&pios_rtc_main_cfg);
    #endif

    ///////////////////////////////////////////////////////////////////////////

    #ifndef ERASE_FLASH
	/* Initialize watchdog as early as possible to catch faults during init
	 * but do it only if there is no debugger connected
	 */
	if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) == 0) {
		PIOS_WDG_Init();
	}
    #endif

    ///////////////////////////////////////////////////////////////////////////

    /* Set up pulse timers */

	//inputs
    PIOS_TIM_InitClock(&tim_1_cfg);
    PIOS_TIM_InitClock(&tim_16_cfg);

	//outputs
	PIOS_TIM_InitClock(&tim_2_cfg);
	PIOS_TIM_InitClock(&tim_15_cfg);
	PIOS_TIM_InitClock(&tim_3_cfg);
	PIOS_TIM_InitClock(&tim_4_cfg);

	/* IAP System Setup */
	PIOS_IAP_Init();
	uint16_t boot_count = PIOS_IAP_ReadBootCount();
	if (boot_count < 3) {
		PIOS_IAP_WriteBootCount(++boot_count);
		AlarmsClear(SYSTEMALARMS_ALARM_BOOTFAULT);
	} else {
		/* Too many failed boot attempts, force hw config to defaults */
		HwNaze32ProSetDefaults(HwNaze32ProHandle(), 0);
		ModuleSettingsSetDefaults(ModuleSettingsHandle(),0);
		AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
	}

    ///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_USB)
	/* Initialize board specific USB data */
	PIOS_USB_BOARD_DATA_Init();

	/* Flags to determine if various USB interfaces are advertised */
	bool usb_hid_present = false;

    #if defined(PIOS_INCLUDE_USB_CDC)
	bool usb_cdc_present = false;
	if (PIOS_USB_DESC_HID_CDC_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
	usb_cdc_present = true;
    #else
	if (PIOS_USB_DESC_HID_ONLY_Init()) {
		PIOS_Assert(0);
	}
	usb_hid_present = true;
    #endif

	///////////////////////////////////////////////////////////////////////////

	uintptr_t pios_usb_id;
	PIOS_USB_Init(&pios_usb_id, PIOS_BOARD_HW_DEFS_GetUsbCfg(bdinfo->board_rev));

    ///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_USB_CDC)

	uint8_t hw_usb_vcpport;

	/* Configure the USB VCP port */
	HwNaze32ProUSB_VCPPortGet(&hw_usb_vcpport);

	if (!usb_cdc_present) {
		/* Force VCP port function to disabled if we haven't advertised VCP in our USB descriptor */
		hw_usb_vcpport = HWNAZE32PRO_USB_VCPPORT_DISABLED;
	}

	switch (hw_usb_vcpport)

	{
	case HWNAZE32PRO_USB_VCPPORT_DISABLED:
		break;

	case HWNAZE32PRO_USB_VCPPORT_USBTELEMETRY:
        #if defined(PIOS_INCLUDE_COM)
		{
			uintptr_t pios_usb_cdc_id;
			if (PIOS_USB_CDC_Init(&pios_usb_cdc_id, &pios_usb_cdc_cfg, pios_usb_id)) {
				PIOS_Assert(0);
			}
			uint8_t * rx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_TELEM_USB_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_TELEM_USB_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_telem_usb_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						rx_buffer, PIOS_COM_TELEM_USB_RX_BUF_LEN,
						tx_buffer, PIOS_COM_TELEM_USB_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
        #endif	/* PIOS_INCLUDE_COM */
		break;

	case HWNAZE32PRO_USB_VCPPORT_COMBRIDGE:
        #if defined(PIOS_INCLUDE_COM)
		{
			uintptr_t pios_usb_cdc_id;
			if (PIOS_USB_CDC_Init(&pios_usb_cdc_id, &pios_usb_cdc_cfg, pios_usb_id)) {
				PIOS_Assert(0);
			}
			uint8_t * rx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_BRIDGE_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_BRIDGE_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_vcp_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						rx_buffer, PIOS_COM_BRIDGE_RX_BUF_LEN,
						tx_buffer, PIOS_COM_BRIDGE_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
        #endif	/* PIOS_INCLUDE_COM */
		break;

	case HWNAZE32PRO_USB_VCPPORT_DEBUGCONSOLE:
        #if defined(PIOS_INCLUDE_COM)
        #if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
		{
			uintptr_t pios_usb_cdc_id;
			if (PIOS_USB_CDC_Init(&pios_usb_cdc_id, &pios_usb_cdc_cfg, pios_usb_id)) {
				PIOS_Assert(0);
			}
			uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_debug_id, &pios_usb_cdc_com_driver, pios_usb_cdc_id,
						NULL, 0,
						tx_buffer, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
        #endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
        #endif	/* PIOS_INCLUDE_COM */
		break;
	}
    #endif	/* PIOS_INCLUDE_USB_CDC */

    ///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_USB_HID)

	/* Configure the usb HID port */
	uint8_t hw_usb_hidport;

	HwNaze32ProUSB_HIDPortGet(&hw_usb_hidport);

	if (!usb_hid_present) {
		/* Force HID port function to disabled if we haven't advertised HID in our USB descriptor */
		hw_usb_hidport = HWNAZE32PRO_USB_HIDPORT_DISABLED;
	}

	switch (hw_usb_hidport)
	{
	case HWNAZE32PRO_USB_HIDPORT_DISABLED:
		break;

	case HWNAZE32PRO_USB_HIDPORT_USBTELEMETRY:
        #if defined(PIOS_INCLUDE_COM)
		{
			uintptr_t pios_usb_hid_id;
			if (PIOS_USB_HID_Init(&pios_usb_hid_id, &pios_usb_hid_cfg, pios_usb_id)) {
				PIOS_Assert(0);
			}
			uint8_t * rx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_TELEM_USB_RX_BUF_LEN);
			uint8_t * tx_buffer = (uint8_t *) PIOS_malloc(PIOS_COM_TELEM_USB_TX_BUF_LEN);
			PIOS_Assert(rx_buffer);
			PIOS_Assert(tx_buffer);
			if (PIOS_COM_Init(&pios_com_telem_usb_id, &pios_usb_hid_com_driver, pios_usb_hid_id,
						rx_buffer, PIOS_COM_TELEM_USB_RX_BUF_LEN,
						tx_buffer, PIOS_COM_TELEM_USB_TX_BUF_LEN)) {
				PIOS_Assert(0);
			}
		}
        #endif	/* PIOS_INCLUDE_COM */
		break;
	}

    #endif	/* PIOS_INCLUDE_USB_HID */
    #endif	/* PIOS_INCLUDE_USB */

	///////////////////////////////////////////////////////////////////////////

	/* Configure the IO ports */

	/* UART1 Port */
	uint8_t hw_uart1;

	HwNaze32ProUart1Get(&hw_uart1);

	switch (hw_uart1)
	{
	case HWNAZE32PRO_UART1_DISABLED:
		break;

	case HWNAZE32PRO_UART1_TELEMETRY:
        #if defined(PIOS_INCLUDE_TELEMETRY_RF) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart1_cfg, PIOS_COM_TELEM_RF_RX_BUF_LEN, PIOS_COM_TELEM_RF_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_telem_rf_id);
        #endif /* PIOS_INCLUDE_TELEMETRY_RF */
		break;

	case HWNAZE32PRO_UART1_GPS:
        #if defined(PIOS_INCLUDE_GPS) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart1_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_GPS_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
        #endif
		break;

	case HWNAZE32PRO_UART1_DEBUGCONSOLE:
        #if defined(PIOS_INCLUDE_DEBUG_CONSOLE) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart1_cfg, 0, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_aux_id);
        #endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
		break;

	case HWNAZE32PRO_UART1_COMBRIDGE:
        #if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart1_cfg, PIOS_COM_BRIDGE_RX_BUF_LEN, PIOS_COM_BRIDGE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_bridge_id);
        #endif
		break;

	case HWNAZE32PRO_UART1_MAVLINKTX:
        #if defined(PIOS_INCLUDE_MAVLINK)
		PIOS_Board_configure_com(&pios_uart1_cfg, 0, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_mavlink_id);
        #endif  /* PIOS_INCLUDE_MAVLINK */
		break;

	case HWNAZE32PRO_UART1_MAVLINKTX_GPS_RX:
        #if defined(PIOS_INCLUDE_GPS)
        #if defined(PIOS_INCLUDE_MAVLINK)
		PIOS_Board_configure_com(&pios_uart1_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
		pios_com_mavlink_id = pios_com_gps_id;
        #endif  /* PIOS_INCLUDE_MAVLINK */
        #endif  /* PIOS_INCLUDE_GPS */
		break;

	case HWNAZE32PRO_UART1_HOTTTELEMETRY:
        #if defined(PIOS_INCLUDE_HOTT) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart1_cfg, PIOS_COM_HOTT_RX_BUF_LEN, PIOS_COM_HOTT_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_hott_id);
        #endif /* PIOS_INCLUDE_HOTT */
		break;

	case HWNAZE32PRO_UART1_FRSKYSENSORHUB:
        #if defined(PIOS_INCLUDE_FRSKY_SENSOR_HUB) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart1_cfg, 0, PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sensor_hub_id);
        #endif /* PIOS_INCLUDE_FRSKY_SENSOR_HUB */
		break;

	case HWNAZE32PRO_UART1_LIGHTTELEMETRYTX:
        #if defined(PIOS_INCLUDE_LIGHTTELEMETRY)
	    PIOS_Board_configure_com(&pios_uart1_cfg, 0, PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_lighttelemetry_id);
        #endif
		break;

	case HWNAZE32PRO_UART1_FRSKYSPORTTELEMETRY:
        #if defined(PIOS_INCLUDE_FRSKY_SPORT_TELEMETRY)
		PIOS_Board_configure_com(&pios_uart1_sport_cfg, PIOS_COM_FRSKYSPORT_RX_BUF_LEN, PIOS_COM_FRSKYSPORT_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sport_id);
        #endif /* PIOS_INCLUDE_FRSKY_SPORT_TELEMETRY */
		break;
	}

	///////////////////////////////////////////////////////////////////////////

	/* UART2 Port */
	uint8_t hw_uart2;

	HwNaze32ProUart2Get(&hw_uart2);

	switch (hw_uart2)
	{
	case HWNAZE32PRO_UART2_DISABLED:
		break;

	case HWNAZE32PRO_UART2_TELEMETRY:
        #if defined(PIOS_INCLUDE_TELEMETRY_RF) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart2_cfg, PIOS_COM_TELEM_RF_RX_BUF_LEN, PIOS_COM_TELEM_RF_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_telem_rf_id);
        #endif /* PIOS_INCLUDE_TELEMETRY_RF */
		break;

	case HWNAZE32PRO_UART2_GPS:
        #if defined(PIOS_INCLUDE_GPS) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart2_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_GPS_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
        #endif
		break;

	case HWNAZE32PRO_UART2_DEBUGCONSOLE:
        #if defined(PIOS_INCLUDE_DEBUG_CONSOLE) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart2_cfg, 0, PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_aux_id);
        #endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */
		break;

	case HWNAZE32PRO_UART2_COMBRIDGE:
        #if defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart2_cfg, PIOS_COM_BRIDGE_RX_BUF_LEN, PIOS_COM_BRIDGE_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_bridge_id);
        #endif
		break;

	case HWNAZE32PRO_UART2_MAVLINKTX:
        #if defined(PIOS_INCLUDE_MAVLINK)
		PIOS_Board_configure_com(&pios_uart2_cfg, 0, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_mavlink_id);
        #endif  /* PIOS_INCLUDE_MAVLINK */
		break;

	case HWNAZE32PRO_UART2_MAVLINKTX_GPS_RX:
        #if defined(PIOS_INCLUDE_GPS)
        #if defined(PIOS_INCLUDE_MAVLINK)
		PIOS_Board_configure_com(&pios_uart2_cfg, PIOS_COM_GPS_RX_BUF_LEN, PIOS_COM_MAVLINK_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_gps_id);
		pios_com_mavlink_id = pios_com_gps_id;
        #endif  /* PIOS_INCLUDE_MAVLINK */
        #endif  /* PIOS_INCLUDE_GPS */
		break;

	case HWNAZE32PRO_UART2_HOTTTELEMETRY:
        #if defined(PIOS_INCLUDE_HOTT) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart2_cfg, PIOS_COM_HOTT_RX_BUF_LEN, PIOS_COM_HOTT_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_hott_id);
        #endif /* PIOS_INCLUDE_HOTT */
		break;

	case HWNAZE32PRO_UART2_FRSKYSENSORHUB:
        #if defined(PIOS_INCLUDE_FRSKY_SENSOR_HUB) && defined(PIOS_INCLUDE_USART) && defined(PIOS_INCLUDE_COM)
		PIOS_Board_configure_com(&pios_uart2_cfg, 0, PIOS_COM_FRSKYSENSORHUB_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sensor_hub_id);
        #endif /* PIOS_INCLUDE_FRSKY_SENSOR_HUB */
		break;

	case HWNAZE32PRO_UART2_LIGHTTELEMETRYTX:
        #if defined(PIOS_INCLUDE_LIGHTTELEMETRY)
	    PIOS_Board_configure_com(&pios_uart2_cfg, 0, PIOS_COM_LIGHTTELEMETRY_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_lighttelemetry_id);
        #endif /* PIOS_INCLUDE_LIGHTTELEMETRY */
		break;

	case HWNAZE32PRO_UART2_FRSKYSPORTTELEMETRY:
        #if defined(PIOS_INCLUDE_FRSKY_SPORT_TELEMETRY)
		PIOS_Board_configure_com(&pios_uart2_sport_cfg, PIOS_COM_FRSKYSPORT_RX_BUF_LEN, PIOS_COM_FRSKYSPORT_TX_BUF_LEN, &pios_usart_com_driver, &pios_com_frsky_sport_id);
        #endif /* PIOS_INCLUDE_FRSKY_SPORT_TELEMETRY */
		break;
	}

	///////////////////////////////////////////////////////////////////////////

	/* Configure the rcvr port */
	uint8_t hw_rcvrport;

	HwNaze32ProRcvrPortGet(&hw_rcvrport);

	switch (hw_rcvrport)
	{
	case HWNAZE32PRO_RCVRPORT_DISABLED:
		break;

	case HWNAZE32PRO_RCVRPORT_PPM:
        #if defined(PIOS_INCLUDE_PPM)
		{
			uintptr_t pios_ppm_id;
			PIOS_PPM_Init(&pios_ppm_id, &pios_ppm_cfg);

			uintptr_t pios_ppm_rcvr_id;
			if (PIOS_RCVR_Init(&pios_ppm_rcvr_id, &pios_ppm_rcvr_driver, pios_ppm_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PPM] = pios_ppm_rcvr_id;
		}
        #endif	/* PIOS_INCLUDE_PPM */
		break;

	case HWNAZE32PRO_RCVRPORT_DSM2:
	case HWNAZE32PRO_RCVRPORT_DSMX10BIT:
	case HWNAZE32PRO_RCVRPORT_DSMX11BIT:
        #if defined(PIOS_INCLUDE_DSM)
		{
			enum pios_dsm_proto proto;

			switch (hw_rcvrport)
			{
			case HWNAZE32PRO_RCVRPORT_DSM2:
				proto = PIOS_DSM_PROTO_DSM2;
				break;

			case HWNAZE32PRO_RCVRPORT_DSMX10BIT:
				proto = PIOS_DSM_PROTO_DSMX10BIT;
				break;

			case HWNAZE32PRO_RCVRPORT_DSMX11BIT:
				proto = PIOS_DSM_PROTO_DSMX11BIT;
				break;

			default:
				PIOS_Assert(0);
				break;
			}
			uint8_t hw_DSMxBind;

			HwNaze32ProDSMxBindGet(&hw_DSMxBind);

			PIOS_Board_configure_dsm(&pios_rcvr_dsm_hsum_cfg, &pios_rcvr_dsm_bind_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_DSMMAINPORT, &hw_DSMxBind);
		}
        #endif	/* PIOS_INCLUDE_DSM */
		break;

	case HWNAZE32PRO_RCVRPORT_HOTTSUMD:
	case HWNAZE32PRO_RCVRPORT_HOTTSUMH:
        #if defined(PIOS_INCLUDE_HSUM)
		{
			enum pios_hsum_proto proto;

			switch (hw_rcvrport)
			{
			case HWNAZE32PRO_RCVRPORT_HOTTSUMD:
				proto = PIOS_HSUM_PROTO_SUMD;
				break;

			case HWNAZE32PRO_RCVRPORT_HOTTSUMH:
				proto = PIOS_HSUM_PROTO_SUMH;
				break;

			default:
				PIOS_Assert(0);
				break;
			}
			PIOS_Board_configure_hsum(&pios_rcvr_dsm_hsum_cfg, &pios_usart_com_driver,
				&proto, MANUALCONTROLSETTINGS_CHANNELGROUPS_HOTTSUM);
		}
        #endif	/* PIOS_INCLUDE_HSUM */
		break;

	case HWNAZE32PRO_RCVRPORT_SBUS:
        #if defined(PIOS_INCLUDE_SBUS) && defined(PIOS_INCLUDE_USART)
		{
			uintptr_t pios_usart_sbus_id;
			if (PIOS_USART_Init(&pios_usart_sbus_id, &pios_rcvr_sbus_cfg)) {
				PIOS_Assert(0);
			}
			uintptr_t pios_sbus_id;
			if (PIOS_SBus_Init(&pios_sbus_id, &pios_rcvr_sbus_aux_cfg, &pios_usart_com_driver, pios_usart_sbus_id)) {
				PIOS_Assert(0);
			}
			uintptr_t pios_sbus_rcvr_id;
			if (PIOS_RCVR_Init(&pios_sbus_rcvr_id, &pios_sbus_rcvr_driver, pios_sbus_id)) {
				PIOS_Assert(0);
			}
			pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_SBUS] = pios_sbus_rcvr_id;
		}
        #endif	/* PIOS_INCLUDE_SBUS */
		break;
		break;
	}

    ///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_GCSRCVR)
	GCSReceiverInitialize();
	uintptr_t pios_gcsrcvr_id;
	PIOS_GCSRCVR_Init(&pios_gcsrcvr_id);
	uintptr_t pios_gcsrcvr_rcvr_id;
	if (PIOS_RCVR_Init(&pios_gcsrcvr_rcvr_id, &pios_gcsrcvr_rcvr_driver, pios_gcsrcvr_id)) {
		PIOS_Assert(0);
	}
	pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_GCS] = pios_gcsrcvr_rcvr_id;
    #endif	/* PIOS_INCLUDE_GCSRCVR */

    ///////////////////////////////////////////////////////////////////////////

    #if defined PIOS_INCLUDE_ADC && defined PIOS_INCLUDE_PWM && defined PIOS_INCLUDE_SERVO

    uint8_t hw_outport;
	uint8_t number_of_pwm_outputs;
	uint8_t number_of_adc_ports;
	bool use_pwm_in;

	HwNaze32ProOutPortGet(&hw_outport);

	switch (hw_outport)
	{
	case HWNAZE32PRO_OUTPORT_PWM8:
		number_of_pwm_outputs = 8;
		use_pwm_in            = false;
		number_of_adc_ports   = 0;
		break;

	case HWNAZE32PRO_OUTPORT_PWM8ADC1:
		number_of_pwm_outputs = 8;
		use_pwm_in            = false;
		number_of_adc_ports   = 1;
		break;

	case HWNAZE32PRO_OUTPORT_PWM8ADC2:
		number_of_pwm_outputs = 8;
		use_pwm_in            = false;
		number_of_adc_ports   = 2;
		break;

	case HWNAZE32PRO_OUTPORT_PWM8PWM_IN:
		number_of_pwm_outputs = 8;
		use_pwm_in            = true;
		number_of_adc_ports   = 0;
		break;

	case HWNAZE32PRO_OUTPORT_PWM8PWM_INADC1:
		number_of_pwm_outputs = 8;
		use_pwm_in            = true;
		number_of_adc_ports   = 1;
		break;

	case HWNAZE32PRO_OUTPORT_PWM8PWM_INADC2:
		number_of_pwm_outputs = 8;
		use_pwm_in            = true;
		number_of_adc_ports   = 2;
		break;

	default:

		PIOS_Assert(0);
		break;
	}
    #endif

    ///////////////////////////////////////////////////////////////////////////

    #ifndef PIOS_DEBUG_ENABLE_DEBUG_PINS
    #ifdef PIOS_INCLUDE_SERVO
	pios_servo_cfg.num_channels = number_of_pwm_outputs;
	PIOS_Servo_Init(&pios_servo_cfg);
    #endif
    #else
	PIOS_DEBUG_Init(&pios_tim_servo_all_channels, NELEMENTS(pios_tim_servo_all_channels));
    #endif

    #if defined(PIOS_INCLUDE_PWM)
	if (use_pwm_in == true) {
		pios_pwm_cfg.channels = &pios_tim_rangefinder_pwm[0];
		uintptr_t pios_pwm_id;
		PIOS_PWM_Init(&pios_pwm_id, &pios_pwm_cfg);

		uintptr_t pios_pwm_rcvr_id;
		if (PIOS_RCVR_Init(&pios_pwm_rcvr_id, &pios_pwm_rcvr_driver, pios_pwm_id)) {
			PIOS_Assert(0);
		}
		pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_PWM] = pios_pwm_rcvr_id;
	}
    #endif	/* PIOS_INCLUDE_PWM */

    #if defined(PIOS_INCLUDE_ADC)
	if(number_of_adc_ports > 0) {
		internal_adc_cfg.number_of_used_pins = number_of_adc_ports;

		uint32_t internal_adc_id;
		if(PIOS_INTERNAL_ADC_Init(&internal_adc_id, &internal_adc_cfg) < 0) {
			PIOS_Assert(0);
			panic(9);
		}

		if(PIOS_ADC_Init(&pios_internal_adc_id, &pios_internal_adc_driver, internal_adc_id) < 0)
		    panic(10);
	}
    #endif /* PIOS_INCLUDE_ADC */

    ///////////////////////////////////////////////////////////////////////////

	PIOS_WDG_Clear();
	PIOS_DELAY_WaitmS(200);
	PIOS_WDG_Clear();

	///////////////////////////////////////////////////////////////////////////

    #if defined(PIOS_INCLUDE_MPU6000)
	if (PIOS_MPU6000_Init(pios_spi_internal_id, 0, &pios_mpu6000_cfg) != 0)
		panic(1);
	if (PIOS_MPU6000_Test() != 0)
		panic(2);

	// To be safe map from UAVO enum to driver enum
	uint8_t hw_gyro_range;
	HwNaze32ProGyroRangeGet(&hw_gyro_range);

	switch(hw_gyro_range)
	{
		case HWNAZE32PRO_GYRORANGE_250:
			PIOS_MPU6000_SetGyroRange(PIOS_MPU60X0_SCALE_250_DEG);
			break;

		case HWNAZE32PRO_GYRORANGE_500:
			PIOS_MPU6000_SetGyroRange(PIOS_MPU60X0_SCALE_500_DEG);
			break;

		case HWNAZE32PRO_GYRORANGE_1000:
			PIOS_MPU6000_SetGyroRange(PIOS_MPU60X0_SCALE_1000_DEG);
			break;

		case HWNAZE32PRO_GYRORANGE_2000:
			PIOS_MPU6000_SetGyroRange(PIOS_MPU60X0_SCALE_2000_DEG);
			break;
	}

	uint8_t hw_accel_range;
	HwNaze32ProAccelRangeGet(&hw_accel_range);

	switch(hw_accel_range)
	{
		case HWNAZE32PRO_ACCELRANGE_2G:
			PIOS_MPU6000_SetAccelRange(PIOS_MPU60X0_ACCEL_2G);
			break;

		case HWNAZE32PRO_ACCELRANGE_4G:
			PIOS_MPU6000_SetAccelRange(PIOS_MPU60X0_ACCEL_4G);
			break;

		case HWNAZE32PRO_ACCELRANGE_8G:
			PIOS_MPU6000_SetAccelRange(PIOS_MPU60X0_ACCEL_8G);
			break;

		case HWNAZE32PRO_ACCELRANGE_16G:
			PIOS_MPU6000_SetAccelRange(PIOS_MPU60X0_ACCEL_16G);
			break;
	}

	// the filter has to be set before rate else divisor calculation will fail
	uint8_t hw_mpu6000_dlpf;
	HwNaze32ProMPU6000DLPFGet(&hw_mpu6000_dlpf);

	enum pios_mpu60x0_filter mpu6000_dlpf = \
	    (hw_mpu6000_dlpf == HWNAZE32PRO_MPU6000DLPF_256) ? PIOS_MPU60X0_LOWPASS_256_HZ : \
	    (hw_mpu6000_dlpf == HWNAZE32PRO_MPU6000DLPF_188) ? PIOS_MPU60X0_LOWPASS_188_HZ : \
	    (hw_mpu6000_dlpf == HWNAZE32PRO_MPU6000DLPF_98)  ? PIOS_MPU60X0_LOWPASS_98_HZ  : \
	    (hw_mpu6000_dlpf == HWNAZE32PRO_MPU6000DLPF_42)  ? PIOS_MPU60X0_LOWPASS_42_HZ  : \
	    (hw_mpu6000_dlpf == HWNAZE32PRO_MPU6000DLPF_20)  ? PIOS_MPU60X0_LOWPASS_20_HZ  : \
	    (hw_mpu6000_dlpf == HWNAZE32PRO_MPU6000DLPF_10)  ? PIOS_MPU60X0_LOWPASS_10_HZ  : \
	    (hw_mpu6000_dlpf == HWNAZE32PRO_MPU6000DLPF_5)   ? PIOS_MPU60X0_LOWPASS_5_HZ   : \
	    pios_mpu6000_cfg.default_filter;
	PIOS_MPU6000_SetLPF(mpu6000_dlpf);

	uint8_t hw_mpu6000_samplerate;
	HwNaze32ProMPU6000RateGet(&hw_mpu6000_samplerate);

	uint16_t mpu6000_samplerate = \
	    (hw_mpu6000_samplerate == HWNAZE32PRO_MPU6000RATE_200)  ? 200  : \
	    (hw_mpu6000_samplerate == HWNAZE32PRO_MPU6000RATE_333)  ? 333  : \
	    (hw_mpu6000_samplerate == HWNAZE32PRO_MPU6000RATE_333)  ? 444  : \
	    (hw_mpu6000_samplerate == HWNAZE32PRO_MPU6000RATE_500)  ? 500  : \
	    (hw_mpu6000_samplerate == HWNAZE32PRO_MPU6000RATE_666)  ? 666  : \
	    (hw_mpu6000_samplerate == HWNAZE32PRO_MPU6000RATE_1000) ? 1000 : \
	    (hw_mpu6000_samplerate == HWNAZE32PRO_MPU6000RATE_2000) ? 2000 : \
	    (hw_mpu6000_samplerate == HWNAZE32PRO_MPU6000RATE_4000) ? 4000 : \
	    (hw_mpu6000_samplerate == HWNAZE32PRO_MPU6000RATE_8000) ? 8000 : \
	    pios_mpu6000_cfg.default_samplerate;
	PIOS_MPU6000_SetSampleRate(mpu6000_samplerate);
    #endif /* PIOS_INCLUDE_MPU6000 */

    ///////////////////////////////////////////////////////////////////////////

    PIOS_WDG_Clear();

    uint8_t Magnetometer;
	HwNaze32ProMagnetometerGet(&Magnetometer);

	if (Magnetometer == HWNAZE32PRO_MAGNETOMETER_INTERNAL)
	{
		#if defined(PIOS_INCLUDE_HMC5983)
        if (PIOS_HMC5983_Init(pios_spi_internal_id, 1, &pios_hmc5983_cfg) != 0)
		    panic(3);

	    PIOS_WDG_Clear();

	    if (PIOS_HMC5983_Test() != 0)
		    panic(4);
        #endif /* PIOS_INCLUDE_HMC5983 */
	}

	if (Magnetometer == HWNAZE32PRO_MAGNETOMETER_EXTERNAL)
	{
		#if defined(PIOS_INCLUDE_HMC5883)
		if (PIOS_HMC5883_Init(pios_i2c_external_id, &pios_hmc5883_external_cfg) != 0)
			panic(7);

		PIOS_WDG_Clear();

		if (PIOS_HMC5883_Test() != 0)
			panic(8);

		// setup sensor orientation
		uint8_t ExtMagOrientation;
		HwNaze32ProExtMagOrientationGet(&ExtMagOrientation);

		enum pios_hmc5883_orientation hmc5883_orientation = \
			(ExtMagOrientation == HWNAZE32PRO_EXTMAGORIENTATION_TOP0DEGCW)      ? PIOS_HMC5883_TOP_0DEG      : \
			(ExtMagOrientation == HWNAZE32PRO_EXTMAGORIENTATION_TOP90DEGCW)     ? PIOS_HMC5883_TOP_90DEG     : \
			(ExtMagOrientation == HWNAZE32PRO_EXTMAGORIENTATION_TOP180DEGCW)    ? PIOS_HMC5883_TOP_180DEG    : \
			(ExtMagOrientation == HWNAZE32PRO_EXTMAGORIENTATION_TOP270DEGCW)    ? PIOS_HMC5883_TOP_270DEG    : \
			(ExtMagOrientation == HWNAZE32PRO_EXTMAGORIENTATION_BOTTOM0DEGCW)   ? PIOS_HMC5883_BOTTOM_0DEG   : \
			(ExtMagOrientation == HWNAZE32PRO_EXTMAGORIENTATION_BOTTOM90DEGCW)  ? PIOS_HMC5883_BOTTOM_90DEG  : \
			(ExtMagOrientation == HWNAZE32PRO_EXTMAGORIENTATION_BOTTOM180DEGCW) ? PIOS_HMC5883_BOTTOM_180DEG : \
			(ExtMagOrientation == HWNAZE32PRO_EXTMAGORIENTATION_BOTTOM270DEGCW) ? PIOS_HMC5883_BOTTOM_270DEG : \
			pios_hmc5883_external_cfg.Default_Orientation;
		PIOS_HMC5883_SetOrientation(hmc5883_orientation);
		#endif  /* PIOS_INCLUDE_HMC5883 */
	}

    ///////////////////////////////////////////////////////////////////////////

    PIOS_WDG_Clear();

    #if defined(PIOS_INCLUDE_MS5611_SPI)
	if (PIOS_MS5611_SPI_Init(pios_spi_internal_id, 2, &pios_ms5611_cfg) != 0) {
		PIOS_Assert(0);
	}
    #endif	/* PIOS_INCLUDE_MS5611_SPI */

    ///////////////////////////////////////////////////////////////////////////

    PIOS_WDG_Clear();

    #if defined(PIOS_INCLUDE_GPIO)
	PIOS_GPIO_Init();
    #endif

	///////////////////////////////////////////////////////////////////////////

	/* Make sure we have at least one telemetry link configured or else fail initialization */
	PIOS_Assert(pios_com_telem_rf_id || pios_com_telem_usb_id);

	///////////////////////////////////////////////////////////////////////////
}

/**
 * @}
 */
