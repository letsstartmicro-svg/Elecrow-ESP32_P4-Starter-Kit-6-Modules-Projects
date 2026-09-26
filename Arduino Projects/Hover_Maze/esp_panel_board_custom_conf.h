/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file   esp_panel_board_custom_conf.h
 * @brief  Custom board configuration: Elecrow ESP32-P4 7" 1024x600 MIPI-DSI (EK79007) + GT911 touch
 */

#pragma once

// *INDENT-OFF*

#define ESP_PANEL_BOARD_DEFAULT_USE_CUSTOM  (1)

#if ESP_PANEL_BOARD_DEFAULT_USE_CUSTOM
///////////////////////////////////////////////////// General ///////////////////////////////////////////////////////////
#define ESP_PANEL_BOARD_NAME                "Custom:Custom"
#define ESP_PANEL_BOARD_WIDTH               (1024)  // Panel width (horizontal, in pixels)
#define ESP_PANEL_BOARD_HEIGHT              (600)   // Panel height (vertical, in pixels)

///////////////////////////////////////////////////// LCD ///////////////////////////////////////////////////////////////
#define ESP_PANEL_BOARD_USE_LCD             (1)

#if ESP_PANEL_BOARD_USE_LCD
#define ESP_PANEL_BOARD_LCD_CONTROLLER      EK79007
#define ESP_PANEL_BOARD_LCD_BUS_TYPE        (ESP_PANEL_BUS_TYPE_MIPI_DSI)

#if ESP_PANEL_BOARD_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_MIPI_DSI
    /* For host */
    #define ESP_PANEL_BOARD_LCD_MIPI_DSI_LANE_NUM           (2)
    #define ESP_PANEL_BOARD_LCD_MIPI_DSI_LANE_RATE_MBPS     (1000)
    /* For refresh panel (DPI) */
    #define ESP_PANEL_BOARD_LCD_MIPI_DPI_CLK_MHZ            (51)
    #define ESP_PANEL_BOARD_LCD_MIPI_DPI_PIXEL_BITS         (ESP_PANEL_LCD_COLOR_BITS_RGB565)
    #define ESP_PANEL_BOARD_LCD_MIPI_DPI_HPW                (70)
    #define ESP_PANEL_BOARD_LCD_MIPI_DPI_HBP                (160)
    #define ESP_PANEL_BOARD_LCD_MIPI_DPI_HFP                (160)
    #define ESP_PANEL_BOARD_LCD_MIPI_DPI_VPW                (10)
    #define ESP_PANEL_BOARD_LCD_MIPI_DPI_VBP                (23)
    #define ESP_PANEL_BOARD_LCD_MIPI_DPI_VFP                (12)
    /* For DSI power PHY */
    #define ESP_PANEL_BOARD_LCD_MIPI_PHY_LDO_ID             (3)
#else
    #error "This board configuration only supports the MIPI-DSI bus."
#endif // ESP_PANEL_BOARD_LCD_BUS_TYPE

/* Color */
#define ESP_PANEL_BOARD_LCD_COLOR_BITS          (ESP_PANEL_LCD_COLOR_BITS_RGB565)
#define ESP_PANEL_BOARD_LCD_COLOR_BGR_ORDER     (0)     // 0: RGB, 1: BGR
#define ESP_PANEL_BOARD_LCD_COLOR_INEVRT_BIT    (0)     // 0/1

/* Transformation */
#define ESP_PANEL_BOARD_LCD_SWAP_XY             (0)
#define ESP_PANEL_BOARD_LCD_MIRROR_X            (0)
#define ESP_PANEL_BOARD_LCD_MIRROR_Y            (0)
#define ESP_PANEL_BOARD_LCD_GAP_X               (0)
#define ESP_PANEL_BOARD_LCD_GAP_Y               (0)

/* Reset pin */
#define ESP_PANEL_BOARD_LCD_RST_IO              (5)
#define ESP_PANEL_BOARD_LCD_RST_LEVEL           (0)
#endif // ESP_PANEL_BOARD_USE_LCD

///////////////////////////////////////////////////// Touch /////////////////////////////////////////////////////////////
#define ESP_PANEL_BOARD_USE_TOUCH               (1)

#if ESP_PANEL_BOARD_USE_TOUCH
#define ESP_PANEL_BOARD_TOUCH_CONTROLLER        GT911
#define ESP_PANEL_BOARD_TOUCH_BUS_TYPE          (ESP_PANEL_BUS_TYPE_I2C)
#define ESP_PANEL_BOARD_TOUCH_BUS_SKIP_INIT_HOST        (0)

    /* For general */
    #define ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID           (0)
#if !ESP_PANEL_BOARD_TOUCH_BUS_SKIP_INIT_HOST
    /* For host */
    #define ESP_PANEL_BOARD_TOUCH_I2C_CLK_HZ            (400 * 1000)
    #define ESP_PANEL_BOARD_TOUCH_I2C_SCL_PULLUP        (1)
    #define ESP_PANEL_BOARD_TOUCH_I2C_SDA_PULLUP        (1)
    #define ESP_PANEL_BOARD_TOUCH_I2C_IO_SCL            (19)
    #define ESP_PANEL_BOARD_TOUCH_I2C_IO_SDA            (18)
#endif
    /* For panel */
    #define ESP_PANEL_BOARD_TOUCH_I2C_ADDRESS           (0)     // 0 = default address

/* Transformation */
#define ESP_PANEL_BOARD_TOUCH_SWAP_XY           (0)
#define ESP_PANEL_BOARD_TOUCH_MIRROR_X          (0)
#define ESP_PANEL_BOARD_TOUCH_MIRROR_Y          (0)

/* Control pins */
#define ESP_PANEL_BOARD_TOUCH_RST_IO            (40)
#define ESP_PANEL_BOARD_TOUCH_RST_LEVEL         (0)
#define ESP_PANEL_BOARD_TOUCH_INT_IO            (41)
#define ESP_PANEL_BOARD_TOUCH_INT_LEVEL         (0)
#endif // ESP_PANEL_BOARD_USE_TOUCH

///////////////////////////////////////////////////// Backlight /////////////////////////////////////////////////////////
#define ESP_PANEL_BOARD_USE_BACKLIGHT           (1)

#if ESP_PANEL_BOARD_USE_BACKLIGHT
#define ESP_PANEL_BOARD_BACKLIGHT_TYPE          (ESP_PANEL_BACKLIGHT_TYPE_PWM_LEDC)
    #define ESP_PANEL_BOARD_BACKLIGHT_IO        (20)
    #define ESP_PANEL_BOARD_BACKLIGHT_ON_LEVEL  (1)
    #define ESP_PANEL_BOARD_BACKLIGHT_PWM_FREQ_HZ          (5000)
    #define ESP_PANEL_BOARD_BACKLIGHT_PWM_DUTY_RESOLUTION  (10)
#define ESP_PANEL_BOARD_BACKLIGHT_IDLE_OFF      (0)
#endif // ESP_PANEL_BOARD_USE_BACKLIGHT

///////////////////////////////////////////////////// IO expander ///////////////////////////////////////////////////////
#define ESP_PANEL_BOARD_USE_EXPANDER            (0)

///////////////////////////////////////////////////// File Version //////////////////////////////////////////////////////
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_MAJOR 1
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_MINOR 2
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_PATCH 0

#endif // ESP_PANEL_BOARD_DEFAULT_USE_CUSTOM

// *INDENT-ON*
