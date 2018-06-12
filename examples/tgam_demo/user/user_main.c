/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "esp_common.h"
#include "user_config.h"
#include "gpio.h"
#include "uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "c_types.h"
#include "esp8266/ets_sys.h"

static xTaskHandle key_handler_task_handle;
void tgam_enter_sleep(void);
void user_gpio_init(void);
void TGAM_powerenable(void);
void TGAM_powerdisable(void);
void buzzer_enable(void);
void buzzer_disable(void);
void button_enable(void);

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;
        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void wifi_event_handler_cb(System_Event_t *event)
{
    if (event == NULL) {
        return;
    }

    switch (event->event_id) {
        case EVENT_STAMODE_GOT_IP:
            os_printf("sta got ip ,create task and free heap size is %d\n", system_get_free_heap_size());
            user_conn_init();
            break;

        case EVENT_STAMODE_CONNECTED:
            os_printf("sta connected\n");
            break;

        case EVENT_STAMODE_DISCONNECTED:
            wifi_station_connect();
            break;

        default:
            break;
    }
}

void tgam_led_on(void)
{
    GPIO_OUTPUT_SET(14, 0);
}

void tgam_led_off(void)
{
    GPIO_OUTPUT_SET(14, 1);
}

void led_task(void  *pvParameters)
{
    u8 cnt=0;
    while(1)
    {
        //os_printf("led task\r\n");
        cnt++;
        switch ( cnt%2 )
        {
            case 0://R
                tgam_led_on();
                break;
            case 1://G
                tgam_led_off();
                break;
        }
        vTaskDelay(1000 / portTICK_RATE_MS);  //send every 1 seconds
    }
    os_printf("led task delete !!!!!!!!!!!!\r\n");
    vTaskDelete(NULL);
}

void key_task(void  *pvParameters)
{
    while(1)
    {
        //os_printf("key task\r\n");
        vTaskSuspend( NULL );
        uart_init_new();
        buzzer_enable();
        vTaskDelay(200 / portTICK_RATE_MS);
        buzzer_disable();
        os_printf("wakeup callback !!!!!!!!!\r\n");
        user_gpio_init();
        button_enable();
        TGAM_powerenable();
        wifi_set_opmode(STATION_MODE);

        struct station_config config;
        bzero(&config, sizeof(struct station_config));
        sprintf(config.ssid, SSID);
        sprintf(config.password, PASSWORD);
        wifi_station_set_config(&config);

        wifi_set_event_handler_cb(wifi_event_handler_cb);

        wifi_station_connect();
        vTaskSuspend( NULL );
        tgam_enter_sleep();
    }
    os_printf("key task delete !!!!!!!!!!!!\r\n");
    vTaskDelete(NULL);
}

void user_gpio_init(void)
{
    //引脚功能设置
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4); //TGAM_powerenable
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13); //float
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14); //LED
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5); //BUZZER

    GPIO_OUTPUT_SET(4, 0);
    GPIO_OUTPUT_SET(13, 0);
    GPIO_OUTPUT_SET(14, 0);
    GPIO_OUTPUT_SET(5, 0);
}

void TGAM_powerenable(void)
{
    GPIO_OUTPUT_SET(4, 1);
}

void TGAM_powerdisable(void)
{
    GPIO_OUTPUT_SET(4, 0);
}

void buzzer_enable(void)
{
    GPIO_OUTPUT_SET(5, 1);
}

void buzzer_disable(void)
{
    GPIO_OUTPUT_SET(5, 0);
}

static void gpio_intr_handler()
{
    uint32 gpio_status;

    _xt_isr_mask(1<<ETS_GPIO_INUM);    //disable interrupt
    gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
    if(gpio_status & BIT(12))//判断中断针脚
    {
        //printf("go sleep !!!!\n");
        //tgam_enter_sleep();
        xTaskResumeFromISR( key_handler_task_handle );
    }
    GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, gpio_status ); //clear interrupt mask
    _xt_isr_unmask(1 << ETS_GPIO_INUM); //Enable the GPIO interrupt
}

void button_enable(void)
{
    GPIO_ConfigTypeDef pGPIOConfig;

    gpio_pin_wakeup_disable();
    pGPIOConfig.GPIO_Pin = GPIO_Pin_12;
    pGPIOConfig.GPIO_Mode = GPIO_Mode_Input;
    pGPIOConfig.GPIO_Pullup = GPIO_PullUp_DIS;
    pGPIOConfig.GPIO_IntrType = GPIO_PIN_INTR_NEGEDGE;
    gpio_config(&pGPIOConfig);

    gpio_intr_handler_register(gpio_intr_handler,NULL);
    GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, 0xFFFFFFFF ); //clear interrupt mask
    _xt_isr_unmask(1 << ETS_GPIO_INUM);    //Enable the GPIO interrupt
}

void fpm_wakup_cb_func(void)
{
    return;
}

void tgam_enter_sleep(void)
{
    os_printf("go sleep !!!!!!!!!\r\n");
    tgam_led_off();
    TGAM_powerdisable();
    buzzer_disable();
    wifi_station_disconnect();
    wifi_set_opmode(NULL_MODE);
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    wifi_fpm_open();
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12); //key
    gpio_pin_wakeup_enable(12,GPIO_PIN_INTR_LOLEVEL);
    wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func);     // Set wakeup callback
    buzzer_enable();
    vTaskDelay(200 / portTICK_RATE_MS);  //send every 1 seconds
    buzzer_disable();
    vTaskDelay(200 / portTICK_RATE_MS);  //send every 1 seconds
    buzzer_enable();
    vTaskDelay(200 / portTICK_RATE_MS);  //send every 1 seconds
    buzzer_disable();
    wifi_fpm_do_sleep(0xFFFFFFF);
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    uart_init_new();
    button_enable();
    xTaskCreate(led_task,  "led_task", 256 * 2,    NULL,   2,  NULL);
    xTaskCreate(key_task,  "key_task", 256 * 2,    NULL,   2,  &key_handler_task_handle);
    os_printf("SDK version:%s %d\n", system_get_sdk_version(), system_get_free_heap_size());
    tgam_enter_sleep();
}
