/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial API and implementation and/or initial documentation
 *******************************************************************************/

#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mqtt/MQTTClient.h"

#include "user_config.h"
#include "json/cJSON.h"
//#include "freertos/semphr.h"
#include "ThinkGearStreamParser.h"

#define MQTT_CLIENT_THREAD_NAME         "mqtt_client_thread"
#define MQTT_CLIENT_THREAD_STACK_WORDS  2048
#define MQTT_CLIENT_THREAD_PRIO         8

//SemaphoreHandle_t MQTTpubSemaphore = NULL;
MQTTMessage message;
u8 mqtt_payload[1024 * 2];

void TGAM_powerenable(void);
void TGAM_powerdisable(void);

xTaskHandle mqttc_client_handle;

void jsonMQTTDataPublish( unsigned char extendedCodeLevel,
                  unsigned char code,
                  unsigned char valueLength,
                  const unsigned char *value,
                  void *customData )
{
    cJSON * root =  cJSON_CreateObject();
    int rc = 0;

    //cJSON_AddNumberToObject(root, "ChipID", (system_get_chip_id()));//根节点下添加
    switch ( code ) {
        case ( 0x16 ):
            cJSON_AddNumberToObject(root, "ChipID", (system_get_chip_id()));//根节点下添加
            cJSON_AddNumberToObject(root, "Blink", (value[0] & 0xFF));
            break;
        case ( PARSER_CODE_MEDITATION ):
            cJSON_AddNumberToObject(root, "ChipID", (system_get_chip_id()));//根节点下添加
            cJSON_AddNumberToObject(root, "Meditation", (value[0] & 0xFF));
            break;
        case ( PARSER_CODE_POOR_QUALITY ):
            cJSON_AddNumberToObject(root, "ChipID", (system_get_chip_id()));//根节点下添加
            cJSON_AddNumberToObject(root, "Poor_Signal", (value[0] & 0xFF));
            break;
        case ( PARSER_CODE_ATTENTION ):
            cJSON_AddNumberToObject(root, "ChipID", (system_get_chip_id()));//根节点下添加
            cJSON_AddNumberToObject(root, "Attention", (value[0] & 0xFF));
            break;
        case ( PARSER_CODE_EEG_POWERS ):
            cJSON_AddNumberToObject(root, "ChipID", (system_get_chip_id()));//根节点下添加
            cJSON_AddNumberToObject(root, "EEG_POWERS", (value[0] & 0xFF));
            break;
        case ( PARSER_CODE_RAW_SIGNAL ):
            cJSON_Delete(root);
            return;
            cJSON_AddNumberToObject(root, "Raw", ((short) (( value[0] << 8 ) | value[1])));
            break;
        default:
            cJSON_Delete(root);
            return;
            break;
    }

    printf("%s\r\n", cJSON_Print(root));
    /*
    message.qos = QOS0;
    message.retained = 0;
    //xSemaphoreTake( MQTTpubSemaphore, portMAX_DELAY );
    memset(payload,0,sizeof(payload));
    message.payload = payload;
    strcpy(payload, cJSON_Print(root));
    //xSemaphoreGive( MQTTpubSemaphore );
    message.payloadlen = strlen(payload);
    vTaskResume(mqttc_client_handle);
*/
    cJSON_Delete(root);
}
static void messageArrived(MessageData* data)
{
    printf("Message arrived: %s\n", data->message->payload);
}

static void mqtt_client_thread(void* pvParameters)
{
    printf("mqtt client thread starts\n");
    MQTTClient client;
    Network network;
    unsigned char sendbuf[80], readbuf[80] = {0};
    int rc = 0, count = 0;
    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

    pvParameters = 0;
    NetworkInit(&network);
    MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

    char* address = MQTT_BROKER;

    while ((rc = NetworkConnect(&network, address, MQTT_PORT)) != 0) {
        TGAM_powerdisable();
        printf("Return code from network connect is %d\n", rc);
        vTaskDelay(1000 / portTICK_RATE_MS);  //send every 1 seconds
    }

#if defined(MQTT_TASK)

    if ((rc = MQTTStartTask(&client)) != pdPASS) {
        TGAM_powerdisable();
        printf("Return code from start tasks is %d\n", rc);
    } else {
        printf("Use MQTTStartTask\n");
    }

#endif

    connectData.MQTTVersion = 3;
    connectData.clientID.cstring = "Estack_TGAM";

    if ((rc = MQTTConnect(&client, &connectData)) != 0) {
        TGAM_powerdisable();
        printf("Return code from MQTT connect is %d\n", rc);
    } else {
        printf("MQTT Connected\n");
    }

    if ((rc = MQTTSubscribe(&client, "Estack/TGAM/sub", QOS0, messageArrived)) != 0) {
        TGAM_powerdisable();
        printf("Return code from MQTT subscribe is %d\n", rc);
    } else {
        printf("MQTT subscribe to topic \"Estack/TGAM/sub\"\n");
        vTaskDelay(1000 / portTICK_RATE_MS);  //send every 1 seconds
        TGAM_powerenable();
    }

    while (1) {
        /*message.qos = QOS0;
        message.retained = 0;
        memset(payload,0,sizeof(payload));
        message.payload = payload;
        sprintf(payload, "hello");//cJSON_Print(root));
        message.payloadlen = strlen(payload);*/
        vTaskSuspend( mqttc_client_handle );
        //xSemaphoreTake( MQTTpubSemaphore, portMAX_DELAY );
        if ((rc = MQTTPublish(&client, "Estack/TGAM/pub", &message)) != 0) {
            TGAM_powerdisable();
            printf("Return code from MQTT publish is %d\n", rc);
        } else {
            printf("MQTT publish topic \"Estack/TGAM/pub\"\r\n");
        }
        //xSemaphoreGive( MQTTpubSemaphore );
        //vTaskDelay(2);  //send every 1 seconds
    }

    printf("mqtt_client_thread going to be deleted\n");
    vTaskDelete(NULL);
    return;
}

void user_conn_init(void)
{
    int ret;
    //MQTTpubSemaphore = xSemaphoreCreateMutex();
    ret = xTaskCreate(mqtt_client_thread,
                      MQTT_CLIENT_THREAD_NAME,
                      MQTT_CLIENT_THREAD_STACK_WORDS,
                      NULL,
                      MQTT_CLIENT_THREAD_PRIO,
                      &mqttc_client_handle);

    if (ret != pdPASS)  {
        printf("mqtt create client thread %s failed\n", MQTT_CLIENT_THREAD_NAME);
    }
}
