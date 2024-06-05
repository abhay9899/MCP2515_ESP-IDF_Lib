/* Created by Abhay Dutta, 3rd June, 2024 */

/* ESP32 example code for reading CAN message with MCP2515 CAN interface via SPI */
/* In this example, 3 CAN Devices are added to the SPI bus and from each bus, CAN ID and Data are decoded*/
/* For using this example and library, please assign the required pins in "spi_comp.h" according to your project need*/

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "MCP2515.h"
#include "spi_comp.h"
#include "mcp_can_dfs.h"


////==============================INITIALIZER AND FUNCTION FOR TWAI=============================////
void twai_init()
{
  //Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  //Install TWAI driver
  if(twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) 
  {
    printf("Driver installed\n");
  } 
  else 
  {
    printf("Failed to install driver\n");
    return;
  }
  //Start TWAI driver
  if(twai_start() == ESP_OK) 
  {
    printf("Driver started\n");
  } 
  else 
  {
    printf("Failed to start driver\n");
    return;
  }
}

////==============================FUNCTION FOR TRANSMITTING TWAI (CAN) MESSAGE=============================////
void twai_tx(uint32_t add, uint8_t data_len)
{
  printf("inside twai tx task\n");
  twai_message_t tx_message;
  //configurations for CAN transmission

  tx_message.identifier = add;
  tx_message.extd = 1;
  tx_message.rtr = 0;
  tx_message.data_length_code = data_len;

  for (int i = 0; i < data_len; i++) 
  {
    tx_message.data[i] = twai_tx_data[i];
  }

  if (twai_transmit(&tx_message, pdMS_TO_TICKS(1000)) == ESP_OK) 
  {
    printf("Message queued for transmission\n");
    twai_clear_transmit_queue();
  } 
  else 
  {
    printf("Failed to queue message for transmission\n");
  }
}

////============================TASK FOR MCP2515 READ and WRITE============================////
void MCP_RW_task(void *arg)
{
  while(1)
  {
    ////=====================================For Device 1 CAN Read=================================////
    remove_device();
    add_device_one();

    readMsgBuf(&rxId, &len, rxBuf);
    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
    {
      sprintf(msgString, "Device 1 Ex-ID: 0x%.8X | Data Length: %1d | Data: ", (rxId & 0x1FFFFFFF), len);
    }
    else
    {
      sprintf(msgString, "Device 1 Sd-ID: 0x%.3X | Data Length: %1d | Data: ", rxId, len);
    }
    printf(msgString);
    vTaskDelay(1);

    if((rxId & 0x40000000) == 0x40000000)
    {   
      // Determine if message is a remote request frame.
      sprintf(msgString, "REMOTE REQUEST FRAME\n");
      printf(msgString);
    } 
    else 
    {
        for(uint8_t i = 0; i < len; i++)
        {
          sprintf(msgString, "0x%.2X,  ", rxBuf[i]);
          printf(msgString);
        }
    }
    vTaskDelay(1);

    //clearing out all the previous CAN data!
    for(uint8_t i = 0; i < len; i++)
    {
        rxBuf[i] = 0;
    }
    rxId = 0x00000000;
    len = 0;

    printf("\n\n");
    vTaskDelay(5 / portTICK_PERIOD_MS);

    ////=====================================For Device 2 CAN Read================================////
    remove_device();
    add_device_two();

    readMsgBuf(&_rxId, &_len, _rxBuf);
    if((_rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
    {
      sprintf(msgString, "Device 2 Ex-ID: 0x%.8X | Data Length: %1d | Data: ", (_rxId & 0x1FFFFFFF), _len);
    }
    else
    {
      sprintf(msgString, "Device 2 Sd-ID: 0x%.3X | Data Length: %1d | Data: ", _rxId, _len);
    }
    printf(msgString);
    vTaskDelay(1);

    if((_rxId & 0x40000000) == 0x40000000)
    {    
      // Determine if message is a remote request frame.
      sprintf(msgString, "REMOTE REQUEST FRAME\n");
      printf(msgString);
    } 
    else 
    {
        for(uint8_t i = 0; i < _len; i++)
        {
          sprintf(msgString, "0x%.2X,  ", _rxBuf[i]);
          printf(msgString);
        }
    }
    vTaskDelay(1);

    //clearing out all the previous CAN data!
    for(uint8_t i = 0; i < _len; i++)
    {
        _rxBuf[i] = 0;
    }
    _rxId = 0x00000000;
    _len = 0;

    printf("\n\n");
    vTaskDelay(5 / portTICK_PERIOD_MS);

    ////===============================For Device 3 MODULE CAN Read================================////
    remove_device();
    add_device_three();

    readMsgBuf(&__rxId, &__len, __rxBuf);
    if((__rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
    {
      sprintf(msgString, "Device 3 Ex-ID: 0x%.8X | Data Length: %1d | Data: ", (__rxId & 0x1FFFFFFF), __len);
    }
    else
    {
        sprintf(msgString, "Device 3 Sd-ID: 0x%.3X | Data Length: %1d | Data: ", __rxId, __len);
    }
    printf(msgString);
    vTaskDelay(1);

    if((__rxId & 0x40000000) == 0x40000000)
    {    
      // Determine if message is a remote request frame.
      sprintf(msgString, "REMOTE REQUEST FRAME\n");
      printf(msgString);
    } 
    else 
    {
        for(uint8_t i = 0; i < __len; i++)
        {
          sprintf(msgString, "0x%.2X,  ", __rxBuf[i]);
          printf(msgString);
        }

    }
    vTaskDelay(1);

    //clearing out all the previous CAN data!
    for(uint8_t i = 0; i < __len; i++)
    {
        __rxBuf[i] = 0;
    }
    __rxId = 0x00000000;
    __len = 0;

    printf("\n\n");
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}


////==================================TASK FOR TWAI (CAN) READ===================================////
void TWAI_read_task(void *arg)
{
  while (1)
  {
    // Wait for message to be received
    twai_message_t message;
    printf("CAN_STATUS :: Reception Started\n");
    if (twai_receive(&message, portMAX_DELAY) == ESP_OK)
    {
        printf("CAN_STATUS :: Message received\n");
        printf("CAN_STATUS :: ID is %X\n", message.identifier);
    }

    vTaskDelay(1);
  }
}


////=====================================APP MAIN======================================////
void app_main(void)
{
  len = 0;
  _len = 0;
  __len = 0;
	spi = spi_init_HSPI();

  //========For Device 1 CAN=========//
  add_device_one();
	if(MCP2515_begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
	{
  	printf("MCP2515 Device 1 Initialized Successfully!\n");
  }
	else
	{
  	printf("Error Initializing MCP2515\n");
  }

	setMode(MCP_NORMAL);
 	printf("MCP2515 Device 1 Receive START\n");

  //========For Device 2 CAN=========//
  remove_device();
  add_device_two();
 	if(MCP2515_begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
	{
    	printf("MCP2515 Device 2 Initialized Successfully!\n");
	}
	else
	{
    	printf("Error Initializing MCP2515\n");
	}
	setMode(MCP_NORMAL);
 	printf("MCP2515 Device 2 Receive START\n");

  //========For DEvice 3 CAN=========//
  remove_device();
  add_device_three();
  if(MCP2515_begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK)
  {
      printf("MCP2515 Device 3 Initialized Successfully!\n");
  }
  else
  {
      printf("Error Initializing MCP2515\n");
  }
  setMode(MCP_NORMAL);
  printf("MCP2515 Device 3 Receive START\n");

  twai_init();
  xTaskCreate(MCP_RW_task, "MCP_RW_task", 4096, NULL, 3, NULL);
  // xTaskCreate(TWAI_read_task, "TWAI_read_task", 4096, NULL, 3, NULL);
}
