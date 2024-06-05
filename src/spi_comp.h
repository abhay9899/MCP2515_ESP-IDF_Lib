/* Created by Abhay Dutta, 3rd June, 2024 */

#ifndef _SPI_COMP_H
#define _SPI_COMP_H

#define DMA_CHAN     2

//*******************Please assign the Pins according to your needs *********************//
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14

#define PIN_NUM_CS   25    //for device1
#define PIN_NUM_CS_1 26    //for device2
#define PIN_NUM_CS_2  27   //for device3


uint32_t gbt_can_id;
uint32_t ccs_can_id;
uint32_t pms_can_id;
uint32_t temp_id;


char msgString[128];
uint8_t twai_tx_data[8];

//for device1
uint32_t rxId;
unsigned char len;
unsigned char rxBuf[8];

//for device2
uint32_t _rxId;
unsigned char _len;
unsigned char _rxBuf[8];

//for device3
uint32_t __rxId;
unsigned char __len;
unsigned char __rxBuf[8];

uint8_t spiToggle;

spi_device_handle_t spi;
spi_device_handle_t _spi;
esp_err_t ret;

spi_device_handle_t spi_init_HSPI(void);

void add_device_one();
void add_device_two();
void add_device_three();
void remove_device();

#endif