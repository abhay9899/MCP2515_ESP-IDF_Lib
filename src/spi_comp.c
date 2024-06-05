/* Created by Abhay Dutta, 3rd June, 2024 */

#include "driver/spi_master.h"
#include "spi_comp.h"
#include "driver/gpio.h"

 spi_device_interface_config_t devCfg;
 spi_device_interface_config_t _devCfg;
 spi_device_interface_config_t __devCfg;

inline void add_device_one()
{
  ret = spi_bus_add_device(HSPI_HOST, &devCfg, &spi);
  ESP_ERROR_CHECK(ret);
  
}
inline void add_device_two()
{
  ret = spi_bus_add_device(HSPI_HOST, &_devCfg, &spi);
  ESP_ERROR_CHECK(ret);
}
inline void add_device_three()
{
  ret = spi_bus_add_device(HSPI_HOST, &__devCfg, &spi);
  ESP_ERROR_CHECK(ret);
}

inline void remove_device()
{
  ret = spi_bus_remove_device(spi);
  ESP_ERROR_CHECK(ret);
}


////=============initializaion for HSPI (CCS) CAN==============////
spi_device_handle_t spi_init_HSPI(void) 
{
  spi_bus_config_t buscfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = (4 * 8)
  };

  //========for CCS Device SPI=========//
  devCfg.mode = 0;
  devCfg.clock_speed_hz = 10000000;
  devCfg.spics_io_num = PIN_NUM_CS;
  devCfg.queue_size = 5;

  //========for GBT Device SPI=========//
  _devCfg.mode = 0;
  _devCfg.clock_speed_hz = 10000000;
  _devCfg.spics_io_num = PIN_NUM_CS_1;
  _devCfg.queue_size = 5;

  //========for MODULE Device SPI=========//
  __devCfg.mode = 0;
  __devCfg.clock_speed_hz = 10000000;
  __devCfg.spics_io_num = PIN_NUM_CS_2;
  __devCfg.queue_size = 5;
  

  //Initialize the HSPI bus
  ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);
  ESP_ERROR_CHECK(ret);
  if(ret == ESP_OK)
  {
    printf("\nSuccessfully Initialized HSPI!\n");
  }
  else
  {
    printf("\nInitialization Failed!\n");
  }
  return spi;
}