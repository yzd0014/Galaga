#ifndef HSPI_H
#define HSPI_H

#include "main.h"

#define PB_MOSI (5)
#define PB_SCK  (3)
#define PB_DC   (4)
#define PA_CS   (10)
#define PA_RST  (15)

void hspi_w8(SPI_TypeDef *SPIx, uint8_t dat);
void hspi_w16(SPI_TypeDef *SPIx, uint16_t dat);
void hspi_cmd(SPI_TypeDef *SPIx, uint8_t cmd);
void ili9341_hspi_init(SPI_TypeDef *SPIx);

#endif 

