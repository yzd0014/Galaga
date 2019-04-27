#ifndef GL_H
#define GL_H

#include "main.h"
#include "hspi.h"
#include "Font.h"

#define ILI9341_WHITE       0xFFFF  ///< 255, 255, 255
#define ILI9341_BLACK       0x0000  ///<   0,   0,   0
#define ILI9341_RED         0xF800  ///< 255,   0,   0
#define ILI9341_BLUE  			0x001F  

#define ILI9341_TFTWIDTH   240      ///< ILI9341 max TFT width
#define ILI9341_TFTHEIGHT  320      ///< ILI9341 max TFT height

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif
#ifndef pgm_read_byte
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
 
int16_t _abs(int16_t a){
	if(a < 0) a = a* -1;
	return a;
}
void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h){
	uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);
	// Set column range.
	hspi_cmd(SPI1, 0x2A);
	hspi_w16(SPI1, x1);
	hspi_w16(SPI1, x2);
	
	// Set row range.
	hspi_cmd(SPI1, 0x2B);
	hspi_w16(SPI1, y1);
	hspi_w16(SPI1, y2);
	
	// Set 'write to RAM'
	hspi_cmd(SPI1, 0x2C);
}

void writeColor(uint16_t color, uint32_t len){
	for(int i = 0; i < len; i++){
		hspi_w16(SPI1, color);
	}
}

void writeFillRectPreclipped(int16_t x, int16_t y,
  int16_t w, int16_t h, uint16_t color) {
    setAddrWindow(x, y, w, h);
    writeColor(color, (uint32_t)w * h);
}

void drawPixel(int16_t x, int16_t y, uint16_t color){
	setAddrWindow(x, y, 1, 1);
	writeColor(color, 1);
}



void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
       if((y >= 0) && (y < ILI9341_TFTHEIGHT) && w) { // Y on screen, nonzero width
        if(w < 0) {                      // If negative width...
            x +=  w + 1;                 //   Move X to left edge
            w  = -w;                     //   Use positive width
        }
        if(x < ILI9341_TFTWIDTH) {                 // Not off right
            int16_t x2 = x + w - 1;
            if(x2 >= 0) {                // Not off left
                // Line partly or fully overlaps screen
                if(x  <  0)       { x = 0; w = x2 + 1; } // Clip left
                if(x2 >= ILI9341_TFTWIDTH)  { w = ILI9341_TFTWIDTH  - x;   } // Clip right
                writeFillRectPreclipped(x, y, w, 1, color);
            }
        }
    }
}
	
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if(w && h) {                            // Nonzero width and height?
        if(w < 0) {                         // If negative width...
            x +=  w + 1;                    //   Move X to left edge
            w  = -w;                        //   Use positive width
        }
        if(x < ILI9341_TFTWIDTH) {                    // Not off right
            if(h < 0) {                     // If negative height...
                y +=  h + 1;                //   Move Y to top edge
                h  = -h;                    //   Use positive height
            }
            if(y < ILI9341_TFTHEIGHT) {               // Not off bottom
                int16_t x2 = x + w - 1;
                if(x2 >= 0) {               // Not off left
                    int16_t y2 = y + h - 1;
                    if(y2 >= 0) {           // Not off top
                        // Rectangle partly or fully overlaps screen
                        if(x  <  0)       { x = 0; w = x2 + 1; } // Clip left
                        if(y  <  0)       { y = 0; h = y2 + 1; } // Clip top
                        if(x2 >= ILI9341_TFTWIDTH)  { w = ILI9341_TFTWIDTH  - x;   } // Clip right
                        if(y2 >= ILI9341_TFTHEIGHT) { h = ILI9341_TFTHEIGHT - y;   } // Clip bottom
                        //startWrite();
                        writeFillRectPreclipped(x, y, w, h, color);
                        //endWrite();
                    }
                }
            }
        }
    }
}

void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    if((x >= 0) && (x < ILI9341_TFTWIDTH) && h) { // X on screen, nonzero height
        if(h < 0) {                     // If negative height...
            y +=  h + 1;                //   Move Y to top edge
            h  = -h;                    //   Use positive height
        }
        if(y < ILI9341_TFTHEIGHT) {               // Not off bottom
            int16_t y2 = y + h - 1;
            if(y2 >= 0) {               // Not off top
                // Line partly or fully overlaps screen
                if(y  <  0)       { y = 0; h = y2 + 1; } // Clip top
                if(y2 >= ILI9341_TFTHEIGHT) { h = ILI9341_TFTHEIGHT - y;   } // Clip bottom
                writeFillRectPreclipped(x, y, 1, h, color);
            }
        }
    }
}

void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size) {
	
        if((x >= ILI9341_TFTWIDTH)            || // Clip right
           (y >= ILI9341_TFTHEIGHT)           || // Clip bottom
           ((x + 6 * size - 1) < 0) || // Clip left
           ((y + 8 * size - 1) < 0))   // Clip top
            return;

        if(c >= 176) c++; // Handle 'classic' charset behavior

        //startWrite();
        for(int8_t i=0; i<5; i++ ) { // Char bitmap = 5 columns
            uint8_t line = pgm_read_byte(&font[c * 5 + i]);
            for(int8_t j=0; j<8; j++, line >>= 1) {
                if(line & 1) {
                    if(size == 1)
                        drawPixel(x+i, y+j, color);
                    else
                        fillRect(x+i*size, y+j*size, size, size, color);
                } else if(bg != color) {
                    if(size == 1)
                        drawPixel(x+i, y+j, bg);
                    else
                        fillRect(x+i*size, y+j*size, size, size, bg);
                }
            }
        }
        if(bg != color) { // If opaque, draw vertical line for last column
            if(size == 1) writeFastVLine(x+5, y, 8, bg);
            else          fillRect(x+5*size, y, size, 8*size, bg);
        }
        //endWrite();
   
}

#endif 

