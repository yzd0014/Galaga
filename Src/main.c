/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "GL.h"
#include "hspi.h"
#include <stdlib.h>

#define UFO_W 35
#define UFO_H 18
#define PLAYERBULLET_W 4
#define PLAYERBULLET_H 16
#define SPACESHIP_W 22
#define SPACESHIP_H 40
//gameplay
int8_t fireButtonPressed;
int8_t fireInterval = 1;
int8_t gameOver = 0;
int8_t score = 0;
int8_t lastScore = 0;
int8_t levelCounter = 0;

void SystemClock_Config(void);
void ConfigSPI(void);
void ConfigButton(void);
void ConfigLED(void);
void ConfigTimer(void);
void EXTI0_1_IRQHandler(void);
void TIM2_IRQHandler(void);

void GameObjectsIni(void);
void Render(void);

int g_flag = 0;
int g_counter = 0;

struct Player{
	int16_t position[2];
	int8_t velocity;
	
} player;
int8_t spaceShip[1200];

struct Bullet{
	int16_t position[2];
	int16_t cachePos[2];
	int8_t velocity[2];
	int8_t life;
} playerBullets[4], enemyBullets[4];
int8_t pBCounter = 0;
int8_t eBCounter = 0;

struct Enemy{
	int16_t position[2];
	int8_t velocity;
	int8_t life;
	int8_t fired;
} enemies[4];
int8_t enemyCounter = 0;
int16_t spawnInterval = 1;//start with 0, actual interval is 30
int16_t SPAWANINTERVAL = 240;
int8_t UFO[770];

struct Laser{
	int16_t position[2];
	int8_t velocity;
	int8_t life;
} laser[2];
int8_t laserPointer = 0;
int8_t laserPowerBar = 0;
int8_t powerBarFullValue = 8;
int8_t laserCounter = 0;
//int16_t spawn = 400;

int main(void)
{
	HAL_Init();
  SystemClock_Config();
	
	ConfigSPI();
	ConfigButton();
	ConfigTimer();
	ConfigLED();	
	ili9341_hspi_init(SPI1);
	
	//set background color
	setAddrWindow(0, 0, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);
	writeColor(ILI9341_BLACK, ILI9341_TFTWIDTH * ILI9341_TFTHEIGHT);
	
	GameObjectsIni();
	while(1){	
		//reset velocity
		player.velocity = 0;
		GPIOC->ODR &= ~(1<<6);//red off
		GPIOC->ODR &= ~(1<<7);//blue off
		
		//read input ********************************************************************
		if(!gameOver){
			if(fireInterval > 0)fireInterval--;
			if(fireButtonPressed && fireInterval == 0){
				fireInterval = 40;
				fireButtonPressed = 0;
				//GPIOC->ODR |= 1<<8;
				if(pBCounter < 4){
					playerBullets[pBCounter].position[0] = player.position[0];
					playerBullets[pBCounter].position[1] = player.position[1] - 50;
					playerBullets[pBCounter].life = 1;
					pBCounter++;
				}
			}
			if (~(GPIOA->IDR) & (1<<9)) { // right button pressed
				GPIOC->ODR |= 1<<7;
				player.velocity = 2;
			}
		
			if (~(GPIOA->IDR) & (1<<8)) { // left button pressed
				GPIOC->ODR |= 1<<6;
				player.velocity = -2;
			}
		}
		//eventTick*******************************************************************************
		//save score first 
		lastScore = score;
		if(!gameOver){				
			//player
			int16_t positionCache = player.position[0];
			player.position[0] += player.velocity;
			//offset check
			if(player.position[0] < 15 || player.position[0] > ILI9341_TFTWIDTH - 15){
				player.position[0] = positionCache;
			}
			
			//player bullets
			for(int i = 0; i < 4; i++){
				if(playerBullets[i].life > 0){
					playerBullets[i].cachePos[0] = playerBullets[i].position[0];
					playerBullets[i].cachePos[1] = playerBullets[i].position[1];
					playerBullets[i].position[1] += -1;
					
					
					//offset check
					if(playerBullets[i].position[1] < -8){
						pBCounter--;
						playerBullets[i] = playerBullets[pBCounter];//move the last one to current location 
						playerBullets[pBCounter].life = 0;
						
					}
					
				}
			}
			
			//enemy
			if(spawnInterval > 0)spawnInterval--;
			if(spawnInterval == 0 && enemyCounter < 4){
				//GPIOC->ODR |= 1<<8;
				spawnInterval = SPAWANINTERVAL;
				enemies[enemyCounter].position[0] = UFO_W/2 + rand()%(220 - UFO_W);
				enemies[enemyCounter].position[0] = enemies[enemyCounter].position[0] << 4;
				enemies[enemyCounter].position[1] = 1;
				enemies[enemyCounter].position[1] = enemies[enemyCounter].position[1] << 4;
				if(levelCounter == 0)enemies[enemyCounter].velocity = 12;
				else if(levelCounter == 1) enemies[enemyCounter].velocity = 16;
				else if(levelCounter == 2) enemies[enemyCounter].velocity = 18;
				enemies[enemyCounter].fired = 0;
				enemies[enemyCounter].life = 1;
				enemyCounter++;
			}
			for(int i = 0; i < 4; i++){
				if(enemies[i].life > 0){
					//position update
					enemies[i].position[1] += enemies[i].velocity;
					
					//fire bullet
					if((enemies[i].position[1] >> 4) > 50 && enemies[i].fired == 0){
						enemies[i].fired = 1;
						if(eBCounter < 4){
							enemyBullets[eBCounter].life = 1;
							enemyBullets[eBCounter].position[0] = enemies[i].position[0];
							enemyBullets[eBCounter].position[1] = enemies[i].position[1] + 160;
							//use fixed point number for enemy bullet velocity and position, 270 = screenHeight - 50
							int16_t veloScalar;
							if(levelCounter == 0) veloScalar = 170;
							else if(levelCounter == 1) veloScalar = 150;
							else if(levelCounter == 2) veloScalar = 140;
							enemyBullets[eBCounter].velocity[0] = (player.position[0]*16 - enemies[i].position[0])/veloScalar;
							enemyBullets[eBCounter].velocity[1] = (player.position[1]*16 - enemies[i].position[1] - 160)/veloScalar;
							eBCounter++;
						}
					}
					
					//offset check
					if((enemies[i].position[1]>>4) >= ILI9341_TFTHEIGHT + UFO_H - 1){
						enemyCounter--;
						enemies[i] = enemies[enemyCounter];
						enemies[enemyCounter].life = 0;
					}
					
					//collision detecion against player
					if((enemies[i].position[1]>>4)> ILI9341_TFTHEIGHT - 40){
						if(abs((enemies[i].position[0]>>4) - player.position[0]) <= SPACESHIP_W/2+UFO_W/2){
							//enemies[i].life = 0;
							gameOver = 1;
						}
					}
					
					//collision detection against player bullet
					for(int j = 0; j < 4; j++){
						if(playerBullets[j].life > 0 && enemies[i].position[1] > 288){
							if((enemies[i].position[1]>>4) + UFO_H > playerBullets[j].position[1] - PLAYERBULLET_H/2){
								if(abs((enemies[i].position[0]>>4) - playerBullets[j].position[0]) <= UFO_W/2 + PLAYERBULLET_W/2){

									fillRect((enemies[i].position[0]>>4) - UFO_W/2, (enemies[i].position[1]>>4) - UFO_H - 2,
									UFO_W, UFO_H+3, ILI9341_BLACK);
									fillRect(playerBullets[j].cachePos[0] - PLAYERBULLET_W/2, playerBullets[j].cachePos[1] - PLAYERBULLET_H/2, 
									PLAYERBULLET_W, PLAYERBULLET_H, ILI9341_BLACK);
									enemyCounter--;
									pBCounter--;
									enemies[i] = enemies[enemyCounter];
									enemies[enemyCounter].life = 0;
									playerBullets[j] = playerBullets[pBCounter];//move the last one to current location 
									playerBullets[pBCounter].life = 0;
									
									score++;
								}
							}
						}
					}
					
				}
			}
			//enemy bullets
			for(int i = 0; i < 4; i++){
				if(enemyBullets[i].life > 0){
					enemyBullets[i].cachePos[0] = enemyBullets[i].position[0];
					enemyBullets[i].cachePos[1] = enemyBullets[i].position[1];
					enemyBullets[i].position[0] += enemyBullets[i].velocity[0];
					enemyBullets[i].position[1] += enemyBullets[i].velocity[1];
					
					//offset check
					if((enemyBullets[i].position[1]>>4) > 326){
						eBCounter--;
						enemyBullets[i] = enemyBullets[eBCounter];//move the last one to current location 
						enemyBullets[eBCounter].life = 0;
					}

					//collision detection against player
					if(((enemyBullets[i].position[1]>>4) + 5) > ILI9341_TFTHEIGHT - 40){
						if(abs((enemyBullets[i].position[0]>>4) - player.position[0]) <= PLAYERBULLET_W + SPACESHIP_W/2){
							//enemyBullets[i].life = 0;
							gameOver = 1;
						}
					}
				}
			}
			//laser update
			if(laserPointer < 2 && laserPowerBar >= powerBarFullValue ){
				laserCounter++;
				laserPowerBar = 0;
				laser[laserPointer].life = 1;
				if(levelCounter == 0)laser[laserPointer].velocity = 28;
				else if(levelCounter == 1) laser[laserPointer].velocity = 30;
				else if(levelCounter == 2)laser[laserPointer].velocity = 34;
				laser[laserPointer].position[0] = 10 + rand()%220;
				laser[laserPointer].position[1] = -36;
				laser[laserPointer].position[1] = laser[laserPointer].position[1]<<4;
				laserPointer++;
				
				//difficulty ramping 
				if(laserCounter == 5){
					powerBarFullValue = 4;
					SPAWANINTERVAL = 140;
					levelCounter = 1;
				}
				if(laserCounter == 10){
					powerBarFullValue = 2;
					SPAWANINTERVAL = 40;
					levelCounter = 2;
				}
			}
			for(int i = 0; i < 2; i++){
				if(laser[i].life > 0){
					laser[i].position[1] += laser[i].velocity;
				
					//offset check
					if((laser[i].position[1]>>4) > ILI9341_TFTHEIGHT + 37){
						laserPointer--;
						laser[i] = laser[laserPointer];
						laser[laserPointer].life = 0;
					}
				
				//collision detection agains player
					if((laser[i].position[1]>>4) + 36 > ILI9341_TFTHEIGHT - 40){
						if(abs(laser[i].position[0] - player.position[0]) <= SPACESHIP_W/2 + PLAYERBULLET_W){
							gameOver = 1;
						}
					}
				}
			}			
		}
		//render*********************************************************************************
		Render();
	}
 
}

void readBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color, int8_t spriteIndex) {

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    //startWrite();
    for(int16_t j=0; j<h; j++, y++) {
        for(int16_t i=0; i<w; i++) {
            if(i & 7) byte <<= 1;
            else      byte   = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
            if(byte & 0x80) {
							//drawPixel(x+i, y, color);
							if(spriteIndex == 0){
								spaceShip[y*30+x+i] = 1;
							}
							else if(spriteIndex == 1){
								UFO[y*35+x+i] = 1;
							}
						}
        }
    }
    //endWrite();
}

void GameObjectsIni(void){
//initial player
	player.position[0] = ILI9341_TFTWIDTH / 2;
	player.position[1] = ILI9341_TFTHEIGHT;
	player.velocity = 0;	
	readBitmap(0, 0, spaceShipBitmap, 30, 40, ILI9341_WHITE, 0);
	readBitmap(0, 0, UFOBitmap, 35, 22, ILI9341_WHITE, 1);
	
	//ini bullets
	for(int i =0; i < 4; i++){
		playerBullets[i].life = 0;
	}
	
	//enemy bullets
	for(int i =0; i < 4; i++){
		enemyBullets[i].life = 0;
	}
	//ini enemyies
	for(int i = 0; i < 4; i++){
		enemies[i].life = 0;
	}
	
	//initialize laser
	for(int i = 0; i < 2; i++){
		laser[i].life = 0;
	}	
}

void Render(void){
	//refresh**************************************************************
	//scoreboard
	if(lastScore != score){
		int16_t low = lastScore % 10;
		int16_t high = lastScore / 10;
		drawChar(10, 10, high + 48, ILI9341_BLACK, ILI9341_BLACK, 2);
		drawChar(22, 10, low + 48, ILI9341_BLACK, ILI9341_BLACK, 2);
	}

	//laser
	for(int i = 0; i < 2; i++){
		if(laser[i].life > 0){
			fillRect(laser[i].position[0] - PLAYERBULLET_W, (laser[i].position[1]>>4) - 39,
				PLAYERBULLET_W * 2, 3, ILI9341_BLACK);
		}
	}
	//player
	{
		setAddrWindow(player.position[0]-15, player.position[1]-40, 30, 40);
		for(int i = 0; i < 1200; i++){
			if(spaceShip[i] == 1)hspi_w16(SPI1, ILI9341_WHITE);
			else hspi_w16(SPI1, ILI9341_BLACK);
		}
	}
	
	{
		for(int i = 0; i < 4; i++){	
			//int16_t test_y = 340;
		//int16_t test_x = 60;
		int16_t pos_y_top = (enemies[i].position[1]>>4) - 22 + 1;
			int16_t c;
			int16_t rows = 22;
			if(pos_y_top <= 0) {
				rows = 22 + pos_y_top;
				c = -1 * pos_y_top * UFO_W;
				pos_y_top = 0;
			}
			else {
				c = 0;
			}
			
			int16_t pos_y_bottom = (enemies[i].position[1]>>4);
			int16_t max;
			if(pos_y_bottom >= ILI9341_TFTHEIGHT){
				rows = 22 - (pos_y_bottom - ILI9341_TFTHEIGHT + 1);
				max = 770 - (pos_y_bottom - ILI9341_TFTHEIGHT + 1) * UFO_W;
			}
			else {
				max = 770;
			}
			if(rows > 0 && enemies[i].life > 0){
				//g_counter = 0;
				setAddrWindow((enemies[i].position[0]>>4)-UFO_W/2, pos_y_top, 35, rows);
				for(;c<max;c++){
					if(UFO[c] == 1) hspi_w16(SPI1, ILI9341_BLUE);
					else hspi_w16(SPI1, ILI9341_BLACK);
				}
				//if(rows == 22)g_flag = 1;
			}
			int t = 2000;
			if(enemies[i].life == 0) rows = 0;
			t= t/22 * (22 - rows);
			for(int i = 0; i < t; i++){
				__ASM("NOP");
			}
			
			
		}
	}
	
	
	
	
	
//draw*****************************************************************************************	
	//bullets
	for(int i = 0; i < 4; i++){
		if(playerBullets[i].life > 0){
			fillRect(playerBullets[i].cachePos[0] - PLAYERBULLET_W/2, playerBullets[i].cachePos[1] - PLAYERBULLET_H/2, 
			PLAYERBULLET_W, PLAYERBULLET_H, ILI9341_BLACK);
			fillRect(playerBullets[i].position[0] - PLAYERBULLET_W/2, playerBullets[i].position[1] - PLAYERBULLET_H/2, 
			PLAYERBULLET_W, PLAYERBULLET_H, ILI9341_WHITE);
		}
		if(enemyBullets[i].life > 0){
			fillRect((enemyBullets[i].cachePos[0]>>4) - 5, (enemyBullets[i].cachePos[1]>>4) - 5, 10, 10, ILI9341_BLACK);
			fillRect((enemyBullets[i].position[0]>>4) - 5, (enemyBullets[i].position[1]>>4) - 5, 10, 10, ILI9341_BLUE);
		}
	}
	
	//laser
	for(int i = 0; i < 2; i++){
		if(laser[i].life > 0){
			fillRect(laser[i].position[0] - PLAYERBULLET_W, (laser[i].position[1]>>4) - 36,
				PLAYERBULLET_W * 2, 72, ILI9341_RED);
		}
	}
	
	//scoreboard
	int16_t low = score % 10;
	int16_t high = score / 10;
	drawChar(10, 10, high + 48, ILI9341_WHITE, ILI9341_WHITE, 2);
	drawChar(22, 10, low + 48, ILI9341_WHITE, ILI9341_WHITE, 2);
	
}
void ConfigTimer(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;//enable clock for TIM2
	
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1);
	
	TIM2->PSC = 48000 - 1;
	TIM2->ARR = 250;
	TIM2->DIER |= TIM_DIER_UIE;//enable timer interrupt
	TIM2->CR1 |= TIM_CR1_CEN;//enable timer
}
void TIM2_IRQHandler(void){
	laserPowerBar++;
	TIM2->SR &= ~(TIM_SR_UIF);
}

void EXTI0_1_IRQHandler(void){
	if((EXTI->IMR & EXTI_IMR_MR0) && (EXTI->PR & EXTI_PR_PR0)){
		fireButtonPressed = 1;
	}
	EXTI->PR |= EXTI_PR_PR0;
}	
void ConfigLED(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;//enable clock for LED
	
	//config LEDs
	GPIOC->MODER |= (1<<16)|(1<<18)|(1<<12)|(1<<14);
	GPIOC->MODER &= ~((2<<16)|(2<<18)|(2<<12)|(2<<14));
	
	GPIOC->OTYPER &= ~((1<<8)|(1<<9)|(1<<6)|(1<<7));
	GPIOC->OSPEEDR &= ~((1<<16)|(1<<18)|(1<<12)|(1<<14));
	GPIOC->PUPDR &= ~((3<<16)|(3<<18)|(3<<12)|(3<<14));
}

void ConfigButton(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;//enable clock for Button
	
	GPIOA->MODER &= ~(3|(3<<16)|(3<<18));
	GPIOA->OSPEEDR &= ~(1|(1<<8)|(1<<9));
	GPIOA->PUPDR = 0;
	GPIOA->PUPDR |= (2|(1<<16)|(1<<18));
	//GPIOA->PUPDR &= 1;
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;//eneable clock for SYSCFG
	EXTI->IMR |= EXTI_EMR_EM0;//unmask EXTI0 line
	EXTI->RTSR |= EXTI_RTSR_TR0;//set interrupt trigger to rising edge

	SYSCFG->EXTICR[0] &= ~7;  
	
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(EXTI0_1_IRQn, 1);
}

void ConfigSPI(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;//enable clock for Button
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;//GPIOB enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;//SPI clock
	
	//config SPI
	GPIOB->MODER |= ((0x2 << (PB_MOSI*2))|(0x2 << (PB_SCK*2))|(0x1 << (PB_DC*2)));//select mode
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3|GPIO_AFRL_AFSEL5);//select alternate function
	GPIOB->PUPDR |= ((0x1 << (PB_MOSI*2))|(0x1 << (PB_SCK*2)));//push up resistor 
  GPIOB->OTYPER  &= ~((0x1 << PB_MOSI)|(0x1 << PB_SCK)|(0x1 << PB_DC));// Output type: Push-pull
	GPIOB->OSPEEDR |= ((0x3 << (PB_MOSI * 2))|(0x3 << (PB_SCK * 2))|(0x3 << (PB_DC * 2))); //high speed mode
	
	GPIOA->MODER |= ((0x1 << (PA_CS * 2))|(0x1 << (PA_RST * 2)));
	
//Set initial pin values.
//(The 'Chip Select' pin tells the display if it
//should be listening. '0' means 'hey, listen!', and
//'1' means 'ignore the SCK/MOSI/DC pins'.)
	GPIOA->ODR |= (1 << PA_CS);
	//   (See the 'sspi_cmd' method for 'DC' pin info.)
	GPIOB->ODR |=  (1 << PB_DC);
	// Set SCK high to start
	GPIOB->ODR |=  (1 << PB_SCK);
	// Reset the display by pulling the reset pin low,
// delaying a bit, then pulling it high.
	GPIOA->ODR &= ~(1 << PA_RST);
	HAL_Delay(150);
	GPIOA->ODR |=  (1 << PA_RST);
	HAL_Delay(150);
	
	//initial SPI
	// Make sure that the peripheral is off, and reset it.
	SPI1->CR1 &= ~(SPI_CR1_SPE);
	RCC->APB2RSTR |=  (RCC_APB2RSTR_SPI1RST);
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST);
// Set clock polarity and phase.
	SPI1->CR1 |=  (SPI_CR1_CPOL|SPI_CR1_CPHA);
	//SPI1->CR1 &= ~(SPI_CR1_CPOL|SPI_CR1_CPHA);
	// Set the STM32 to act as a host device.
	SPI1->CR1 |=  (SPI_CR1_MSTR);
	// Set software 'Chip Select' pin.
	SPI1->CR1 |=  (SPI_CR1_SSM);	
	// (Set the internal 'Chip Select' signal.)
	SPI1->CR1 |=  (SPI_CR1_SSI);
	// Use MSB-first format.
  SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);
	// Enable the peripheral.
	SPI1->CR1 |=  (SPI_CR1_SPE);
	GPIOA->ODR &= ~(1 << PA_CS);
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
