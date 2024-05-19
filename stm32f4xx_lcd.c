/**
  ******************************************************************************
  * @file    FPU_DEMONSTRATION\Src\stm32f4xx_julia.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    07-September-2016
  * @brief   Julia-set implementation
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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


/* Includes ------------------------------------------------------------------*/
#include <stm32f4xx_lcd.h> 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#define ITERATION      	((uint32_t)128)
//#define RGB(r,g,b)  ((0xFFul<<24)|((r)<<16)|((g)<<8)|((b)>>0)) //5 red | 6 green | 5 blue
#define RGB(r,g,b) ((r>>3)<<11) | ((g>>2)<<5) | (b>>3);  // Convert colors to RGB565

void setbackgroundcolor(uint16_t size_x, uint16_t size_y, uint16_t color, uint16_t * buffer)
{
  uint16_t        index_x, index_y;

  for (index_y = 0; index_y < size_y; index_y++)
  {
    for (index_x = 0; index_x < size_x; index_x++)
    {
      /* Store the value in the buffer */
      buffer[index_x+index_y*size_x] = color; //ILI9341_COLOR_RED;
    }
  }
}

/**
* @brief  Displays new picture on LCD
* @param  image: image array to display
* @param  clut: pointer to color map
* @param  message: Text string to display
* @retval None
*/
void UpdateLCD(uint16_t * image, uint16_t * clut, uint8_t * message)
{
  /* Local variable */
  uint32_t index_x = 0x00, index_y = 0x00;

  /* Display the picture */
  for (index_y = 0; index_y < YSIZE_PHYS; index_y++)
  {
    for (index_x = 0; index_x < XSIZE_PHYS; index_x++)
    {
      image[index_x+index_y*XSIZE_PHYS]= clut[image[index_x+index_y*XSIZE_PHYS]];
    }
  }
}

/**
* @brief  Initializes memory map
* @param  clut: pointer to memory map
* @retval None
*/
void InitCLUT(uint16_t * clut)
{
  /* Local variables */
  uint32_t       iteration_number = 0x00;
  uint32_t  red = 0, green = 0, blue = 0;
  double c;

  /* Color map generation */
  for (iteration_number = 0; iteration_number < ITERATION; iteration_number++)
  {
    /* Generate red, green and blue values */
    //    red =  (iteration_number * 256 / ITERATION ) % 256;
    //    green =  (iteration_number * 256 / ITERATION ) % 256;
    //    blue =  (iteration_number * 256 / ITERATION ) % 256;

    c = 3.0 * (ITERATION - iteration_number) /ITERATION;
    red = (uint32_t)((c < 1.0) ? 255 * ( 1.0 - c) : (c > 2.0) ? 255 * (c - 2.0) : 0);
    green = (uint32_t)((c < 1.0) ? 255 * c : (c > 2.0) ? 0 : 255 * (2.0 - c));
    blue = (uint32_t)((c < 1.0) ? 0 : (c > 2.0) ? 255 * (3.0 - c) : 255 * (c - 1.0));

    clut[iteration_number] = RGB(red, green, blue);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics ************************/
