#include <stm32f4xx_hal.h>
#include "ILI9341.h"

#define SELECT()      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET)
#define DESELECT()    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET)
#define COMMAND()     HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET)
#define DATA()        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET)

/* IL9341 command set */
/* Level 1 commands */
#define ILI9341_NO_OP                                  0x00   /* no op */
#define ILI9341_SOFTWARE_RESET                         0x01   /* Software Reset */
#define ILI9341_READ_DISPLAY_ID                        0x04   /* Read display identification information */
#define ILI9341_READ_DISPLAY_STATUS                    0x09   /* Read Display Status */
#define ILI9341_READ_DISPLAY_POWER_MODE                0x0A   /* Read Display Power Mode */
#define ILI9341_READ_MADCTL                            0x0B   /* Read Display MADCTL */
#define ILI9341_READ_PIXEL_FORMAT                      0x0C   /* Read Display Pixel Format */
#define ILI9341_READ_IMAGE_FORMAT                      0x0D   /* Read Display Image Format */
#define ILI9341_READ_SIGNAL_MODE                       0x0E   /* Read Display Signal Mode */
#define ILI9341_READ_SELF_DIAGNOSTIC                   0x0F   /* Read Display Self-Diagnostic Result */
#define ILI9341_SLEEP_IN                               0x10   /* Enter Sleep Mode */
#define ILI9341_SLEEP_OUT                              0x11   /* Sleep out register */
#define ILI9341_PARTIAL_MODE_ON                        0x12   /* Partial Mode ON */
#define ILI9341_NORMAL_MODE_ON                         0x13   /* Normal Display Mode ON */
#define ILI9341_INVERTED_OFF                           0x20   /* Display Inversion OFF */
#define ILI9341_INVERTED_ON                            0x21   /* Display Inversion ON */
#define ILI9341_GAMMA_REGISTER                         0x26   /* Gamma register */
#define ILI9341_DISPLAY_OFF                            0x28   /* Display off register */
#define ILI9341_DISPLAY_ON                             0x29   /* Display on register */
#define ILI9341_COLUMN_ADDRESS_SET                     0x2A   /* Column address register */
#define ILI9341_PAGE_ADDRESS_SET                       0x2B   /* Page address register */
#define ILI9341_GRAM_WRITE                             0x2C   /* GRAM register */
#define ILI9341_COLOR_SET                              0x2D   /* Color SET */
#define ILI9341_GRAM_READ                              0x2E   /* Memory Read */
#define ILI9341_PARTIAL_AREA                           0x30   /* Partial Area */
#define ILI9341_VERTICAL_SCROLLING_DEFINITION          0x33   /* Vertical Scrolling Definition */
#define ILI9341_TEARING_OFF                            0x34   /* Tearing Effect Line OFF */
#define ILI9341_TEARING_ON                             0x35   /* Tearing Effect Line ON */
#define ILI9341_MEMORY_ACCESS_CONTROL                  0x36   /* Memory Access Control register*/
#define ILI9341_VERTICAL_SCROLLING_START_ADDRESS       0x37   /* Vertical Scrolling Start Address */
#define ILI9341_IDLE_OFF                               0x38   /* Idle Mode OFF */
#define ILI9341_IDLE_ON                                0x39   /* Idle Mode ON */
#define ILI9341_PIXEL_FORMAT_SET                       0x3A   /* Pixel Format register */
#define ILI9341_WRITE_MEMORY_CONTINUE                  0x3C   /* Write Memory Continue */
#define ILI9341_READ_MEMORY_CONTINUE                   0x3E   /* Read Memory Continue */
#define ILI9341_SET_TEAR_SCANLINE                      0x44   /* Set Tear Scanline */
#define ILI9341_GET_SCANLINE                           0x45   /* Get Scanline */
#define ILI9341_WRITE_BRIGHTNESS                       0x51   /* Write Brightness Display register */
#define ILI9341_READ_BRIGHTNESSV                       0x52   /* Read Display Brightness */
#define ILI9341_WRITE_CONTROL                          0x53   /* Write Control Display register*/
#define ILI9341_READ_CONTROL                           0x54   /* Read CTRL Display */
#define ILI9341_WRITE_CONTENT_ADAPTIVE_BRIGHTNESS      0x55   /* Write Content Adaptive Brightness Control */
#define ILI9341_READ_CONTENT_ADAPTIVE_BRIGHTNESS       0x56   /* Read Content Adaptive Brightness Control */
#define ILI9341_WRITE_MINIMUM_BRIGHTNESS               0x5E   /* Write CABC Minimum Brightness */
#define ILI9341_READ_MINIMUM BRIGHTNESS                0x5F   /* Read CABC Minimum Brightness */
#define ILI9341_READ_ID1                               0xDA   /* Read ID1 */
#define ILI9341_READ_ID2                               0xDB   /* Read ID2 */
#define ILI9341_READ_ID3                               0xDC   /* Read ID3 */
/* Level 2 Commands */
#define ILI9341_RGB_INTERFACE_CONTROL                  0xB0   /* RGB Interface Signal Control */
#define ILI9341_FRAME_RATE_CONTROL_1                   0xB1   /* Frame Rate Control (In Normal Mode) */
#define ILI9341_FRAME_RATE_CONTROL_2                   0xB2   /* Frame Rate Control (In Idle Mode) */
#define ILI9341_FRAME_RATE_CONTROL_3                   0xB3   /* Frame Rate Control (In Partial Mode) */
#define ILI9341_INVERSION_CONTROL                      0xB4   /* Display Inversion Control */
#define ILI9341_BLANKING_PORCH_CONTROL                 0xB5   /* Blanking Porch Control register */
#define ILI9341_DISPLAY_FUNCTION_CONTROL               0xB6   /* Display Function Control register */
#define ILI9341_ENTRY_MODE_SET                         0xB7   /* Entry Mode Set */
#define ILI9341_BACKLIGHT_1                            0xB8   /* Backlight Control 1 */
#define ILI9341_BACKLIGHT_2                            0xB9   /* Backlight Control 2 */
#define ILI9341_BACKLIGHT_3                            0xBA   /* Backlight Control 3 */
#define ILI9341_BACKLIGHT_4                            0xBB   /* Backlight Control 4 */
#define ILI9341_BACKLIGHT_5                            0xBC   /* Backlight Control 5 */
#define ILI9341_BACKLIGHT_7                            0xBE   /* Backlight Control 7 */
#define ILI9341_BACKLIGHT_8                            0xBF   /* Backlight Control 8 */
#define ILI9341_POWER_CONTROL_1                        0xC0   /* Power Control 1 register */
#define ILI9341_POWER_CONTROL_2                        0xC1   /* Power Control 2 register */
#define ILI9341_VCOM_CONTROL_1                         0xC5   /* VCOM Control 1 register */
#define ILI9341_VCOM_CONTROL_2                         0xC7   /* VCOM Control 2 register */
#define ILI9341_NV_MEMORY_WRITE                        0xD0   /* NV Memory Write */
#define ILI9341_NV_MEMORY_PROTECTION_KEY               0xD1   /* NV Memory Protection Key */
#define ILI9341_READ_NVM_STATUS                        0xD2   /* NV Memory Status Read */
#define ILI9341_READ_ID4                               0xD3   /* Read ID4 */
#define ILI9341_POSITIVE_GAMMA_CORRECTION              0xE0   /* Positive Gamma Correction register */
#define ILI9341_NEGATIVE_GAMMA_CORRECTION              0xE1   /* Negative Gamma Correction register */
#define ILI9341_DIGITAL_GAMMA_CONTROL_1                0xE2   /* Digital Gamma Control 1 */
#define ILI9341_DIGITAL_GAMA_CONTROL_2                 0xE3   /* Digital Gamma Control 2 */
#define ILI9341_INTERFACE_CONTROL                      0xF6   /* Interface control register */
/* Extended register commands */
#define ILI9341_POWER_A                                0xCB   /* Power control A register */
#define ILI9341_POWER_B                                0xCF   /* Power control B register */
#define ILI9341_DRIVER_TIMING_CONTROL_A                0xE8   /* Driver timing control A */
#define ILI9341_DRIVER_TIMING_CONTROL_B                0xEA   /* Driver timing control B */
#define ILI9341_POWER_SEQUENCE                         0xED   /* Power on sequence register */
#define ILI9341_3GAMMA_ENABLE                          0xF2   /* 3 Gamma enable register */
#define ILI9341_PUMP_RATIO_CONTROL                     0xF7   /* Pump ratio control register */

static void SPI_init(void);
static void SPI_transmit(uint8_t *data,int length);

static void ILI9341_write_command(uint8_t command);
static void ILI9341_write_data(uint8_t data);

static SPI_HandleTypeDef spi;

void ILI9341_init(void)
{
	/* initialize control pins */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef gpio_init;
	gpio_init.Pin = GPIO_PIN_2;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOC,&gpio_init);
	gpio_init.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOD,&gpio_init);
	DESELECT();

	/* initialize SPI5 */
	SPI_init();

	/* select SPI5 */
	SELECT();     // only one SPI slave on bus (keep slave selected)

	/* ILI9341 initialization sequence */

	/* software reset */
	ILI9341_write_command(ILI9341_SOFTWARE_RESET);

	/* display out of sleep mode */
	ILI9341_write_command(ILI9341_SLEEP_OUT);

	/* set memory access control */
	ILI9341_write_command(ILI9341_MEMORY_ACCESS_CONTROL);
	ILI9341_write_data(0x00);

	/* set frame rate control */
	ILI9341_write_command(ILI9341_FRAME_RATE_CONTROL_1);
	ILI9341_write_data(0x00);
	ILI9341_write_data(0x1B);

	/* set display function control */
	ILI9341_write_command(ILI9341_DISPLAY_FUNCTION_CONTROL);
	ILI9341_write_data(0x0A);
	ILI9341_write_data(0xA7);
	ILI9341_write_data(0x27);
	ILI9341_write_data(0x04);

	/* select pixel data format*/
	ILI9341_write_command(ILI9341_PIXEL_FORMAT_SET);
	ILI9341_write_data(0x66);

	/* configure RGB interface */
	ILI9341_write_command(ILI9341_RGB_INTERFACE_CONTROL);
	ILI9341_write_data(0xC2);

	/* enable display */
	ILI9341_write_command(ILI9341_DISPLAY_ON);

	/* delay */
	HAL_Delay(120);

	/* select RGB interface */
	ILI9341_write_command(ILI9341_INTERFACE_CONTROL);
	ILI9341_write_data(0x01);
	ILI9341_write_data(0x00);
	ILI9341_write_data(0x06);

	DESELECT();

	/* clear display */
	//ILI9341_clear_screen();
}

void ILI9341_set_window(int x, int y, int width, int height)
{
	int end_column = x + width - 1;
	ILI9341_write_command(ILI9341_COLUMN_ADDRESS_SET);
	uint8_t caset[] = {(unsigned int)x >> 8,x,(unsigned int)end_column >> 8,end_column};
	for (int i = 0 ; i < sizeof caset ; i++)
		ILI9341_write_data(caset[i]);

	int end_page = y + height - 1;
	COMMAND();
	ILI9341_write_command(ILI9341_PAGE_ADDRESS_SET);
	uint8_t raset[] = {(unsigned int)y >> 8,y,(unsigned int)end_page >> 8,end_page};
	for (int i = 0 ;  i < sizeof raset ; i++)
		ILI9341_write_data(raset[i]);

	ILI9341_write_command(ILI9341_GRAM_WRITE);
}

void ILI9341_push_color(unsigned char red, unsigned char green, unsigned char blue)
{
	ILI9341_write_data(red);
	ILI9341_write_data(green);
	ILI9341_write_data(blue);
}

void ILI9341_draw_pixel(int x, int y, unsigned char red, unsigned char green, unsigned char blue)
{
	ILI9341_set_window(x,y,1,1);
	ILI9341_push_color(red,green,blue);
}

void ILI9341_clear_screen(void)
{
	ILI9341_set_window(0,0,DISPLAY_WIDTH,DISPLAY_HEIGHT);
	for (int i = 0 ; i < DISPLAY_WIDTH * DISPLAY_HEIGHT ; i++)
		ILI9341_push_color(0xFF,0x00,0x00);
}

static void SPI_init(void)
{
	spi.Instance = SPI5;
	spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi.Init.Mode = SPI_MODE_MASTER;
	spi.Init.DataSize = SPI_DATASIZE_8BIT;
	spi.Init.Direction = SPI_DIRECTION_2LINES;
	spi.Init.NSS = SPI_NSS_SOFT;
	spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	HAL_SPI_Init(&spi);         // calls HAL_SPI_MspInit()
	__HAL_SPI_ENABLE(&spi);     // enable SPI instance
}

static void SPI_transmit(uint8_t *data,int length)
{
	for (int i = 0 ; i < length ; i++)
	{
		while (!(SPI5->SR & SPI_SR_TXE))
			;
		SPI5->DR = data[i];
	}
	while (SPI5->SR & SPI_SR_BSY)
		;
}

static void ILI9341_write_command(uint8_t command)
{
	COMMAND();
	SPI_transmit(&command, 1);
}

static void ILI9341_write_data(uint8_t data)
{
	DATA();
	SPI_transmit(&data, 1);
}

/***** ILI9341 initialization sequence (long) *****/
//	ILI9341_write_command(0xCA);
//	ILI9341_write_data(0xC3);
//	ILI9341_write_data(0x08);
//	ILI9341_write_data(0x50);
//	ILI9341_write_command(ILI9341_POWER_B);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0xC1);
//	ILI9341_write_data(0x30);
//	ILI9341_write_command(ILI9341_POWER_SEQUENCE);
//	ILI9341_write_data(0x64);
//	ILI9341_write_data(0x03);
//	ILI9341_write_data(0x12);
//	ILI9341_write_data(0x81);
//	ILI9341_write_command(ILI9341_DRIVER_TIMING_CONTROL_A);
//	ILI9341_write_data(0x85);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x78);
//	ILI9341_write_command(ILI9341_POWER_A);
//	ILI9341_write_data(0x39);
//	ILI9341_write_data(0x2C);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x34);
//	ILI9341_write_data(0x02);
//	ILI9341_write_command(ILI9341_PUMP_RATIO_CONTROL);
//	ILI9341_write_data(0x20);
//	ILI9341_write_command(ILI9341_DRIVER_TIMING_CONTROL_B);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x00);
//	ILI9341_write_command(ILI9341_FRAME_RATE_CONTROL_1);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x1B);
//	ILI9341_write_command(ILI9341_POWER_CONTROL_1);
//	ILI9341_write_data(0x10);
//	ILI9341_write_command(ILI9341_POWER_CONTROL_2);
//	ILI9341_write_data(0x10);
//	ILI9341_write_command(ILI9341_VCOM_CONTROL_1);
//	ILI9341_write_data(0x45);
//	ILI9341_write_data(0x15);
//	ILI9341_write_command(ILI9341_VCOM_CONTROL_1);
//	ILI9341_write_data(0x90);
//	ILI9341_write_command(ILI9341_MEMORY_ACCESS_CONTROL);
//	ILI9341_write_data(0xC8);
//	ILI9341_write_command(ILI9341_3GAMMA_ENABLE);
//	ILI9341_write_data(0x00);
//	ILI9341_write_command(ILI9341_RGB_INTERFACE_CONTROL);
//	ILI9341_write_data(0xC2);
//	ILI9341_write_command(ILI9341_DISPLAY_FUNCTION_CONTROL);
//	ILI9341_write_data(0x0A);
//	ILI9341_write_data(0xA7);
//	ILI9341_write_data(0x27);
//	ILI9341_write_data(0x04);
//	ILI9341_write_command(ILI9341_COLUMN_ADDRESS_SET);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0xEF);
//	ILI9341_write_command(ILI9341_PAGE_ADDRESS_SET);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x01);
//	ILI9341_write_data(0x3F);
//	ILI9341_write_command(ILI9341_INTERFACE_CONTROL);
//	ILI9341_write_data(0x01);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x06);
//	ILI9341_write_command(ILI9341_GAMMA_REGISTER);
//	ILI9341_write_data(0x01);
//	ILI9341_write_command(ILI9341_POSITIVE_GAMMA_CORRECTION);
//	ILI9341_write_data(0x0F);
//	ILI9341_write_data(0x29);
//	ILI9341_write_data(0x24);
//	ILI9341_write_data(0x0C);
//	ILI9341_write_data(0x0E);
//	ILI9341_write_data(0x09);
//	ILI9341_write_data(0x4E);
//	ILI9341_write_data(0x78);
//	ILI9341_write_data(0x3C);
//	ILI9341_write_data(0x09);
//	ILI9341_write_data(0x13);
//	ILI9341_write_data(0x05);
//	ILI9341_write_data(0x17);
//	ILI9341_write_data(0x11);
//	ILI9341_write_data(0x00);
//	ILI9341_write_command(ILI9341_NEGATIVE_GAMMA_CORRECTION);
//	ILI9341_write_data(0x00);
//	ILI9341_write_data(0x16);
//	ILI9341_write_data(0x1B);
//	ILI9341_write_data(0x04);
//	ILI9341_write_data(0x11);
//	ILI9341_write_data(0x07);
//	ILI9341_write_data(0x31);
//	ILI9341_write_data(0x33);
//	ILI9341_write_data(0x42);
//	ILI9341_write_data(0x05);
//	ILI9341_write_data(0x0C);
//	ILI9341_write_data(0x0A);
//	ILI9341_write_data(0x28);
//	ILI9341_write_data(0x2F);
//	ILI9341_write_data(0x0F);
//	ILI9341_write_command(ILI9341_SLEEP_OUT);
//	HAL_Delay(200);
//	ILI9341_write_command(ILI9341_DISPLAY_ON);





