#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320
#define BPP               3

void ILI9341_init(void);
void ILI9341_set_window(int x, int y, int width, int height);
void ILI9341_clear_screen(void);
void ILI9341_push_color(unsigned char red, unsigned char green, unsigned char blue);
