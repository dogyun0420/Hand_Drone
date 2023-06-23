static const uint8_t PROGMEM drone [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x8C, 0xFF, 0x80, 0x01, 0xFE, 0x63, 0xFE,
0x7F, 0xCC, 0xFF, 0x80, 0x03, 0xFE, 0x23, 0xFE, 0x7F, 0xFF, 0xFF, 0x80, 0x03, 0xFF, 0xFF, 0xFE,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x0F, 0xE0, 0x00, 0xF8, 0x00,
0x00, 0x1E, 0x00, 0x0F, 0xF0, 0x00, 0xF8, 0x00, 0x00, 0x1E, 0x00, 0x1F, 0xF8, 0x00, 0xF8, 0x00,
0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00,
0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00,
0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF0, 0x00, 0x00,
0x00, 0x00, 0x07, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xE0, 0x00, 0x00,
0x00, 0x00, 0x3F, 0xFC, 0x7F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFE, 0xFF, 0xFC, 0x00, 0x00,
0x00, 0x00, 0x7F, 0xFE, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x3F, 0xFC, 0x7F, 0x00, 0x00,
0x00, 0x01, 0xF0, 0x07, 0xC0, 0x1F, 0x00, 0x00, 0x00, 0x01, 0xE0, 0x00, 0x00, 0x07, 0x80, 0x00,
0x00, 0x01, 0xE0, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x01, 0xE0, 0x00, 0x00, 0x07, 0x00, 0x00,
0x00, 0x01, 0xC0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x01, 0xE0, 0x00, 0x00, 0x07, 0x00, 0x00,
0x00, 0x01, 0xE0, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x03, 0x00, 0x00
};
static const uint8_t PROGMEM up[] = { /* 0X00,0X01,0X16,0X00,0X16,0X00, */
0X00,0X78,0X00,0X00,0XFC,0X00,0X01,0XFE,0X00,0X03,0XFF,0X00,0X07,0XFF,0X80,0X0F,
0XFF,0XC0,0X1F,0XFF,0XE0,0X3F,0XFF,0XF0,0X7F,0XFF,0XF8,0XFF,0XFF,0XFC,0XFF,0XFF,
0XFC,0XFE,0XFD,0XFC,0XFC,0XFC,0XFC,0X38,0XFC,0X70,0X00,0XFC,0X00,0X00,0XFC,0X00,
0X00,0XFC,0X00,0X00,0XFC,0X00,0X00,0XFC,0X00,0X00,0XFC,0X00,0X00,0X78,0X00,0X00,
0X30,0X00,};
static const uint8_t PROGMEM down[] = { /* 0X00,0X01,0X16,0X00,0X16,0X00, */
0X00,0X30,0X00,0X00,0X78,0X00,0X00,0XFC,0X00,0X00,0XFC,0X00,0X00,0XFC,0X00,0X00,
0XFC,0X00,0X00,0XFC,0X00,0X00,0XFC,0X00,0X38,0XFC,0X70,0XFC,0XFC,0XFC,0XFE,0XFD,
0XFC,0XFF,0XFF,0XFC,0XFF,0XFF,0XFC,0X7F,0XFF,0XF8,0X3F,0XFF,0XF0,0X1F,0XFF,0XE0,
0X0F,0XFF,0XC0,0X07,0XFF,0X80,0X03,0XFF,0X00,0X01,0XFE,0X00,0X00,0XFC,0X00,0X00,
0X78,0X00,};
static const uint8_t PROGMEM left[] = { /* 0X00,0X01,0X16,0X00,0X16,0X00, */
0X00,0X78,0X00,0X00,0XF8,0X00,0X01,0XFC,0X00,0X03,0XFC,0X00,0X07,0XFC,0X00,0X0F,
0XF8,0X00,0X1F,0XF0,0X00,0X3F,0XE0,0X00,0X7F,0XFF,0XF0,0XFF,0XFF,0XF8,0XFF,0XFF,
0XFC,0XFF,0XFF,0XFC,0XFF,0XFF,0XF8,0X7F,0XFF,0XF0,0X3F,0XE0,0X00,0X1F,0XF0,0X00,
0X0F,0XF8,0X00,0X07,0XFC,0X00,0X03,0XFC,0X00,0X01,0XFC,0X00,0X00,0XF8,0X00,0X00,
0X78,0X00,};
static const uint8_t PROGMEM right[] = { /* 0X00,0X01,0X16,0X00,0X16,0X00, */
0X00,0X78,0X00,0X00,0X7C,0X00,0X00,0XFE,0X00,0X00,0XFF,0X00,0X00,0XFF,0X80,0X00,
0X7F,0XC0,0X00,0X3F,0XE0,0X00,0X1F,0XF0,0X3F,0XFF,0XF8,0X7F,0XFF,0XFC,0XFF,0XFF,
0XFC,0XFF,0XFF,0XFC,0X7F,0XFF,0XFC,0X3F,0XFF,0XF8,0X00,0X1F,0XF0,0X00,0X3F,0XE0,
0X00,0X7F,0XC0,0X00,0XFF,0X80,0X00,0XFF,0X00,0X00,0XFE,0X00,0X00,0X7C,0X00,0X00,
0X78,0X00,};

void Init_LCD(void);
void drawtext(unsigned char xpos, unsigned char ypos, char *text, uint16_t color, uint8_t text_size);
void Axis_LCD_Display(void);
void Axis_Data_Display(void);
void Disconnected_Display(void);
void Power_Display(void);
