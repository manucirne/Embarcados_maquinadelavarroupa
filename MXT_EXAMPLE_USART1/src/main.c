/**
 * \file
 *
 * \brief Example of usage of the maXTouch component with USART
 *
 * This example shows how to receive touch data from a maXTouch device
 * using the maXTouch component, and display them in a terminal window by using
 * the USART driver.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 *
 * \section intro Introduction
 * This simple example reads data from the maXTouch device and sends it over
 * USART as ASCII formatted text.
 *
 * \section files Main files:
 * - example_usart.c: maXTouch component USART example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usart_serial.h: configuration of USART driver
 *
 * \section apiinfo maXTouch low level component API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and Xmega devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * This example will read data from the connected maXTouch explained board
 * over TWI. This data is then processed and sent over a USART data line
 * to the board controller. The board controller will create a USB CDC class
 * object on the host computer and repeat the incoming USART data from the
 * main controller to the host. On the host this object should appear as a
 * serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
 *
 * Connect a terminal application to the serial port object with the settings
 * Baud: 57600
 * Data bits: 8-bit
 * Stop bits: 1 bit
 * Parity: None
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"

#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"


#ifndef CONF_USART_SERIAL_H
#define CONF_USART_SERIAL_H

/** UART Interface */
#define USART_SERIAL_EXAMPLE              CONSOLE_UART
/** Baudrate setting */
#define USART_SERIAL_EXAMPLE_BAUDRATE     (115200UL)
/** Character length setting */
#define USART_SERIAL_CHAR_LENGTH          US_MR_CHRL_8_BIT
/** Parity setting */
#define USART_SERIAL_PARITY               US_MR_PAR_NO
/** Stop bits setting */
#define USART_SERIAL_STOP_BIT             US_MR_NBSTOP_1_BIT

#endif/* CONF_USART_SERIAL_H_INCLUDED */

#define LED_PIO           PIOC                  // periferico que controla o LED
#define LED_PIO_ID        12                    // ID do periférico PIOC (controla LED)
#define LED_PIO_IDX       8u                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1u << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

/**
 typedef struct {
	 const uint8_t *data;
	 uint16_t width;
	 uint16_t height;
	 uint8_t dataSize;
 } tImage;
 */
 

#include "icones/diadia.h"
#include "icones/play.h"
#include "icones/lavagens.h"
#include "icones/fast.h"
#include "icones/sol.h"
#include "icones/pesada.h"
#include "icones/fundo.h"
#include "icones/voltar.h"
#include "icones/home.h"
#include "icones/menu.h"


#define MAX_ENTRIES        3
#define STRING_LENGTH     70

#define USART_TX_MAX_LENGTH     0xff

#define MAQUINA1

struct ili9488_opt_t g_ili9488_display_opt; // apagar
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/4;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/4;


void pesada_callback(void);
void rapida_callback(void);
void diadia_callback(void);
void playpause_callback(void);
void voltar_callback(void);
void menu_callback(void);
void inicio(void);
void centrifuga_callback(void);
void enxague_callback(void);


	
/** \brief Touch event struct */

typedef struct ciclo t_ciclo;

struct ciclo{
	char nome[32];           // nome do ciclo, para ser exibido
	int  enxagueTempo;       // tempo que fica em cada enxague
	int  enxagueQnt;         // quantidade de enxagues
	int  centrifugacaoRPM;   // velocidade da centrifugacao
	int  centrifugacaoTempo; // tempo que centrifuga
	char heavy;              // modo pesado de lavagem
	char bubblesOn;  
	
	t_ciclo *previous;
	t_ciclo *next;
};

t_ciclo c_rapida = {.nome = "Rapido",
	.enxagueTempo = 5,
	.enxagueQnt = 3,
	.centrifugacaoRPM = 900,
	.centrifugacaoTempo = 5,
	.heavy = 0,
	.bubblesOn = 1

};

t_ciclo c_diadia = {.nome = "Diario",
	.enxagueTempo = 15,
	.enxagueQnt = 2,
	.centrifugacaoRPM = 1200,
	.centrifugacaoTempo = 8,
	.heavy = 0,
	.bubblesOn = 1,
};

t_ciclo c_pesada = {.nome = "Pesado",
	.enxagueTempo = 10,
	.enxagueQnt = 3,
	.centrifugacaoRPM = 1200,
	.centrifugacaoTempo = 10,
	.heavy = 1,
	.bubblesOn = 1,
};


typedef struct botao t_botao;

struct botao {
	uint x;
	uint y;
	uint size;
	tImage *image;
	void (*p_handler)(void);
};


t_botao LavagemRapida={
	.x = 170,
	.y = 20,
	.size = 100,
	.p_handler = rapida_callback,
	.image = &fast,
};

t_botao LavagemDiadia= {
	.x = 20,
	.y = 170,
	.size = 100,
	.p_handler = diadia_callback,
	.image = &sol,
};

t_botao LavagemPesada = {
	.x = 20,
	.y = 20,
	.size = 100,
	.p_handler = pesada_callback,
	.image = &pesada,
};

t_botao playpause = {
	.x = 310,
	.y = 170,
	.size = 100,
	.p_handler = playpause_callback,
	.image = &play,
};

t_botao volta = {
	.x = 310,
	.y = 20,
	.size = 100,
	.p_handler = voltar_callback,
	.image = &home,
};

t_botao opcao = {
	.x = 170,
	.y = 170,
	.size = 100,
	.p_handler = menu_callback,
	.image = &menu,
};

t_botao centrifuga = {
	.x = 170,
	.y = 170,
	.size = 100,
	.p_handler = centrifuga_callback,
	.image = &menu,
};

t_botao enxague = {
	.x = 20,
	.y = 170,
	.size = 100,
	.p_handler = enxague_callback,
	.image = &menu,
};








/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile Bool f_rtt_alarme = false;
volatile Bool em_ciclo = false;
//int passo =0;
int cronometro =0;
int t_atual =0 ;
volatile Bool flag_inicio = true;
volatile Bool flag_rapida = false;
volatile Bool flag_diadia = false;
volatile Bool flag_pesada = false;
volatile Bool flag_pause = true;
volatile Bool flag_menu = false;

char buffert [32];

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void tela_atual(t_ciclo nome, t_botao b[], int n);
void draw_screen(void);
void inicio(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(LED_PIO, LED_PIO_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}

void rapida_callback(void){
		char buf[STRING_LENGTH];
		sprintf(buf, "entrou no rapida\n");
		printf(buf);
		flag_inicio = false;
		flag_rapida = true;
		
}

void diadia_callback(void){
		char buf[STRING_LENGTH];
		sprintf(buf, "entrou no diadia\n");
		printf(buf);
		flag_inicio = false;
		flag_diadia = true;
		
}

void pesada_callback(void){
	char buf[STRING_LENGTH];
	sprintf(buf, "entrou no pesada\n");
	printf(buf);
	flag_inicio = false;
	flag_pesada = true;
	
}

void menu_callback(void){
	char buf[STRING_LENGTH];
	sprintf(buf, "entrou no menu\n");
	printf(buf);
	flag_inicio = false;
	flag_menu = true;
	
}

void enxague_callback(void){
	char buf[STRING_LENGTH];
	sprintf(buf, "entrou no enxague\n");
	printf(buf);
	
}

void centrifuga_callback(void){
	char buf[STRING_LENGTH];
	sprintf(buf, "entrou no centrifuga\n");
	printf(buf);
	
}

void playpause_callback(void){
	flag_pause = !flag_pause;
}

void voltar_callback(void){
	inicio();
	flag_inicio = true;
	flag_pause = true;
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

inicio(){
	draw_screen();
	ili9488_draw_pixmap(LavagemPesada.x,
	LavagemPesada.y,
	LavagemPesada.image->width,
	LavagemPesada.image->height,
	LavagemPesada.image->data);
	ili9488_draw_pixmap(LavagemDiadia.x,
	LavagemDiadia.y,
	LavagemDiadia.image->width,
	LavagemDiadia.image->height,
	LavagemDiadia.image->data);
	ili9488_draw_pixmap(LavagemRapida.x,
	LavagemRapida.y,
	LavagemRapida.image->width,
	LavagemRapida.image->height,
	LavagemRapida.image->data);
	ili9488_draw_pixmap(opcao.x,
	opcao.y,
	opcao.image->width,
	opcao.image->height,
	opcao.image->data);
	
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_string(LavagemPesada.x + LavagemPesada.image->width/2 -30,
	LavagemPesada.y + LavagemPesada.image->height+10,
	"Pesada" );
	ili9488_draw_string(LavagemDiadia.x + LavagemDiadia.image->width/2-30,
	LavagemDiadia.y + LavagemDiadia.image->height+10,
	"Dia a Dia" );
	ili9488_draw_string(LavagemRapida.x + LavagemRapida.image->width/2-30,
	LavagemRapida.y + LavagemRapida.image->height+10,
	"Rapida" );
	ili9488_draw_string(opcao.x + opcao.image->width/2-30,
	opcao.y + opcao.image->height+10,
	"Menu" );
	
}
tela_atual(t_ciclo cicle, t_botao b[], int n){
	
	// contador if 1 enxague if 2 centrifugação e conta o tempo em cada uma
	t_atual= cicle.enxagueTempo;
	em_ciclo = true;
	
	draw_screen();
	for (int i = 0; i < n; i++){
		ili9488_draw_pixmap(b[i].x,
		b[i].y,
		b[i].image->width,
		b[i].image->height,
		b[i].image->data);
	}
	;
						
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	ili9488_draw_string(20 , 20,cicle.nome );
	ili9488_draw_string(320 , 129,"Home" );
	ili9488_draw_string(295 , 266,"Iniciar" );
	ili9488_draw_string(295 , 296,"Lavagem" );
}

//animação tentativa tosca
void animacao(tImage imagens[], int n ){
	for(int i = 0; i < n, i++){
		ili9488_draw_pixmap(30,
		30,
		imagens[i]->width,
		imagens[i]->height,
		imagens[i]->data);
		}
	}
}

void tela_menu( t_botao b[], int n){
	
	draw_screen();
	for (int i = 0; i < n; i++){
		ili9488_draw_pixmap(b[i].x,
		b[i].y,
		b[i].image->width,
		b[i].image->height,
		b[i].image->data);
	}
	ili9488_draw_string(320 , 129,"Home" );
	ili9488_draw_string(295 , 266,"Iniciar" );
	ili9488_draw_string(295 , 296,"Lavagem" );
}

// Pensar em como chamar isso no botão de fim - possivekmente com uma flag no mein mesmo (feio, mas é o possível)
/*
void callback_fim_menu(t_ciclo *c_novo, int Tenxa, int Tcentri, int Qenxa, int RPMcentri, int forte, int bolhas){
	c_novo->nome = "Personalizado";
		c_novo->enxagueTempo = Tenxa;
		c_novo->enxagueQnt = Qenxa;
		c_novo->centrifugacaoRPM = RPMcentri;
		c_novo->centrifugacaoTempo = Tcentri;
		c_novo->heavy = forte;
		c_novo->bubblesOn = bolhas;
	
		
}
	*/
	

	
	//ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	//ili9488_draw_string(20 , 20,cicle.nome );


void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void io_init(void){
	/* led */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_PIO_IDX_MASK, PIO_DEFAULT);
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}


int processa_touch(t_botao  b[], t_botao  *rtn, uint N ,uint x, uint y ){
	char buf[STRING_LENGTH];
	
	printf( "entrou no touch");
	
	for(int i = 0;i < N;i++){
		printf( "XT: %d  YT: %d /n", x, y);
		printf( "BXT: %d  BYT: %d size: %d/n", b[i].x, b[i].y, b[i].size);
		if(x >= (b[i].x) && x <= (b[i].x + b[i].size)) {
			if(y >= (b[i].y) && y <= (b[i].y + b[i].size) ){
				*rtn = b[i];
				return 1;
			}
		}
	}
	return 0;	
}

	
static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

/**
 * \brief Set maXTouch configuration
 *
 * This function writes a set of predefined, optimal maXTouch configuration data
 * to the maXTouch Xplained Pro.
 *
 * \param device Pointer to mxt_device struct
 */




static void mxt_init(struct mxt_device *device)
{
	
	//Apagar - led para teste
	// para que possamos controlar o LED.
		pmc_enable_periph_clk(LED_PIO_ID);
		//  fazer pio como saida
		pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 0, 0, 0);
	
	//Fim do apagar led
	
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, 480, 320);
}



uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT - ILI9488_LCD_HEIGHT*touch_x/4096;
}



void mxt_handler(struct mxt_device *device, t_botao botoes[], uint Nbotoes)
{
	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	 * maximum 2 events at the time */
	do {
		/* Temporary buffer for each new touch event line */
		char buf[STRING_LENGTH];
	
		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		 // eixos trocados (quando na vertical LCD)
		uint32_t conv_x = convert_axis_system_x(touch_event.x);
		uint32_t conv_y = convert_axis_system_y(touch_event.y);
		
		/* Format a new entry in the data string that will be sent over USART */
		sprintf(buf, "X:%3d Y:%3d \n", conv_x, conv_y);
		
		/* -----------------------------------------------------*/
		t_botao bAtual;
		//sprintf(buf, " processa touch");
		if(processa_touch(botoes, &bAtual, Nbotoes, conv_x, conv_y))
			//sprintf(buf,bAtual.x);
			//printf(buf);
			bAtual.p_handler();
		//update_screen(conv_x, conv_y);
		/* -----------------------------------------------------*/

		/* Add the new string to the string buffer */
		strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		 * if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));

	/* If there is any entries in the buffer, send them over USART */
	if (i > 0) {
		usart_serial_write_packet(USART_SERIAL_EXAMPLE, (uint8_t *)tx_buf, strlen(tx_buf));
	}
}


int main(void)
{
	struct mxt_device device; /* Device data container */

	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};

	sysclk_init(); /* Initialize system clocks */
	io_init();
	board_init();  /* Initialize board */
	configure_lcd();
	
	
	// Inicializa flags.
	f_rtt_alarme = true;
	flag_inicio = true;
	flag_rapida = false;
	flag_diadia = false;
	flag_pesada = false;
	flag_pause = true;
	
	/* Initialize the mXT touch device */
	
	mxt_init(&device);
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);

	printf("\n\rmaXTouch data USART transmitter\n\r");
		
	/* -----------------------------------------------------*/
	
	
	
	inicio();
	

	/* -----------------------------------------------------*/
	t_botao botoes[] = {LavagemPesada, LavagemDiadia, LavagemRapida, opcao};
		int nb1 = 4;
	t_botao botoes2[] = {playpause, volta};
	int nb2 = 2;
	// criar os botões de ciclo para as escolhas
	t_botao botoes_menu[] = {enxague, centrifuga, playpause, volta};
		int nbm = 4;
	
	
	while (true) {
		/* Check for any pending messages and run message handler if any
		 * message is found in the queue */
		
		if(flag_inicio){
			if (mxt_is_message_pending(&device)) {
				mxt_handler(&device, botoes, nb1);
			}
		}
		
		if(flag_pesada){
			tela_atual(c_pesada, botoes2, nb2);
			flag_pesada = false;
		}
		if(flag_rapida){
			tela_atual(c_rapida, botoes2, nb2);
			flag_rapida = false;
		}
		if(flag_diadia){
			tela_atual(c_diadia, botoes2, nb2);
			flag_diadia = false;
		}
		if(flag_menu){
			tela_menu(botoes_menu, nbm);
			flag_menu = false;
		}
		
		if(!flag_inicio && !flag_menu){
			if (mxt_is_message_pending(&device)) {
				mxt_handler(&device, botoes_menu, nbm);
			}
			
		}
		if(flag_menu){
			if (mxt_is_message_pending(&device)) {
				mxt_handler(&device, botoes_menu, nbm);
			}
			
		}
		if (f_rtt_alarme){
			uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
			uint32_t irqRTTvalue  = 2;
			// reinicia RTT para gerar um novo IRQ
			
			if(em_ciclo & !flag_pause){
				sprintf(buffert,"%02d",t_atual - cronometro);
				//ili9488_draw_string(80, 200,buffert );
				 font_draw_text(&calibri_36, buffert, 80, 200, 1);
				 if (mxt_is_message_pending(&device)) {
					 mxt_handler(&device, botoes2, nb2);
				 }
				
				if (cronometro == t_atual){
					em_ciclo = false;
					ili9488_draw_string(300, 250,"ACABOU!");
					cronometro = 0;
					inicio();
					flag_pause = true;
					flag_inicio = true;
					
				}
				
				cronometro ++;	
			}
			
			RTT_init(pllPreScale, irqRTTvalue);
			
			f_rtt_alarme = false;
		}
		
		
		
		
	}

	return 0;
}
