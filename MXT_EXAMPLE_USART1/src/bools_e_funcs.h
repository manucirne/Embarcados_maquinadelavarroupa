/*
 * IncFile1.h
 *
 * Created: 18/04/2019 11:30:43
 *  Author: manucirne
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_
/************************************************************************/
/* Flags                                                 */
/************************************************************************/
volatile Bool f_rtt_alarme = false;
volatile Bool em_ciclo = false;
volatile int cronometro =0;
volatile int t_atual =0 ;
volatile Bool flag_inicio = true;
volatile Bool flag_rapida = false;
volatile Bool flag_diadia = false;
volatile Bool flag_pesada = false;
volatile Bool flag_pause = true;
volatile Bool flag_play = false;
volatile Bool flag_menu = false;
volatile Bool flag_animation = false;
volatile Bool trava = false;
volatile Bool flag_porta = false;
volatile Bool flag_Fporta = false;
volatile Bool flag_less = false;
volatile Bool flag_plus = false;
volatile Bool flag_enxa = false;
volatile Bool flag_centri = false;
volatile Bool flag_lava = false;
volatile Bool flag_molho = false;
volatile Bool flag_menu_desenha = false;
volatile Bool flag_fim_menu = false;

/************************************************************************/
/*Globais                                                        */
/************************************************************************/
volatile int enxatempo = 0;
volatile int lavatempo = 0;
volatile int molhotempo = 0;
volatile int centritempo = 0;
volatile int trava_count = 0;
volatile int pag_menu = 0;
#define pag_molho  0
#define pag_lava  1
#define pag_enxa  2
#define pag_centri  3
#define xLock 435
#define yLock 70

// LED
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_PIO_IDX_MASK (1 << LED_IDX)

// LED  MOTOR
#define LED_PIO_MOTOR      PIOC
#define LED_PIO_ID_MOTOR   ID_PIOC
#define LED_IDX_MOTOR      17
#define LED_PIO_IDX_MASK_MOTOR (1 << LED_IDX_MOTOR)

// LED  PORTA ABERTA
#define LED_PIO_PORTA      PIOB
#define LED_PIO_ID_PORTA   ID_PIOB
#define LED_IDX_PORTA      1
#define LED_PIO_IDX_MASK_PORTA (1 << LED_IDX_PORTA)



// Botão PORTA
#define BUT_PIO      PIOB
#define BUT_PIO_ID   ID_PIOB
#define BUT_IDX  2
#define BUT_IDX_MASK (1 << BUT_IDX)

// Botão TRANCA

#define BUT_PIO_TRANCA      PIOD
#define BUT_PIO_ID_TRANCA   ID_PIOD
#define BUT_IDX_TRANCA  28
#define BUT_IDX_MASK_TRANCA (1 << BUT_IDX_TRANCA)


/************************************************************************/
/* Funções                                                          */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask);
void io_init(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void tela_atual(t_ciclo nome, t_botao b[], int n);
void draw_screen(void);
void inicio(void);
void novo_ciclo(int Tenxa, int Tcentri, int Qenxa,int lavatempo, int molhotempo, int RPMcentri, int forte, int bolhas);
void proxima_pagina(struct mxt_device device, t_botao botoes[], int nb1, t_botao botoes2[], int nb2, t_botao botoes_menu[], int nbm, char nomes[][30], int Nnomes);
void mxt_handler(struct mxt_device *device, t_botao botoes[], uint Nbotoes);
uint32_t convert_axis_system_y(uint32_t touch_x);
uint32_t convert_axis_system_x(uint32_t touch_y);
static void mxt_init(struct mxt_device *device);
void font_draw_text(tFont *font, const char *text, int x, int y, int spacing);
int processa_touch(t_botao  b[], t_botao  *rtn, uint N ,uint x, uint y );
void BUT_init(void);
void animacao(tImage imagens[], int n );
void tela_menu(char nomes[][30], int n);

void pesada_callback(void);
void rapida_callback(void);
void diadia_callback(void);
void playpause_callback(void);
void voltar_callback(void);
void menu_callback(void);
void centrifuga_callback(void);
void enxague_callback(void);
void travar_callback(void);
void anterior_callback(void);
void proximo_callback(void);
void trava_imagem(void);
void plus_callback(void);
void less_callback(void);
void fim_menu_callback(void);
void porta_aberta(void);
void porta_callback(void);




#endif /* INCFILE1_H_ */