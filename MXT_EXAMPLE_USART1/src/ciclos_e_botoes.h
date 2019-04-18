/*
 * ciclos_e_botoes.h
 *
 * Created: 18/04/2019 11:32:57
 *  Author: manucirne
 */ 


#ifndef CICLOS_E_BOTOES_H_
#define CICLOS_E_BOTOES_H_
typedef struct botao t_botao;
typedef struct ciclo t_ciclo;

struct ciclo{
	char nome[32];           // nome do ciclo, para ser exibido
	int  molhoTempo;		 // tempo de molho da roupa
	int lavagemTempo;		 // Tempo de lavagem da roupa
	int  enxagueTempo;       // tempo que fica em cada enxague
	int  enxagueQnt;         // quantidade de enxagues
	int  centrifugacaoRPM;   // velocidade da centrifugacao
	int  centrifugacaoTempo; // tempo que centrifuga
	char heavy;              // modo pesado de lavagem
	char bubblesOn;
	
	t_ciclo *previous;
	t_ciclo *next;
};

struct botao {
	uint x;
	uint y;
	uint size;
	tImage *image;
	void (*p_handler)(void);
};







#endif /* CICLOS_E_BOTOES_H_ */