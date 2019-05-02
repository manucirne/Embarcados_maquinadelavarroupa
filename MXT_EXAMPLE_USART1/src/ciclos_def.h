/*
 * ciclos_def.h
 *
 * Created: 02/05/2019 14:44:03
 *  Author: manucirne
 */ 


#ifndef CICLOS_DEF_H_
#define CICLOS_DEF_H_

const t_ciclo c_rapida = {.nome = "Rapido",
	.molhoTempo = 3,
	.lavagemTempo = 4,
	.enxagueTempo = 5,
	.enxagueQnt = 1,
	.centrifugacaoRPM = 900,
	.centrifugacaoTempo = 5,
	.heavy = 0,
	.bubblesOn = 1

};

const t_ciclo c_diadia = {.nome = "Diario",
	.molhoTempo = 5,
	.lavagemTempo = 7,
	.enxagueTempo = 15,
	.enxagueQnt = 2,
	.centrifugacaoRPM = 1200,
	.centrifugacaoTempo = 8,
	.heavy = 0,
	.bubblesOn = 1,
};

const t_ciclo c_pesada = {.nome = "Pesado",
	.molhoTempo = 6,
	.lavagemTempo = 10,
	.enxagueTempo = 10,
	.enxagueQnt = 3,
	.centrifugacaoRPM = 1200,
	.centrifugacaoTempo = 10,
	.heavy = 1,
	.bubblesOn = 1,
};

t_ciclo c_novo = {.nome = "Ciclo Personalizado",
	.molhoTempo = 0,
	.lavagemTempo = 0,
	.enxagueTempo = 0,
	.enxagueQnt = 0,
	.centrifugacaoRPM = 1200,
	.centrifugacaoTempo = 0,
	.heavy = 1,
	.bubblesOn = 1,
};





#endif /* CICLOS_DEF_H_ */