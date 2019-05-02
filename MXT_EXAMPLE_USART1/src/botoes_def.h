/*
 * botoes_def.h
 *
 * Created: 02/05/2019 14:41:50
 *  Author: manucirne
 */ 


#ifndef BOTOES_DEF_H_
#define BOTOES_DEF_H_

const t_botao LavagemRapida={
	.x = 170,
	.y = 20,
	.size = 100,
	.p_handler = rapida_callback,
	.image = &fast,
};

const t_botao LavagemDiadia= {
	.x = 20,
	.y = 170,
	.size = 100,
	.p_handler = diadia_callback,
	.image = &sol,
};

const t_botao LavagemPesada = {
	.x = 20,
	.y = 20,
	.size = 100,
	.p_handler = pesada_callback,
	.image = &pesada,
};

const t_botao playpause = {
	.x = 310,
	.y = 170,
	.size = 100,
	.p_handler = playpause_callback,
	.image = &play,
};
const t_botao fim_menu = {
	.x = 310,
	.y = 170,
	.size = 100,
	.p_handler = fim_menu_callback,
	.image = &play,
};


const t_botao homeB = {
	.x = 310,
	.y = 20,
	.size = 100,
	.p_handler = voltar_callback,
	.image = &home,
};

const t_botao anterior = {
	.x = 0,
	.y = 120,
	.size = 100,
	.p_handler = anterior_callback,
	.image = &voltar,
};

const t_botao proximo = {
	.x = 380,
	.y = 120,
	.size = 100,
	.p_handler = proximo_callback,
	.image = &proxima,
};

const t_botao opcao = {
	.x = 170,
	.y = 170,
	.size = 100,
	.p_handler = menu_callback,
	.image = &menu,
};

const t_botao centrifuga = {
	.x = 170,
	.y = 170,
	.size = 100,
	.p_handler = centrifuga_callback,
	.image = &menu,
};

const t_botao enxague = {
	.x = 20,
	.y = 170,
	.size = 100,
	.p_handler = enxague_callback,
	.image = &menu,
};

const t_botao travar = {
	.x = 410,
	.y = 15,
	.size = 48,
	.p_handler = travar_callback,
	.image = &lock,
};

const t_botao plusB = {
	.x = 200,
	.y = 100,
	.size = 48,
	.p_handler = plus_callback,
	.image = &plus,
};

const t_botao lessB = {
	.x = 200,
	.y = 220,
	.size = 48,
	.p_handler = less_callback,
	.image = &less,
};



#endif /* BOTOES_DEF_H_ */