// main.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <sstream>
#include <fstream>
#include <string>
#include <iomanip>
#include <filesystem>
#include <format>
#include <cmath>
#include <algorithm>

//INCLUSAO DE BIBLIOTECAS
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include "environment.h"
#include "robot.h"

#define GENS 51                    //NUMERO DE GERACOES
#define POPULATION 500             //TAMANHO DA POPULACAO
#define CROSSING 350               //70% DA POPULACAO
#define REPRODUCTION 150           //30% DA POPULACAO
#define HEIGHT 200                 //ALTURA DA MATRIZ
#define WIDTH 200                  //LARGURA DA MATRIZ
#define RUNS 1                     //NUMERO DE TESTES DE CADA INDIVIDUOS
#define EXECUTE 2000               //NUMERO DE EXECUCOES DA ARVORE POR TESTE
#define LIMIT 1000                 //LIMITA COMPRIMENTO DO INDIVIDUO
#define ANGLE 5                    //ANGULO QUE O ROBO SE VIRA
#define HIT_DISTANCE 1             //DISTANCIA CONSIDERADA PARA TOQUE
#define VIEW_ANGLE 30              //DEFINE ANGULO DA VISAO LOCAL

/*
 ******************************************************************************************************
 VISAO LOCAL, FUNCAO IFBALL, ALIGN ALTERADO, ECONOMIZA MEMORIA, ENXERGA OBSTACULOS
 ******************************************************************************************************

 -------------------------------------------------------------
 BASEADO EM ALOCACAO DINAMICA DE MEMORIA BUSCA NODE LEFT-RIGHT
 -------------------------------------------------------------

  OBJETIVO:
	INDIVIDUO DEVE PERCORRER A MATRIZ E ALCANCAR A BOLA SE MOVIMENTANDO O MINIMO POSSIVEL, E DESVIANDO
	DOS OBSTACULOS.
	AS POSICOES INICIAIS TANTO DA BOLA COMO DO ROBO SAO ALEATORIAS. O ROBO NAO AGUARDA A BOLA PARAR
	NOVAMENTE PARA SEGUI-LA.

  FITNESS:
	SERA LEVADO EM CONSIDERACAO A DISTANCIA INICIAL ENTRE O ROBO E A BOLA INICIAL E DEPOIS DE CADA TOQUE,
	O NUMERO DE PASSOS DO ROBO E O NUMERO DE VEZES QUE ELE TOCOU A BOLA.

  FUNCOES:
	PROGN3  (3), EXECUTA TRES RAMOS EM SEQUENCIA;
	PROGN2  (2), EXECUTA DOIS RAMOS EM SEQUENCIA;
	IFWALL  (I), EXECUTA O RAMO ESQUERDO SE HOUVER PAREDE E VICE-VERSA;
	IFBALL  (C), EXECUTA O RAMO ESQUERDO SE ENXERGAR BOLA E VICE-VERSA.

  TERMINAIS:
	WALKFRONT (F), FAZ ROBO ANDAR PRA FRENTE;
	WALKBACK  (B), FAZ ROBO ANDAR PRA TRAS;
	RIGHT     (R), FAZ ROBO VIRAR A DIREITA (DEPENDE DE `ANGLE`);
	LEFT      (L), FAZ ROBO VIRAR A ESQUERDA (DEPENDE DE `ANGLE`);
	ALIGN     (A), DIRECIONA ROBO PARA A BOLA (NAO DEPENDE DE `ANGLE`, MAXIMO DE 30 GRAUS).

  ----------------------------------------------------------
  PARAMETROS IMPORTANTES:
	NUMERO DE GERACOES(G): DEFINIDO PELO PARAMETRO `GENS`.
	TAMANHO DA POPULACAO(M): DEFINIDO PELO PARAMETRO `POPULATION`.
	PROBABILIDADE DE REPRODUCAO = DEFINIDA PELO PARAMETRO `REPRODUCTION`.
	PROBABILIDADE DE CRUZAMENTO = DEFINIDA PELO PARAMETRO `CROSSING`.
	PROBABILIDADE DE MUTACAO    =  0%
  ----------------------------------------------------------

  OUTROS DADOS:
	COMPLEXIDADE DOS INDIVIDUOS E LIMITADA (PARAMETRO `LIMIT`).
*/

struct tree {           //ESTRUTURA COM TOPO E TRES RAMIFICACOES

	char info;           //GUARDA APENAS UM CARACTERE REFERENTE A APENAS UM TERMINAL OU FUNCAO,
						 //VISA ECONOMIA DE MEMORIA E FACILIDADE DE IMPLEMENTACAO.
	struct tree* top;
	struct tree* right;
	struct tree* center;
	struct tree* left;
};

struct ind {            //INDIVIDUO COMPLETO COM ARVORE E FITNESS, UTILIZADO APENAS NA RAIZ
	signed long int fitness;
	struct tree* root;
};

struct ball_data {          //POSICAO E DIRECAO DA BOLA
	int dir;
	double lin;
	double col;
};

int n, nbef;                          //CONTROLA NUMERO DE SORTEIOS DO INDIVIDUOS

//AMBIENTE now handled by Environment class
int robot_track[HEIGHT][WIDTH];         //CAMINHO DO ROBO
int ball_track[HEIGHT][WIDTH];          //CAMINHO DA BOLA

Environment env;

Robot robot(env);  // Global robot instance

int fit, unfit, randnum;              //GUARDAM FITNESS

char crossing_auxiliar;               //GUARDA POSICAO DO RAMO DE TROCA

double distance[2];                   //DISTANCIAS

double initial_distance;              //DISTANCIA INICIAL MEDIDA A CADA HIT

struct tree* crossing_pointer;        //RETORNO DE PONTO DE CRUZAMENTO

FILE* file_pointer;

unsigned char ball_movements = 0, ball_hits = 0;


//********************************
//* RECEBE RAIZ E LIBERA MEMORIA *
//* DE UM INDIVIDUO              *
//********************************
void freemem(struct tree* pointer)
{
	if (pointer->left)
		freemem(pointer->left);

	if (pointer->center)
		freemem(pointer->center);

	if (pointer->right)
		freemem(pointer->right);

	free(pointer);
}

//****************
//* MEDE FITNESS *
//****************
double fitness(const Robot& robot, struct ball_data ball)
{
	double calc;

	double distlin = ball.lin - robot.getLine();
	double distcol = ball.col - robot.getColumn();

	double distnow = sqrt((distcol * distcol) + (distlin * distlin));

	if (distance[0] == 0)
	{
		distance[0] = distnow;

		return(0);
	}
	else
	{
		distance[1] = distance[0];
		distance[0] = distnow;

		calc = 10 * (distance[1] - distance[0]);

		return((double)calc);
	}
}

int obstacle(double robotLin, double robotCol, struct ball_data ball, double angle) {
    // Normalize angle
    while (angle >= 360) angle -= 360;
    while (angle < 0) angle += 360;
    
    double lin = robot.getLine();
    double col = robot.getColumn();
    
    // Check path to ball
    while ((int)ball.lin != (int)lin && (int)ball.col != (int)col) {
        if (!env.isPathClear(lin, col, angle, 1))
            return 0;
            
        lin = lin - sin((M_PI * angle) / 180);
        col = col + cos((M_PI * angle) / 180);
    }
    
    return 1;
}

//******************************************
//* EXECUTA RAMO ESQUERDO SE ENXERGAR BOLA *
//* DO CONTRARIO EXECUTA RAMO DIREITO      *
//******************************************
int ifball(const Robot& robot, struct ball_data ball)
{
	double Dlin, Dcol, angle;


	Dlin = ball.lin - robot.getLine();
	Dcol = ball.col - robot.getColumn();

	angle = (180 / M_PI) * atan(Dlin / Dcol);

	if (Dcol >= 0)
		angle = 360 - angle;

	else
		angle = 180 - angle;

	if ((int)(angle - robot.getDirection()) > VIEW_ANGLE || (int)(angle - robot.getDirection()) < -VIEW_ANGLE)
		if (obstacle(robot.getLine(), robot.getColumn(), ball, angle)) return (0);

	return (1);

}

//*********************************
//* RECEBE DADOS DO ROBO E        *
//* VERIFICA SE ELE TOCOU NA BOLA *
//*********************************
void hitverify(const Robot& robot, struct ball_data* ball)
{
	double Dlin, Dcol;

	Dlin = ball->lin - robot.getLine();
	Dcol = ball->col - robot.getColumn();

	if (sqrt((Dlin * Dlin) + (Dcol * Dcol)) <= HIT_DISTANCE)
	{
		//ball_movements = (rand() % 46) + 5;  //BALLMOVE RECEBE NUMERO DE 5 a 50;

		if (!nbef)
		{
			unfit = n / initial_distance;

			nbef = n;
		}
		else
		{
			unfit += (n - nbef) / initial_distance;

			nbef = n;
		}

		ball_movements = 40;
		ball->dir = robot.getDirection();
		ball_hits++;
	}
}

//********************************************
//* ALOCA MEMORIA PARA UMA ESTRUTURA         *
//* E RETORNA UM PONTEIRO PARA ESSA ALOCACAO *
//********************************************
struct tree* alloc(void)
{
	struct tree* pointer;


	pointer = (struct tree*) malloc(sizeof(struct tree));
	if (!pointer)
	{
		printf("\n\n\tFalha na alocacao de memoria!!!\n\n");
		exit(1);
	}

	return(pointer);
}

//*********************************
//* RECEBE PONTEIRO, SORTEIA INFO *
//* E O RETORNA                   *
//*********************************
struct tree* sort(struct tree* pointer)
{
	//int randnum;       //GUARDA NUMERO SORTEADO
	struct tree* aux;  //AUXILIAR DE INICIALIZACAO DO PONTEIRO


	aux = alloc();

	aux->top = NULL;
	aux->left = NULL;
	aux->center = NULL;
	aux->right = NULL;

	if (n == 1)
		randnum = 0;

	if (randnum != 4)
	{
		if (n == 1)
			randnum = 1;     //NA PRIMEIRA EXECUCAO SORTEIA `PROGN3`

		else if (n < LIMIT)
			randnum = (rand() % 8) + 1;  //RANDNUM RECEBE NUMERO DE 1 a 8

		else randnum = (rand() % 4) + 5;  //SE EXCEDER `LIMIT` SORTEIOS PASSA A SORTEAR SOMENTE TERMINAIS
	}
	else
	{
		randnum = 9;
	}

	n++;  //GUARDA COMPLEXIDADE DO INDIVIDUO

	switch (randnum)
	{
	case 1:
		pointer->info = 51;  //GUARDA `3` EM INFO REFERENTE A `PROGN3`
		break;
	case 2:
		pointer->info = 50;  //GUARDA `2` EM INFO REFERENTE A `PROGN2`
		break;
	case 3:
		pointer->info = 73;  //GUARDA `I` EM INFO REFERENTE A `IFWALL`
		break;
	case 4:
		pointer->info = 67;  //GUARDA `C` EM INFO REFERENTE A `IFBALL`
		break;
	case 5:
		pointer->info = 70;  //GUARDA `F` EM INFO REFERENTE A `WALKFRONT`
		break;
	case 6:
		pointer->info = 66;  //GUARDA `B` EM INFO REFERENTE A `WALKBACK`
		break;
	case 7:
		pointer->info = 76;  //GUARDA `L` EM INFO REFERENTE A `LEFT`
		break;
	case 8:
		pointer->info = 82;  //GUARDA `R` EM INFO REFERENTE A `RIGHT`
		break;
	case 9:
		pointer->info = 65;  //GUARDA `A` EM INFO REFERENTE A `ALIGN`
		break;
	}

	if (randnum == 1)
	{
		pointer->left = aux;
		pointer->center = aux;
		pointer->right = aux;
	}
	else if (randnum < 5)
	{
		pointer->left = aux;
		pointer->center = NULL;
		pointer->right = aux;
	}
	else
	{
		pointer->left = NULL;
		pointer->center = NULL;
		pointer->right = NULL;
	}

	freemem(aux);

	return(pointer);
}

//**************************************************
//* RECEBE PONTEIRO, CHAMA FUNCAO QUE SORTEIA INFO *
//* E O POSICIONA NA ARVORE                        *
//**************************************************
struct tree* maketree(struct tree* pointer)
{
	struct tree* aux;

	pointer = sort(pointer);  //RECEBE INFORMACAO SORTEADA

	if (pointer->left)  //CHECA VALIDADE DO PONTEIRO
	{
		aux = alloc();        //ALOCA aux

		pointer->left = aux;  //left APONTA aux

		aux->top = pointer;   //top APONTA pointer

		aux = maketree(aux);  //CONTINUA ARVORE
	}

	if (pointer->center)  //CHECA VALIDADE DO PONTEIRO
	{
		aux = alloc();          //ALOCA aux

		pointer->center = aux;  //center APONTA aux

		aux->top = pointer;     //top APONTA pointer

		aux = maketree(aux);    //CONTINUA ARVORE
	}

	if (pointer->right)  //CHECA VALIDADE DO PONTEIRO
	{
		aux = alloc();         //ALOCA aux

		pointer->right = aux;  //right APONTA aux

		aux->top = pointer;    //top APONTA pointer

		aux = maketree(aux);   //CONTINUA ARVORE
	}

	return(pointer);
}

//****************************************
//* RECEBE UM RAMO E UM PONTEIRO E       *
//* FAZ UMA COPIA DO SEGUNDO NO PRIMEIRO *
//****************************************
struct tree* copy(struct tree* pointer1, struct tree* pointer2) {
	struct tree* aux;

	pointer1->info = pointer2->info;

	if (pointer2->left) {
		aux = alloc();

		pointer1->left = aux;

		aux->top = pointer1;

		copy(aux, pointer2->left);
	} else
		pointer1->left = NULL;

	if (pointer2->center) {
		aux = alloc();

		pointer1->center = aux;

		aux->top = pointer1;

		copy(aux, pointer2->center);
	} else
		pointer1->center = NULL;

	if (pointer2->right) {
		aux = alloc();

		pointer1->right = aux;

		aux->top = pointer1;

		copy(aux, pointer2->right);
	} else
		pointer1->right = NULL;

	return(pointer1);
}

//************************
//* RECEVE INDIVIDUO     *
//* E O IMPRIME NO VIDEO *
//************************
void print(struct tree* pointer) {
	printf("%c ", pointer->info);

	if (pointer->left)
		print(pointer->left);

	if (pointer->center)
		print(pointer->center);

	if (pointer->right)
		print(pointer->right);
}

//*******************************************
//* RECEBE DADOS DA BOLA E DO ROBO E        *
//* MOVIMENTA BOLA DE ACORDO COM O AMBIENTE *
//*******************************************
void moveball(struct ball_data* ball, const Robot& robot) {
	double testlin = 0, testcol = 0, Drest = 0, xi, xf, yi, yf, Dlin, Dcol;
	double lin, col;
	int angle;

	angle = ball->dir;
	lin = ball->lin;
	col = ball->col;

	while (ball->dir < 0)
		ball->dir += 360;

	while (ball->dir > 360)
		ball->dir -= 360;

	angle = ball->dir;

	if (ball_movements > 0) {
		ball_movements--;

		testlin = ball->lin - (2 * sin((M_PI * ball->dir) / 180));
		testcol = ball->col + (2 * cos((M_PI * ball->dir) / 180));

		if (testlin < 0 || testlin > 199 || testcol < 0 || testcol > 199) {
			while (testlin < 0 || testlin > 199 || testcol < 0 || testcol > 199) {
				if (ball->dir == 90 || ball->dir == 270) {
					xi = xf = ball->col;
					yi = yf = -1;
				} else if (ball->dir == 0 || ball->dir == 180) {
					xi = xf = -1;
					yi = yf = ball->lin;
				} else {
					xi = ball->col + (ball->lin / tan((ball->dir * M_PI) / 180));
					xf = ball->col - ((199 - ball->lin) / tan((ball->dir * M_PI) / 180));

					yi = ball->lin + (tan((ball->dir * M_PI) / 180) * ball->col);
					yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (199 - ball->col));
				}

				if (ball->dir > 0 && ball->dir < 180 && xi <= 199 && xi >= 0) {
					Dlin = ball->lin;
					Dcol = xi - ball->col;
					Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

					testlin = 1;
					testcol = xi;

					if (xi >= 198)
						ball->dir = 225;
					else if (xi <= 1)
						ball->dir = 315;
					else
						ball->dir = 360 - ball->dir;
				} else if (ball->dir < 270 && ball->dir > 90 && yi <= 199 && yi >= 0) {
					Dlin = yi - ball->lin;
					Dcol = ball->col;
					Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

					testlin = yi;
					testcol = 1;

					if (yi >= 198)
						ball->dir = 45;
					else if (yi <= 1)
						ball->dir = 315;
					else
						ball->dir = 180 - ball->dir;
				} else if (ball->dir < 360 && ball->dir > 180 && xf <= 199 && xf >= 0) {
					Dlin = 199 - ball->lin;
					Dcol = xf - ball->col;
					Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

					testlin = 198;
					testcol = xf;

					if (xf >= 198)
						ball->dir = 135;
					else if (xf <= 1)
						ball->dir = 45;
					else
						ball->dir = 360 - ball->dir;
				} else if (((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270)) && yf <= 199 && yf >= 0) {
					Dlin = yf - ball->lin;
					Dcol = 199 - ball->col;
					Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

					testlin = yf;
					testcol = 198;

					if (yf >= 198)
						ball->dir = 135;
					else if (yf <= 1)
						ball->dir = 225;
					else
						ball->dir = 180 - ball->dir;
				} else {
					ball->dir -= 180;
				}

				while (ball->dir < 0)
					ball->dir += 360;

				while (ball->dir > 360)
					ball->dir -= 360;

				angle = ball->dir;

				testlin -= Drest * sin((M_PI * ball->dir) / 180);
				testcol += Drest * cos((M_PI * ball->dir) / 180);
			}

			Dlin = robot.getLine() - ball->lin;
			Dcol = robot.getColumn() - ball->col;
			initial_distance = sqrt((Dlin * Dlin) + (Dcol * Dcol));

		} else if (env.getCell((int)testlin, (int)testcol) && (int)testlin < 197 && (int)testlin > 2
			&& (int)testcol < 197 && (int)testcol > 2) {
			if ((int)testlin < 66) {
				if ((int)testcol < 66) {
					if (ball->dir == 90 || ball->dir == 270) {
						xi = xf = ball->col;
						yi = yf = -1;
					} else if (ball->dir == 0 || ball->dir == 180) {
						xi = xf = -1;
						yi = yf = ball->lin;
					} else {
						xi = ball->col - ((25 - ball->lin) / tan((ball->dir * M_PI) / 180));
						xf = ball->col - ((40 - ball->lin) / tan((ball->dir * M_PI) / 180));

						yi = ball->lin - (tan((ball->dir * M_PI) / 180) * (25 - ball->col));
						yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (40 - ball->col));
					}

					if (xi <= 40 && xi >= 25 && ball->dir < 360 && ball->dir > 180) {
						Dlin = 25 - ball->lin;
						Dcol = xi - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 24;
						testcol = xi;

						ball->dir = 360 - ball->dir;
					} else if (xf <= 40 && xf >= 25 && ball->dir < 180 && ball->dir > 0) {
						Dlin = 40 - ball->lin;
						Dcol = xf - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 41;
						testcol = xf;

						ball->dir = 360 - ball->dir;
					} else if (yi <= 40 && yi >= 25 && ((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270))) {
						Dlin = yi - ball->lin;
						Dcol = 25 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yi;
						testcol = 24;

						ball->dir = 180 - ball->dir;
					} else if (yf <= 40 && yf >= 25 && ball->dir < 270 && ball->dir > 90) {
						Dlin = yf - ball->lin;
						Dcol = 40 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yf;
						testcol = 41;

						ball->dir = 180 - ball->dir;
					}
				} else if ((int)testcol < 132) {
					if (ball->dir == 90 || ball->dir == 270) {
						xi = xf = ball->col;
						yi = yf = -1;
					} else if (ball->dir == 0 || ball->dir == 180) {
						xi = xf = -1;
						yi = yf = ball->lin;
					} else {
						xi = ball->col - ((25 - ball->lin) / tan((ball->dir * M_PI) / 180));
						xf = ball->col - ((40 - ball->lin) / tan((ball->dir * M_PI) / 180));

						yi = ball->lin - (tan((ball->dir * M_PI) / 180) * (91 - ball->col));
						yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (106 - ball->col));
					}

					if (xi <= 106 && xi >= 91 && ball->dir < 360 && ball->dir > 180) {
						Dlin = 25 - ball->lin;
						Dcol = xi - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 24;
						testcol = xi;

						ball->dir = 360 - ball->dir;
					} else if (xf <= 106 && xf >= 91 && ball->dir < 180 && ball->dir > 0) {
						Dlin = 40 - ball->lin;
						Dcol = xf - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 41;
						testcol = xf;

						ball->dir = 360 - ball->dir;
					} else if (yi <= 40 && yi >= 25 && ((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270))) {
						Dlin = yi - ball->lin;
						Dcol = 91 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yi;
						testcol = 90;

						ball->dir = 180 - ball->dir;
					} else if (yf <= 40 && yf >= 25 && ball->dir < 270 && ball->dir > 90) {
						Dlin = yf - ball->lin;
						Dcol = 106 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yf;
						testcol = 107;

						ball->dir = 180 - ball->dir;
					}
				} else {
					if (ball->dir == 90 || ball->dir == 270) {
						xi = xf = ball->col;
						yi = yf = -1;
					} else if (ball->dir == 0 || ball->dir == 180) {
						xi = xf = -1;
						yi = yf = ball->lin;
					} else {
						xi = ball->col - ((25 - ball->lin) / tan((ball->dir * M_PI) / 180));
						xf = ball->col - ((40 - ball->lin) / tan((ball->dir * M_PI) / 180));

						yi = ball->lin - (tan((ball->dir * M_PI) / 180) * (160 - ball->col));
						yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (175 - ball->col));
					}

					if (xi <= 175 && xi >= 160 && ball->dir < 360 && ball->dir > 180) {
						Dlin = 25 - ball->lin;
						Dcol = xi - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 24;
						testcol = xi;

						ball->dir = 360 - ball->dir;
					} else if (xf <= 175 && xf >= 160 && ball->dir < 180 && ball->dir > 0) {
						Dlin = 40 - ball->lin;
						Dcol = xf - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 41;
						testcol = xf;

						ball->dir = 360 - ball->dir;
					} else if (yi <= 40 && yi >= 25 && ((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270))) {
						Dlin = yi - ball->lin;
						Dcol = 160 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yi;
						testcol = 159;

						ball->dir = 180 - ball->dir;
					} else if (yf <= 40 && yf >= 25 && ball->dir < 270 && ball->dir > 90) {
						Dlin = yf - ball->lin;
						Dcol = 175 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yf;
						testcol = 176;

						ball->dir = 180 - ball->dir;
					}
				}
			} else if ((int)testlin < 132) {
				if ((int)testcol < 66) {
					if (ball->dir == 90 || ball->dir == 270) {
						xi = xf = ball->col;
						yi = yf = -1;
					} else if (ball->dir == 0 || ball->dir == 180) {
						xi = xf = -1;
						yi = yf = ball->lin;
					} else {
						xi = ball->col - ((91 - ball->lin) / tan((ball->dir * M_PI) / 180));
						xf = ball->col - ((106 - ball->lin) / tan((ball->dir * M_PI) / 180));

						yi = ball->lin - (tan((ball->dir * M_PI) / 180) * (25 - ball->col));
						yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (40 - ball->col));
					}

					if (xi <= 40 && xi >= 25 && ball->dir < 360 && ball->dir > 180) {
						Dlin = 91 - ball->lin;
						Dcol = xi - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 90;
						testcol = xi;

						ball->dir = 360 - ball->dir;
					} else if (xf <= 40 && xf >= 25 && ball->dir < 180 && ball->dir > 0) {
						Dlin = 106 - ball->lin;
						Dcol = xf - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 107;
						testcol = xf;

						ball->dir = 360 - ball->dir;
					} else if (yi <= 106 && yi >= 91 && ((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270))) {
						Dlin = yi - ball->lin;
						Dcol = 25 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yi;
						testcol = 24;

						ball->dir = 180 - ball->dir;
					} else if (yf <= 106 && yf >= 91 && ball->dir < 270 && ball->dir > 90) {
						Dlin = yf - ball->lin;
						Dcol = 40 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yf;
						testcol = 41;

						ball->dir = 180 - ball->dir;
					}
				} else if ((int)testcol < 132) {
					if (ball->dir == 90 || ball->dir == 270) {
						xi = xf = ball->col;
						yi = yf = -1;
					} else if (ball->dir == 0 || ball->dir == 180) {
						xi = xf = -1;
						yi = yf = ball->lin;
					} else {
						xi = ball->col - ((91 - ball->lin) / tan((ball->dir * M_PI) / 180));
						xf = ball->col - ((106 - ball->lin) / tan((ball->dir * M_PI) / 180));

						yi = ball->lin - (tan((ball->dir * M_PI) / 180) * (91 - ball->col));
						yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (106 - ball->col));
					}

					if (xi <= 106 && xi >= 91 && ball->dir < 360 && ball->dir > 180) {
						Dlin = 91 - ball->lin;
						Dcol = xi - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 90;
						testcol = xi;

						ball->dir = 360 - ball->dir;
					} else if (xf <= 106 && xf >= 91 && ball->dir < 180 && ball->dir > 0) {
						Dlin = 106 - ball->lin;
						Dcol = xf - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 107;
						testcol = xf;

						ball->dir = 360 - ball->dir;
					} else if (yi <= 106 && yi >= 91 && ((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270))) {
						Dlin = yi - ball->lin;
						Dcol = 91 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yi;
						testcol = 90;

						ball->dir = 180 - ball->dir;
					} else if (yf <= 106 && yf >= 91 && ball->dir < 270 && ball->dir > 90) {
						Dlin = yf - ball->lin;
						Dcol = 106 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yf;
						testcol = 107;

						ball->dir = 180 - ball->dir;
					}
				} else {
					if (ball->dir == 90 || ball->dir == 270) {
						xi = xf = ball->col;
						yi = yf = -1;
					} else if (ball->dir == 0 || ball->dir == 180) {
						xi = xf = -1;
						yi = yf = ball->lin;
					} else {
						xi = ball->col - ((91 - ball->lin) / tan((ball->dir * M_PI) / 180));
						xf = ball->col - ((106 - ball->lin) / tan((ball->dir * M_PI) / 180));

						yi = ball->lin - (tan((ball->dir * M_PI) / 180) * (160 - ball->col));
						yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (175 - ball->col));
					}

					if (xi <= 175 && xi >= 160 && ball->dir < 360 && ball->dir > 180) {
						Dlin = 91 - ball->lin;
						Dcol = xi - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 90;
						testcol = xi;

						ball->dir = 360 - ball->dir;
					} else if (xf <= 175 && xf >= 160 && ball->dir < 180 && ball->dir > 0) {
						Dlin = 106 - ball->lin;
						Dcol = xf - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 107;
						testcol = xf;

						ball->dir = 360 - ball->dir;
					} else if (yi <= 106 && yi >= 91 && ((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270))) {
						Dlin = yi - ball->lin;
						Dcol = 160 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yi;
						testcol = 159;

						ball->dir = 180 - ball->dir;
					} else if (yf <= 106 && yf >= 91 && ball->dir < 270 && ball->dir > 90) {
						Dlin = yf - ball->lin;
						Dcol = 175 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yf;
						testcol = 176;

						ball->dir = 180 - ball->dir;
					}
				}
			} else {
				if ((int)testcol < 66) {
					if (ball->dir == 90 || ball->dir == 270) {
						xi = xf = ball->col;
						yi = yf = -1;
					} else if (ball->dir == 0 || ball->dir == 180) {
						xi = xf = -1;
						yi = yf = ball->lin;
					} else {
						xi = ball->col - ((160 - ball->lin) / tan((ball->dir * M_PI) / 180));
						xf = ball->col - ((175 - ball->lin) / tan((ball->dir * M_PI) / 180));

						yi = ball->lin - (tan((ball->dir * M_PI) / 180) * (25 - ball->col));
						yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (40 - ball->col));
					}

					if (xi <= 40 && xi >= 25 && ball->dir < 360 && ball->dir > 180) {
						Dlin = 160 - ball->lin;
						Dcol = xi - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 159;
						testcol = xi;

						ball->dir = 360 - ball->dir;
					} else if (xf <= 40 && xf >= 25 && ball->dir < 180 && ball->dir > 0) {
						Dlin = 175 - ball->lin;
						Dcol = xf - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 176;
						testcol = xf;

						ball->dir = 360 - ball->dir;
					} else if (yi <= 175 && yi >= 160 && ((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270))) {
						Dlin = yi - ball->lin;
						Dcol = 25 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yi;
						testcol = 24;

						ball->dir = 180 - ball->dir;
					} else if (yf <= 175 && yf >= 160 && ball->dir < 270 && ball->dir > 90) {
						Dlin = yf - ball->lin;
						Dcol = 40 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yf;
						testcol = 41;

						ball->dir = 180 - ball->dir;
					}
				} else if ((int)testcol < 132) {
					if (ball->dir == 90 || ball->dir == 270) {
						xi = xf = ball->col;
						yi = yf = -1;
					} else if (ball->dir == 0 || ball->dir == 180) {
						xi = xf = -1;
						yi = yf = ball->lin;
					} else {
						xi = ball->col - ((160 - ball->lin) / tan((ball->dir * M_PI) / 180));
						xf = ball->col - ((175 - ball->lin) / tan((ball->dir * M_PI) / 180));

						yi = ball->lin - (tan((ball->dir * M_PI) / 180) * (91 - ball->col));
						yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (106 - ball->col));
					}

					if (xi <= 106 && xi >= 91 && ball->dir < 360 && ball->dir > 180) {
						Dlin = 160 - ball->lin;
						Dcol = xi - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 159;
						testcol = xi;

						ball->dir = 360 - ball->dir;
					} else if (xf <= 106 && xf >= 91 && ball->dir < 180 && ball->dir > 0) {
						Dlin = 175 - ball->lin;
						Dcol = xf - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 176;
						testcol = xf;

						ball->dir = 360 - ball->dir;
					} else if (yi <= 175 && yi >= 160 && ((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270))) {
						Dlin = yi - ball->lin;
						Dcol = 91 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yi;
						testcol = 90;

						ball->dir = 180 - ball->dir;
					} else if (yf <= 175 && yf >= 160 && ball->dir < 270 && ball->dir > 90) {
						Dlin = yf - ball->lin;
						Dcol = 107 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yf;
						testcol = 107;

						ball->dir = 180 - ball->dir;
					}
				} else {
					if (ball->dir == 90 || ball->dir == 270) {
						xi = xf = ball->col;
						yi = yf = -1;
					} else if (ball->dir == 0 || ball->dir == 180) {
						xi = xf = -1;
						yi = yf = ball->lin;
					} else {
						xi = ball->col - ((160 - ball->lin) / tan((ball->dir * M_PI) / 180));
						xf = ball->col - ((175 - ball->lin) / tan((ball->dir * M_PI) / 180));

						yi = ball->lin - (tan((ball->dir * M_PI) / 180) * (160 - ball->col));
						yf = ball->lin - (tan((ball->dir * M_PI) / 180) * (175 - ball->col));
					}

					if (xi <= 175 && xi >= 160 && ball->dir < 360 && ball->dir > 180) {
						Dlin = 160 - ball->lin;
						Dcol = xi - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 159;
						testcol = xi;

						ball->dir = 360 - ball->dir;
					} else if (xf <= 175 && xf >= 160 && ball->dir < 180 && ball->dir > 0) {
						Dlin = 175 - ball->lin;
						Dcol = xf - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = 176;
						testcol = xf;

						ball->dir = 360 - ball->dir;
					} else if (yi <= 175 && yi >= 160 && ((ball->dir < 90 && ball->dir >= 0) || (ball->dir <= 360 && ball->dir > 270))) {
						Dlin = yi - ball->lin;
						Dcol = 160 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yi;
						testcol = 159;

						ball->dir = 180 - ball->dir;
					} else if (yf <= 175 && yf >= 160 && ball->dir < 270 && ball->dir > 90) {
						Dlin = yf - ball->lin;
						Dcol = 175 - ball->col;
						Drest = 2 - sqrt((Dlin * Dlin) + (Dcol * Dcol));

						testlin = yf;
						testcol = 176;

						ball->dir = 180 - ball->dir;
					}
				}
			}

			testlin -= Drest * sin((M_PI * ball->dir) / 180);
			testcol += Drest * cos((M_PI * ball->dir) / 180);

			Dlin = robot.lin - ball->lin;
			Dcol = robot.col - ball->col;
			initial_distance = sqrt((Dlin * Dlin) + (Dcol * Dcol));
		}

		if ((int)testlin == 0)
			testlin++;
		else if ((int)testlin == 199)
			testlin--;

		if ((int)testcol == 0)
			testcol++;
		else if ((int)testcol == 199)
			testcol--;

		env.setCell((int)ball->lin, (int)ball->col, 0);

		ball->lin = testlin;
		ball->col = testcol;

		ball_track[(int)ball->lin][(int)ball->col] = 1;
		env.setCell((int)ball->lin, (int)ball->col, 1);
	}
}

//************************************
//* RECEBE DADOS DO ROBO E DA BOLA E *
//* UTILIZANDO OUTRAS FUNCOES SIMULA *
//* OS DOIS NO AMBIENTE              *
//************************************
void execute(struct tree* pointer, struct ball_data* ball)
{
	moveball(ball, robot);

	switch (pointer->info)
	{
	case 51:                    //PROGN3
		execute(pointer->left, ball);    //CHAMA RAMO ESQUERDO

		execute(pointer->center, ball);  //CHAMA RAMO CENTRAL

		execute(pointer->right, ball);   //CHAMA RAMO DIREITO

		break;

	case 50:                    //PROGN2
		execute(pointer->left, ball);

		execute(pointer->right, ball);

		break;

	case 73:                    //IFWALL
		if (robot.isNearWall())
			execute(pointer->left, ball);

		else
			execute(pointer->right, ball);

		break;

	case 67:                    //IFBALL
		if (robot.canSeeBall(ball->lin, ball->col))
			execute(pointer->left, ball);

		else
			execute(pointer->right, ball);

		break;

	case 65:                    //ALIGN
		robot.align(ball->lin, ball->col);

		robot_track[(int)robot.getLine()][(int)robot.getColumn()] = n;

		n++;

		break;

	case 70:                   //WALKFRONT
	{
		robot.walkFront();
		robot_track[(int)robot.getLine()][(int)robot.getColumn()] = n;
		n++;
	}

	hitverify(robot, ball);

	break;

	case 66:                   //WALKBACK
	{
		robot.walkBack();
		robot_track[(int)robot.getLine()][(int)robot.getColumn()] = n;
		n++;
	}

	hitverify(robot, ball);

	break;

	case 76:                   //LEFT
		robot.turnLeft();
		robot_track[(int)robot.getLine()][(int)robot.getColumn()] = n;
		n++;
		break;

	case 82:                   //RIGHT
		robot.turnRight();
		robot_track[(int)robot.getLine()][(int)robot.getColumn()] = n;
		n++;

		break;
	}
}

//*******************************
//* RECEBE MATRIZ E A PREENCHE  *
//* CONFORME O AMBIENTE INICIAL *
//*******************************
// Removed initializeEnvironment - now using Environment class initialize() method

//****************************************
//* RECEBE MATRIZ DE IMAGEM E A PREENCHE *
//* DE ACORDO COM O AMBIENTE INICIAL     *
//****************************************
void setbest(unsigned char matriz[HEIGHT][WIDTH][3])
{
	int lin, col;


	for (lin = 0; lin < HEIGHT; lin++)
		for (col = 0; col < WIDTH; col++)
		{
			if (lin == 0 || lin == 199 || col == 0 || col == 199)
				matriz[lin][col][0] = matriz[lin][col][1] = matriz[lin][col][2] = 255;

			else
				matriz[lin][col][0] = matriz[lin][col][1] = matriz[lin][col][2] = 0;
		}
}

//************************************
//* RECEBE INDIVIDUO E RETORNA O     * 
//* NUMERO DE PONTOS DE COMPLEXIDADE *
//************************************
int length(struct tree* pointer)
{
	if (pointer->left)
	{
		length(pointer->left);
		n++;
	}

	if (pointer->center)
	{
		length(pointer->center);
		n++;
	}

	if (pointer->right)
	{
		length(pointer->right);
		n++;
	}

	return(n);
}

//****************************************************
//* RECEBE O INDIVIDUO E O PONTO DE CRUZAMENTO       *
//* E RETORNA UM PONTEIRO PARA O PONTO DE CRUZAMENTO *
//****************************************************
struct tree* count(struct tree* pointer)
{
	if (pointer->left)
	{
		n--;

		if (n > 0)
			count(pointer->left);

		else if (n == 0)
		{
			crossing_auxiliar = 0;
			crossing_pointer = pointer->left;
			return crossing_pointer;
		}
	}

	if (pointer->center)
	{
		n--;

		if (n > 0)
			count(pointer->center);

		else if (n == 0)
		{
			crossing_auxiliar = 1;
			crossing_pointer = pointer->center;
			return crossing_pointer;
		}
	}

	if (pointer->right)
	{
		n--;

		if (n > 0)
			count(pointer->right);

		else if (n == 0)
		{
			crossing_auxiliar = 2;
			crossing_pointer = pointer->right;
			return crossing_pointer;
		}
	}

	return crossing_pointer;
}

//********************************
//* RECEBE O INDIVIDUO E O SALVA *
//* EM UM ARQUIVO EM DISCO       *
//********************************
void save(struct tree* pointer)
{
	fprintf(file_pointer, "%c", pointer->info);

	if (pointer->left)
		save(pointer->left);

	if (pointer->center)
		save(pointer->center);

	if (pointer->right)
		save(pointer->right);
}

//***************************************************
//* RECEBE PONTEIRO VAZIO, LE ARQUIVO DE UM         *
//* INDIVIDUO E RETORNA O PONTEIRO COM A INFORMACAO *
//***************************************************
struct tree* read(struct tree* pointer)
{
	struct tree* aux;
	char read_aux;


	aux = alloc();

	aux->top = NULL;
	aux->left = NULL;
	aux->center = NULL;
	aux->right = NULL;

	fscanf(file_pointer, "%c", &read_aux);

	pointer->info = read_aux;

	if (pointer->info == 70 || pointer->info == 66 || pointer->info == 76
		|| pointer->info == 82 || pointer->info == 65)
	{
		pointer->left = NULL;
		pointer->center = NULL;
		pointer->right = NULL;
	}

	else if (pointer->info == 50 || pointer->info == 73 || pointer->info == 67)
	{
		pointer->left = aux;
		pointer->center = NULL;
		pointer->right = aux;
	}

	else
	{
		pointer->left = aux;
		pointer->center = aux;
		pointer->right = aux;
	}

	freemem(aux);

	return(pointer);
}

//****************************************************
//* RECEBE O PONTEIRO, CHAMA A FUNCAO READ, MONTA    *
//* O INDIVIDUO E RETORNA O PONTEIRO DESSE INDIVIDUO *
//****************************************************
struct tree* load(struct tree* pointer) {
	struct tree* aux;

	pointer = read(pointer);

	if(pointer->left) {
		aux = alloc();

		pointer->left = aux;

		aux->top = pointer;

		aux = load(aux);
	} else
		pointer->left = NULL;

	if(pointer->center) {
		aux = alloc();

		pointer->center = aux;

		aux->top = pointer;

		aux = load(aux);
	} else
		pointer->center = NULL;

	if(pointer->right) {
		aux = alloc();

		pointer->right = aux;

		aux->top = pointer;

		aux = load(aux);
	} else
		pointer->right = NULL;

	return(pointer);
}

//*******************************************
//* DESENHA OBSTACULO NA MATRIZ DO AMBIENTE *
//*******************************************
void initializeBall(struct ball_data* ball) {
    do {
        ball->col = (int)(rand() % (WIDTH-2)) + 1;
        ball->lin = (int)(rand() % (HEIGHT-2)) + 1;

        if (env.getCell((int)ball->lin, (int)ball->col)) {
            // If position is occupied, randomly adjust either line or column
            if (rand() % 2) {
                ball->col = (int)(rand() % (WIDTH-2)) + 1;
            } else {
                ball->lin = (int)(rand() % (HEIGHT-2)) + 1;
            }
        }
    } while (env.getCell((int)ball->lin, (int)ball->col));
    
    env.setCell((int)ball->lin, (int)ball->col, 1);
}

// Robot initialization now handled by Robot class

void drawbox(int lin, int col, int size) {
	int i, j;

	for (i = lin; i < (lin + size); i++)
		for (j = col; j < (col + size); j++)
			env.setCell(i, j, 2);
}

//*****************************************
//* DESENHA OBSTACULO NA MATRIZ DA IMAGEM *
//*****************************************
void drawboxim(unsigned char matriz[HEIGHT][WIDTH][3], int lin, int col, int size) {
	int i, j;

	for (i = lin; i < (lin + size); i++)
		for (j = col; j < (col + size); j++)
		{
			matriz[i][j][0] = matriz[i][j][1] = matriz[i][j][2] = 255;
		}
}

/**
 * Counts the number of existing files.
 *
 * @param baseName The base name of the files to check (e.g., "robot").
 * @param extension The extension of the file to check (e.g., "txt")
 * @return The number of existing files.
 */
int countExistingFiles(const std::string& baseName, const std::string& extension) {
    int count = 0;
    for (int i = 0;; ++i) {
		std::stringstream ss;
		ss << baseName << std::setfill('0') << std::setw(3) << i << extension;
        std::ifstream file(ss.str());
        if (!std::filesystem::exists(ss.str())) {
            break;
        }
        file.close();
        ++count;
    }
    return count;
}

//************************
//*----------------------*
//*- PROGRAMA PRINCIPAL -*
//*----------------------*
//************************
int main(void)
{
	//SEMENTE DO RANDOM
	long ltime = time(NULL);
	int stime = (unsigned)ltime / 2;
	srand(stime);

	struct ind rob[POPULATION],                 //PONTEIRO INICIAL DO INDIVIDUO
	crossing_results[CROSSING];

	int lin, col;                        //AUXILIARES DE IMPRESSAO DA MATRIZ

	int random_number;                           //AUXILIAR DE SORTEIO

	int i, j, k, gap, list[CROSSING],    //LOOPING E AUXILIAR DE SORTEIO
		crosspoint[2],                     //PONTOS DE CRUZAMENTO
		tree_position[2];                        //POSICAO DO RAMO

	struct ind x, parent[2];             //AUXILIARES DE ORGANIZACAO E CRUZAMENTO

	char a[5];                           //AUXILIAR DE ORGANIZACAO

	FILE* track_files_pointer;

	struct tree* pointer[4];             //PONTEIROS PARA CRUZAMENTO

	unsigned long int best = 0;          //GUARDAM FITNESS

	unsigned char best_track[HEIGHT][WIDTH][3];

	// Robot is now handled by the global Robot instance

	struct ball_data ball;                 //GUARDA TODOS OS DADOS NECESSARIOS DA BOLA

	int generation, nsort, num;

	float generation_average, run_average = 0;

	int generation_maximum, run_maximum = 0;

	time_t run_start_time, run_end_time;

	//HORA DO INICIO
	run_start_time = time(NULL);

	//LIMPA TELA
	system("clear");
	printf("\n******************************************************************************\n");

	//---------------------------------------
	//ALOCA INICIO PARA INDIVIDUOS EM ARQUIVO
	//---------------------------------------
	printf("\n\t\tCarregando\n");
	for (i = 0; i < 100; i++)
	{
		char filename[20];

		snprintf(filename, sizeof(filename), "robots/rb%.3dtr.txt", i);

		if ((file_pointer = fopen(filename, "r")) == NULL)
		{
			printf("\n\tNao existem sucessores para \"%s\"!!!", filename);
			printf("\n\tNovos individuos serao criados.\n\n");

			i = 0;

			break;
		}

		rob[i].root = alloc();

		rob[i].root->top = NULL;

		rob[i].fitness = 0;

		rob[i].root = load(rob[i].root);

		fclose(file_pointer);
	}

	//-------------------------------------------------------------
	//ALOCA INICIO PARA TODOS OS INDIVIDUOS E SORTEIA OS INDIVIDUOS
	//-------------------------------------------------------------
	printf("\n\t\tAlocando e Sorteando\n");
	for (; i < POPULATION; i++)
	{
		rob[i].root = alloc();    //ALOCA

		rob[i].root->top = NULL;

		rob[i].fitness = 0;

		n = 1;                            //SORTEIA
		rob[i].root = maketree(rob[i].root);

		if (i < CROSSING)
		{
			crossing_results[i].root = alloc();

			crossing_results[i].fitness = 0;
		}
	}

	//----------------------------------------
	//VERIFICA SE JA EXISTEM ARQUIVOS DE DADOS
	//----------------------------------------
	{
		int number_of_data_files = countExistingFiles("data/data", ".txt");

		std::stringstream ssFilename;
        ssFilename << "data/data" << std::setfill('0') << std::setw(3) << number_of_data_files << "." << "txt";

		if ((file_pointer = fopen(ssFilename.str().c_str(), "w+")) == NULL) {
			printf("\n\n\tFalha ao tentar criar arquivo!!!\n\n");
			exit(1);
		} else {
			fprintf(file_pointer, "ROBO SEGUIDOR v.1.2\nLuiz Carlos Maia Junior\n");
			fprintf(file_pointer, "GERACAO\tMEDIA\t\tMAIOR\n");
		}
	}

	//*******************
	//* INICIA GERACOES *
	//*******************
	for (generation = 0; generation < GENS; generation++)
	{
		generation_average = generation_maximum = 0;

		//EXECUCAO DO INDIVIDUO
		printf("\n\t\tExecutando ->       ");

		best = 0;

		for (i = 0; i < POPULATION; i++)
		{
			double aux, aux1;

			aux1 = i;

			aux = ((aux1 + 1) / POPULATION) * 100;

			printf("\b\b\b\b");

			if (aux >= 10)
				printf("\b");

			printf("%3.1f%%", aux);
			fflush(stdout);

			rob[i].fitness = 0;

			env.initialize();

			for (j = 0; j < RUNS; j++)
			{
				for (lin = 0; lin < HEIGHT; lin++)   //AMBIENTE DE PASSOS
					for (col = 0; col < WIDTH; col++)
						ball_track[lin][col] = robot_track[lin][col] = 0;


				fit = unfit = ball_hits = 0;

				robot.initialize();
                                initializeBall(&ball);

				robot_track[(int)robot.getLine()][(int)robot.getColumn()] = 1;
				ball_track[(int)ball.lin][(int)ball.col] = 1;


				distance[0] = 0;

				{
					double Dlin, Dcol;

					nbef = 0;

					Dlin = ball.lin - robot.getLine();
					Dcol = ball.col - robot.getColumn();

					initial_distance = sqrt((Dlin * Dlin) + (Dcol * Dcol));
				}


				for (n = 0; n < EXECUTE;)
					execute(rob[i].root, &ball);

				//****************************************************
				//CALCULO DO FITNESS

				rob[i].fitness += 1500 * ball_hits - unfit;
				//rob[i].fitness += 100 * ball_hits - unfit;

				//printf("\n\n%d\n\n", unfit); fflush(stdout);

				//****************************************************

				if (rob[i].fitness >= best && j == (RUNS - 1))
				{
					setbest(best_track);    //MATRIZ QUE VAI VIRAR IMAGEM
					drawboxim(best_track, 25, 25, 16);
					drawboxim(best_track, 25, 91, 16);
					drawboxim(best_track, 25, 160, 16);
					drawboxim(best_track, 91, 25, 16);
					drawboxim(best_track, 91, 91, 16);
					drawboxim(best_track, 91, 160, 16);
					drawboxim(best_track, 160, 25, 16);
					drawboxim(best_track, 160, 91, 16);
					drawboxim(best_track, 160, 160, 16);

					best = rob[i].fitness;

					for (lin = 0; lin < HEIGHT; lin++)
						for (col = 0; col < WIDTH; col++)
						{
							if (robot_track[lin][col] > 0)
							{
								if (robot_track[lin][col] < 20)
								{
									best_track[lin][col][0] = 255;
									best_track[lin][col][1] = 255;
									best_track[lin][col][2] = 0;
								}
								else
								{
									best_track[lin][col][0] = 255;
									best_track[lin][col][1] = 0;
									best_track[lin][col][2] = 0;
								}
							}
							if (ball_track[lin][col] == 1)
							{
								best_track[lin][col][0] = 0;
								best_track[lin][col][1] = 255;
								best_track[lin][col][2] = 0;
							}
						}
				}
			}

			rob[i].fitness /= j;
		}

		//*********************************
		//* ORGANIZA EM ORDEM DECRESCENTE *
		//*********************************
		printf("\n\n\t\tOrganizando\n");

		std::sort(rob, rob + POPULATION, [](const ind& a, const ind& b) {
			return a.fitness > b.fitness; // Sort descending based on fitness
		});

		//**************            
		//* CRUZAMENTO *
		//**************
		if (generation < (GENS - 1))
		{
			printf("\n\t\tCruzando\n");

			for (i = 0; i < CROSSING; i++)
				list[i] = i;

			nsort = CROSSING;

			for (num = 0; num < CROSSING; num += 2)
			{
				for (i = 0; i < 2; i++)  //SORTEIO DOS PAIS
				{
					if (nsort > 0)
						random_number = rand() % nsort;

					else random_number = 0;

					parent[i].root = alloc();

					parent[i].root = copy(parent[i].root, rob[list[random_number]].root);

					nsort--;

					list[random_number] = list[nsort];

					n = 1;

					crosspoint[i] = (rand() % (length(parent[i].root) - 1)) + 2;
				}

				for (i = 0; i < 2; i++)
				{
					n = crosspoint[i] - 1;

					count(parent[i].root);

					pointer[i] = crossing_pointer;

					tree_position[i] = crossing_auxiliar;
				}

				for (i = 0; i < 2; i++)
					pointer[i + 2] = pointer[i]->top;

				switch (tree_position[0])
				{
				case 0:
					pointer[2]->left = pointer[1];

					pointer[1]->top = pointer[2];
					break;

				case 1:
					pointer[2]->center = pointer[1];

					pointer[1]->top = pointer[2];
					break;

				case 2:
					pointer[2]->right = pointer[1];

					pointer[1]->top = pointer[2];
					break;
				}

				switch (tree_position[1])
				{
				case 0:
					pointer[3]->left = pointer[0];

					pointer[0]->top = pointer[3];
					break;

				case 1:
					pointer[3]->center = pointer[0];

					pointer[0]->top = pointer[3];
					break;

				case 2:
					pointer[3]->right = pointer[0];

					pointer[0]->top = pointer[3];
					break;
				}

				crossing_results[num].root = parent[0].root;
				crossing_results[num + 1].root = parent[1].root;
			}

			/*
			  -----------------------
			  MANUSEIO DE MEMORIA
			  -----------------------
				*/
			printf("\n\t\tManuseio de memoria\n");

			for (num = 0; num < CROSSING; num++)
			{
				freemem(rob[REPRODUCTION + num].root);     //LIBERA 'ROB'

				rob[REPRODUCTION + num].root = alloc();    //REALOCA 'ROB'
				rob[REPRODUCTION + num].root->top = NULL;

				rob[REPRODUCTION + num].root = crossing_results[num].root;
			}

		}

		printf("\n\t\tGeracao: %d", generation);

		for (num = 0; num < POPULATION; num++)
		{
			generation_average += rob[num].fitness;

			if (rob[num].fitness > generation_maximum)
				generation_maximum = rob[num].fitness;

			if (generation_maximum > run_maximum)
				run_maximum = generation_maximum;
		}

		generation_average /= POPULATION;

		run_average += generation_average;

		printf("\n\n\tMEDIA GERACAO = %3.4f, MAIOR GERACAO = %3d\n", generation_average, generation_maximum);

		//IMPRIME INDIVIDUO, FITNESS E COMPLEXIDADE
		for(i = 0; i < 10; i++) {
			n = 1;

			printf("\n");

			printf("\n\t%.2d - FITNESS = %3ld, e COMPRIMENTO = %3d\n", i, rob[i].fitness, length(rob[i].root));
		}

		printf("\n");

		fprintf(file_pointer, "%d\t%3.4f  \t%4d\n", generation, generation_average, generation_maximum);
		fflush(file_pointer);

		//******************
		//* SALVA CAMINHOS *
		//****************** 
		int number_of_track_files = countExistingFiles("paths/caminho", ".gif");
		printf("Found %d track files", number_of_track_files);

		printf("\n\t\tSalvando, convertendo e apagando caminhos...\n");
		fflush(stdout);
		{
			std::ostringstream filename_ss, command_ss;
			filename_ss << "paths/caminho" << std::setfill('0') << std::setw(3) << number_of_track_files << ".ppm";
			std::string filename = filename_ss.str();

			if ((track_files_pointer = fopen(filename.c_str(), "w+")) == NULL) {
				printf("\n\n\tArquivo nao pode ser criado.\n\n");
				exit(-1);
			}

			//SALVA UMA MATRIZ PARA UM ARQUIVO TIPO PPM
			fprintf(track_files_pointer, "P3 \n 200 200 \n 255 \n");  //GRAVA COMECO

			for (lin = 0; lin < HEIGHT; lin++)     //GRAVA NO ARQUIVO
			{
				for (col = 0; col < WIDTH; col++)
					fprintf(track_files_pointer, "%d %d %d   ", best_track[lin][col][0], best_track[lin][col][1], best_track[lin][col][2]);

				fprintf(track_files_pointer, "\n");
			}

			fflush(track_files_pointer);
			fclose(track_files_pointer);  //FECHA ARQUIVO

			command_ss << "magick " << filename << " paths/caminho" 
						<< std::setfill('0') << std::setw(3) 
						<< number_of_track_files << ".gif && rm " << filename;
			std::string command = command_ss.str();

			system(command.c_str());

			number_of_track_files++;
		}

		printf("\n******************************************************************************\n");
	}

	run_average /= (GENS - 1);

	printf("\n\tMEDIA GERAL = %3.2f, MAIOR GERAL = %4d\n", run_average, run_maximum);

	fflush(file_pointer);
	fclose(file_pointer);

	//********************
	//* SALVA INDIVIDUOS *
	//********************
	printf("\n\t\tSalvando individuos...\n");
	{
		// setting the index to the first available file number
		int index = countExistingFiles("robots/rb", "tr.txt");
	
		for(int robot_index = 0; robot_index < 100; robot_index++, index++) {
			// rotate after 999
			if(index >= 999)
				index = 0;

			std::ostringstream path_stream;
			path_stream << "robots/rb" << std::setfill('0') << std::setw(3) << index << "tr.txt";
			std::string robot_path = path_stream.str();

			if((file_pointer = fopen(robot_path.c_str(), "w+")) != NULL) {
				save(rob[robot_index].root);

				n = 1;
				fprintf(file_pointer, "\n\nLENGTH = %d\n\nFITNESS = %ld\n",
				length(rob[robot_index].root), rob[robot_index].fitness);
				fclose(file_pointer);
			}
        }
    }

	//******************
	//* LIBERA MEMORIA *
	//******************
	printf("\n\n\t\tLiberando memoria...\n");
	for (i = 0; i < POPULATION; i++)
		freemem(rob[i].root);

	//HORA DO FIM
	run_end_time = time(NULL);

	{
		double timespent;
		int hours;
		int mins;
		int secs;

		timespent = difftime(run_end_time, run_start_time);

		hours = (int)timespent / 3600;

		timespent -= 3600 * hours;

		mins = (int)timespent / 60;

		timespent -= 60 * mins;

		secs = (int)timespent;

		printf("\n\n\tTempo Gasto %.2d:%.2d:%.2d\n\n", hours, mins, secs);

		printf("\n\n\t\"FIM\"\n\n\a");
	}

	//SINAL DE SAIDA
	return(0);
}
