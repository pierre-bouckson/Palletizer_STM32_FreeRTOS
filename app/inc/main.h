/*
 * main.h
 *
 *  Created on: 15 mai 2021
 *      Author: Laurent
 */
#ifndef INC_MAIN_H_
#define INC_MAIN_H_



// Device header
#include "stm32f0xx.h"


// BSP functions
#include "bsp.h"

// FreeRTOS headers
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "delay.h"


/* Global defines */

//Define actuators
#define distribution_carton     1
#define tapis_distrib_carton    1 << 1
#define BEP                     1 << 2
#define porte                   1 << 3
#define poussoir                1   << 4
#define clamp                   1   << 5
#define monter_ascenseur        1   << 6
#define descendre_ascenseur     1   << 8
#define ascenseur_to_limit      1   << 9
#define distrib_palette         1   << 10
#define charger_palette         1   << 11
#define tapis_CVP               1   << 12
#define tourner_carton          1   << 13       //on s'en fout
#define decharger_palettiseur   1   << 14       //on s'en fout
#define charger_palettiseur     1   << 16
#define decharger_palette       1   << 17       //on s'en fout
#define tapis_PVA               1   << 18
#define tapis_DP                1   << 19
#define tapis_fin               1   << 20
#define remover                 1   << 21       //on s'en fout



//Define sensors
#define carton_distribue    1
#define carton_envoye       1 << 1
#define entre_pal           1 << 2
#define porte_ouverte       1 << 3
#define limite_poussoir     1 << 4
#define clamped             1 << 5
#define ascenseur_RDC       1 << 6
//define skip one bit each 7 sensors or actuators
#define ascenseur_etage1    1 << 8
#define ascenseur_etage2    1 << 9
#define sortie_palette      1 << 10
#define limite_porte        1 << 11
#define ascenseur_mouv      1 << 12
#define entree_palette      1 << 13
#define butee_carton        1 << 14

// Global functions
int my_printf	(const char *format, ...);
int my_sprintf	(char *out, const char *format, ...);


#endif /* INC_MAIN_H_ */
