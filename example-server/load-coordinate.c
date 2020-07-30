#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys/node-id.h"
//Arquivo com as coordenadas dos motes
#include "coordinates.h"
//Arquivo com as coordenadas dos eventos
#include "events.h"
//Arquivo com os arrays de coordenadas
#include "load-coordinate.h"

void load_id(void){

	//Mote busca o seu próprio id e subtrai 2 de seu valor   
	my_id=node_id-2;
}

void load_coordinate_mote(void){

	/*Mote busca sua própria coordenada X,Y e Z dentro da matriz de coordenadas
	  no arquivo coordinate.h e armazena elas no vetor*/
	my_coordinate[0]=(unsigned int)(motes_coordinates[my_id][0]*100);
	my_coordinate[1]=(unsigned int)(motes_coordinates[my_id][1]*100);
	my_coordinate[2]=(unsigned int)(motes_coordinates[my_id][2]*100);
} 

void show_coordinate_mote(void){

	//O mote exibe os valores X,Y e Z de sua coordenada
	printf("Coordenada X: %u\n",my_coordinate[0]);
	printf("Coordenada Y: %u\n",my_coordinate[1]);
	printf("Coordenada Z: %u\n",my_coordinate[2]);
}

void load_coordinate_event(int event_count){

	//Se valor do contador de eventos for menor que total de eventos
	if(event_count<total_events){
		/*Mote busca a coordenada X,Y e Z dentro da matriz de eventos
		  no arquivo events.h e armazena elas no vetor*/
		event[0]=(unsigned int)(events_coordinates[event_count][0]*100);
		event[1]=(unsigned int)(events_coordinates[event_count][1]*100);
		event[2]=(unsigned int)(events_coordinates[event_count][2]*100);
	}
}

void show_coordinate_event(void){

		//Mote exibe os valores X,Y e Z do evento
		printf("Coordenada X do evento: %u\n",event[0]);
		printf("Coordenada Y do evento: %u\n",event[1]);
		printf("Coordenada Z do evento: %u\n",event[2]);
}








