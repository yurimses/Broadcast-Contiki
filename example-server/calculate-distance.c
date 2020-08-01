#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "calculate-distance.h"
#include "load-coordinate.h"
//Arquivo com as coordenadas dos motes
//#include "coordinates.h"
//Arquivo com as coordenadas dos eventos
//#include "events.h"

void calculate_difference(void){
	for(i=0;i<3;i++){
		if(event[i]>my_coordinate[i]){
        		diff[i]= event[i]-my_coordinate[i];  
        	}else{
        		diff[i]=my_coordinate[i]-event[i];
        	}
	}	
}

int euclidian_distance(void){
	int distance = 0;
	distance = (int)((sqrt(pow(diff[0],2)+pow(diff[1],2)+pow(diff[2],2))));
	return distance/100;	
}

void print_distance(int distance){
	printf("Distancia euclidiana %d\n", distance);
}



