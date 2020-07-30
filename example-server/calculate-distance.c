#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <calc-distance.h>
//Arquivo com as coordenadas dos motes
#include "coordinates.h"
//Arquivo com as coordenadas dos eventos
#include "events.h"

void calculate_difference(void){
	for(i=0;i<3;i++){
		if(event[i]>my_coordinate[i]){
        		difference[i]= event[i]-my_coordinate[i];  
        	}else{
        		difference[i]=my_coordinate[i]-event[i];
        	}
	}	
}

int euclidian_distance(int *difference){
	int distance = 0;
	distance = (int)((sqrt(pow(difference[0],2)+pow(difference[1],2)+pow(difference[2],2))));
	return distance;	
}



