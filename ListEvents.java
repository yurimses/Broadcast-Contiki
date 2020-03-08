
/*     List events is a generator of events
*      Copyright (c) 2019 Marlon W. Santos <user.santos.santos@icen.ufpa.br>
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>5.
*
*/


package org.contikios.cooja.dialogs;

  //Permite salvar em arquivo os outputs do System.out.println
import java.io.*;
import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.Random;

public class ListEvents{

  private int time,count_motes;
  private int id_mote; 
  private double[] coordinateX;
  private double[] coordinateY;
  private double[] coordinateZ;
  private int startX,endX,startY,endY,startZ,endZ;
  private int max = 2; 
  private int min = 1; 
  private int range = max - min + 1;
  
    //Armazena caminho onde os arquivos.h serão salvos
  private String path="/home/yuri/contiki/examples/er-rest-example/";

    //Armazena o número de vezes que os eventos se repetirão
  public void set_time(int time){
    this.time=time;
  }

    //Retorna o número de vezes que os eventos se repetirão
  public int get_time(){
    return time;
  }


    //Armazena o id do mote
  public void set_id_mote(int id_mote){
    this.id_mote=id_mote;
  }
  
    //Retorna o id do mote
  public int get_id_mote(){
    return id_mote;
  }

    //Armazena o número de motes que foram criados
  public void set_count_motes(int count_motes){
    this.count_motes=count_motes;
  }

    //Retorna o número total de motes criados  
  public int get_count_motes(){
    return count_motes;
  }

    //Armazena as coordenadas dos motes
  public void set_coordinates(double coordinateX,double coordinateY,double coordinateZ){

      //Se o id do mote for Zero, cria para cada mote um vetor X, Y e Z
    if(get_id_mote()==0){
      int n = get_count_motes();
      this.coordinateX = new double[n];
      this.coordinateY = new double[n];
      this.coordinateZ = new double[n];
    }

      /*Busca o id do mote e armazena as coordenadas do motes no vetor no mesmo
        index igual ao seu id*/
    int i=get_id_mote();
    this.coordinateX[i]=coordinateX;
    this.coordinateY[i]=coordinateY;
    this.coordinateZ[i]=coordinateZ;
  }

    //Armazena os limites onde os motes serão criados
  public void set_area(int startX,int endX,int startY,int endY,int startZ,int endZ){
    this.startX=startX;
    this.endX=endX;
    this.startY=startY;
    this.endY=endY;
    this.startZ=startZ;
    this.endZ=endZ;
  }

    //Retorna o limite inferior da coordenada X
  public int get_startX(){
    return startX;
  }

    //Retorna o limite superior da coordenada X
  public int get_endX(){
    return endX;
  }

    //Retorna o limite inferior da coordenada Y
  public int get_startY(){
    return startY;
  }

    //Retorna o limite superior da coordenada Y
  public int get_endY(){
    return endY;
  }

    //Retorna o limite inferior da coordenada Z
  public int get_startZ(){
    return startZ;
  }

    //Retorna o limite superior da coordenada Z
  public int get_endZ(){
    return endZ;
  }

    //Salva as coordenadas dos motes em arquivo
  public void save_coordinate(){

      try{
          //Cria o arquivo coordinates.h
        System.setOut(new PrintStream(new FileOutputStream(get_path()+"coordinates.h",false)));

          //Conteúdo que será salvo no arquivo
        System.out.println("int count_motes="+get_count_motes()+";");  
        System.out.println("double motes_coordinates["+get_count_motes()+"][3]={");
          //Busca as coordenadas dos motes para salvar no arquivo
        read_coordinates();
        System.out.println("};");
      }catch(FileNotFoundException ex){System.out.println("Erro ao criar arquivo!");};
  }     

    //Lê as coordenadas dos motes armazenadas nos vetores coordinate
  public void read_coordinates(){
    for (int i=0;i<get_count_motes();i++){  
      if(i<get_count_motes()-1){
          //Busca as coordenadas X,Y e Z de cada um dos motes criados
        System.out.println("{"+get_coordX(i)+","+get_coordY(i)+","+get_coordZ(i)+"},");
      }else{
        System.out.println("{"+get_coordX(i)+","+get_coordY(i)+","+get_coordZ(i)+"}");
      }
    }
  }

    //Retorna a coordenada X dos motes
  public double get_coordX(int i){
    return coordinateX[i];
  }

    //Retorna a coordenada Y dos motes
  public double get_coordY(int i){
    return coordinateY[i];
  }

    //Retorna a coordenada Z dos motes
  public double get_coordZ(int i){
    return coordinateZ[i];
  }

    //Salva as coordenadas dos eventos em arquivo
  public void save_events(){
    try{
        //Cria o arquivo events.h
      System.setOut(new PrintStream(new FileOutputStream(get_path()+"events.h", false)));
          //Conteúdo que será salvo no arquivo
      System.out.println("int total_events="+get_time()+";");
      System.out.println("double events_coordinates["+get_time()+"][3]={");
        //Chama o gerador de eventos aleatórios
      generate_events();
      System.out.println("};\n");

    }catch(FileNotFoundException ex){System.out.println("Erro ao criar arquivo!");}; 
  }

    //Gera os eventos aleatórios
  public void generate_events(){
    
    for(int i=0;i<get_time();i++){
      if(i<get_time()-1){
          //Busca as coordenadas aleatórias X, Y e Z
        System.out.println("{"+random_coordX()+","+random_coordY()+","+random_coordZ()+"},");
      }else{
        System.out.println("{"+random_coordX()+","+random_coordY()+","+random_coordZ()+"}");
      }
    }
  }

    //Gera e retorna uma coordenada X aleatória
  public double random_coordX(){
    return Math.random()*(get_endX()-get_startX())+get_startX();
  }

    //Gera e retorna uma coordenada Y aleatória
  public double random_coordY(){
    return Math.random()*(get_endY()-get_startY())+get_startY();
  }

    //Gera e retorna uma coordenada Z aleatória
  public double random_coordZ(){
    return Math.random()*(get_endZ()-get_startZ())+get_startZ();
  }

    //Retorna caminho onde os arquivos com coordenadas serão salvos
  public String get_path(){
    return path;
  }
   //Salva as prioridades dos eventos
  public void save_priority(){
	    try{
	        	//Cria o arquivo priority_events.h
		      System.setOut(new PrintStream(new FileOutputStream(get_path()+"priority_events.h", false)));
		        //Conteúdo que será salvo no arquivo
		      System.out.println("int total_priorities_events="+get_time()+";");
		      System.out.println("int priority_events["+get_time()+"]={");
		        //Chama o gerador de prioridade de eventos 
			//1 para crítico e e 2 para não crítico
		      generate_priority();
		      System.out.println("};\n");

	       }catch(FileNotFoundException ex){System.out.println("Erro ao criar arquivo!");}; 
  }
   //Gera prioridades para os eventos de forma aleatória
  public void generate_priority() {
	  for(int i=0; i<get_time(); i++) {
		  if(i<get_time()-1) {
			  System.out.println(random_priority()+",");
		  }else {
			  System.out.println(random_priority());
		  }
	  }
  }
   //Retorna uma prioridade (int)
  public int random_priority() {
	  return (int) (Math.random() * range)+min;
  }
}
