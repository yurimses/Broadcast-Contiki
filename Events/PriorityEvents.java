
/*     Priority events is a generator of priority events
*      Copyright (c) 2019 Yuri Melo 
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

public class PriorityEvents{

  //private int max = 2; 
  //private int min = 1; 
  private int max = 100;
  private int min = 1;
  private int range = max - min + 1;
  

   //Salva as prioridades dos eventos
  public void save_priority(ListEvents le){
	    try{
	        	//Cria o arquivo priority_events.h
		      System.setOut(new PrintStream(new FileOutputStream(le.get_path()+"priority_events.h", false)));
		        //Conteúdo que será salvo no arquivo
		      System.out.println("int total_priorities_events="+le.get_time()+";");
		      System.out.println("int priority_events["+le.get_time()+"]={");
		        //Chama o gerador de prioridade de eventos 
			//1 para crítico e e 2 para não crítico
		      generate_priority(le);
		      System.out.println("};\n");

	       }catch(FileNotFoundException ex){System.out.println("Erro ao criar arquivo priority_events.h!");}; 
  }


   //Gera prioridades para os eventos de forma aleatória
  public void generate_priority(ListEvents le) {
	  for(int i=0; i<le.get_time(); i++) {
		  if(i<le.get_time()-1) {
			  System.out.println(random_priority()+",");
		  }else {
			  System.out.println(random_priority());
		  }
	  }
  }


   //Retorna uma prioridade (int)
  public int random_priority() {
	int number = (int) (Math.random() * range);
	if ((number) <= 75) {
		return 2;	
	}else{
		return 1;	
	}	  
	//return (int) (Math.random() * range)+ min;
  }


  public void save_realsim_events(ListEvents le){
	    try{
	        	//Cria o arquivo priority_events.h
		      System.setOut(new PrintStream(new FileOutputStream(le.get_path()+"realsim_events.h", false)));
		        //Conteúdo que será salvo no arquivo
		      for(int i = 1; i<=le.get_count_motes()+1; i++) {		      		
				System.out.println("0;addnode;"+(float) i);
                      }
		      
		      for(int i = 1; i<=le.get_count_motes()+1; i++) {	
				if(i == 1){	      		
					System.out.println("nodetype;sinkNode;"+(float) i);
				}else{
					System.out.println("nodetype;defaultNode;"+(float) i);
				}
                      }
		      
                      for(int i = 1; i<=le.get_count_motes()+1; i++) {	
		      		for(int j = 1; j<=le.get_count_motes()+1; j++) {
					if (i != j){	
		      				System.out.println("0;setedge;"+(float) i+";"+(float) j+";100;-10;105");
					}
                      		}		
                      }
		      

	       }catch(FileNotFoundException ex){System.out.println("Erro ao criar arquivo realsim_events.h!");}; 
  }
}
