package com.aarribas.evodta;

import com.aarribas.dtasim.*;
import com.aarribas.evodta.ecj.EvoDTAEvaluator.EvoDTATask;
import com.aarribas.evodta.ecj.EvoDTAEvaluator.ExampleContext;

import java.util.ArrayList;

import com.aarribas.traffictools.PathRepresentation;

public class TrafficSwappingHeuristicGP extends TrafficSwappingHeuristic{

	private ArrayList< ArrayList<PathRepresentation>> newRoutes;
	private ArrayList< ArrayList<Double[]>> newRouteFractions;

	private ArrayList< ArrayList<PathRepresentation>> oldRoutes;
	private ArrayList< ArrayList<Double[]>> oldRouteFractions;

	private double iteration;
	
	private EvoDTATask task;

	public void setup(ArrayList< ArrayList<PathRepresentation>> oldRoutes,
			ArrayList< ArrayList<Double[]>> oldRouteFractions,
			ArrayList< ArrayList<PathRepresentation>> newRoutes,
			ArrayList< ArrayList<Double[]>> newRouteFractions,
			int iteration){
		this.oldRoutes  = oldRoutes;
		this.oldRouteFractions = oldRouteFractions;
		this.newRoutes = newRoutes;
		this.newRouteFractions = newRouteFractions;
		this.iteration = (double) iteration;

	}

	public void run(){

		ArrayList< ArrayList<Double[]>> tempRouteFractions =  new ArrayList< ArrayList<Double[]>>();

		
		//TODO VERIFY OLDROUTES VS NEWROUTES BELOW!!!
		for(int setOfRoutesIndex = 0; setOfRoutesIndex< newRoutes.size(); setOfRoutesIndex++){
			
			ArrayList<Double[]> fractionsForOD = new ArrayList<Double[]>();
			
			for(int fractionsIndex = 0; fractionsIndex < newRouteFractions.get(setOfRoutesIndex).size(); fractionsIndex++ ){

				//temp fractions
				Double[] tempFractions = new Double[newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex).length];

				if(fractionsIndex < oldRouteFractions.get(setOfRoutesIndex).size()){

					for(int fracIndex = 0; fracIndex < newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex).length; fracIndex++ ){

						//msa as oldRouteFrac + 1/k of the difference between new routefracs and old ones
						tempFractions[fracIndex] = oldRouteFractions.get(setOfRoutesIndex).get(fractionsIndex)[fracIndex] 
								+ (1/iteration)*(newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex)[fracIndex] 
										-oldRouteFractions.get(setOfRoutesIndex).get(fractionsIndex)[fracIndex]);

					}

				}
				else{

					for(int fracIndex = 0; fracIndex < newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex).length; fracIndex++ ){

						//msa for 0 old route fractions
						tempFractions[fracIndex] =  (1/iteration)*newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex)[fracIndex];

					}

				
				}
				//update fractions
				fractionsForOD.add(tempFractions);
				
			}
			tempRouteFractions.add(fractionsForOD);
		}
		
		//save finalRoutes and finalRouteFractions
		finalRoutes = cloneRoutes(newRoutes);
		finalRouteFractions = tempRouteFractions;
		int x =1;
		int y =1;
		final double result = task.getTaskData().compute(new ExampleContext(x, y));
		
		
		

	}
	
	public void setupGenParams(EvoDTATask task){
		
		this.task = task;
		
	}
	
	public ArrayList<ArrayList<PathRepresentation>> cloneRoutes(ArrayList<ArrayList<PathRepresentation>> routes){
		ArrayList<ArrayList<PathRepresentation>> clonedRoutes = new ArrayList<ArrayList<PathRepresentation>>();
		
		for(int setOfRoutes =0;  setOfRoutes < routes.size(); setOfRoutes++){
			ArrayList<PathRepresentation> rtForOD = new ArrayList<PathRepresentation>();

			for(int rtIndex = 0 ; rtIndex < routes.get(setOfRoutes).size(); rtIndex++ ){
				//clone the full double array
				PathRepresentation rt = routes.get(setOfRoutes).get(rtIndex).clone();
				rtForOD.add(rt);
			}
			
			clonedRoutes.add(rtForOD);
		}
		
		return clonedRoutes;

	}
}
