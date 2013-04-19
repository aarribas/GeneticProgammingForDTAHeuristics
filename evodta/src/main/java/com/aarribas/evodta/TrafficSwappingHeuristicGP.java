package com.aarribas.evodta;

import com.aarribas.dtasim.*;
import com.aarribas.evodta.ecj.EvoDTAEvaluator.EvoDTATask;
import com.aarribas.evodta.ecj.EvoDTAEvaluator.ExampleContext;

import java.util.ArrayList;
import java.util.Arrays;

import com.aarribas.traffictools.PathRepresentation;
import com.aarribas.traffictools.TravelTimeManager;

public class TrafficSwappingHeuristicGP extends TrafficSwappingHeuristic{

	private ArrayList< ArrayList<PathRepresentation>> newRoutes;
	private ArrayList< ArrayList<Double[]>> newRouteFractions;

	private ArrayList< ArrayList<PathRepresentation>> oldRoutes;
	private ArrayList< ArrayList<Double[]>> oldRouteFractions;

	private double iteration;

	private EvoDTATask task;

	private  ArrayList<ArrayList<Double[]>> costPerRoute;

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

		//compute costs for all routes
		updateCostsPerRoute();


		ArrayList< ArrayList<Double[]>> tempRouteFractions =  new ArrayList< ArrayList<Double[]>>();

		//compute routeFractions for non optimal routes and normalise routeFractions for optimal routes
		for(int setOfRoutesIndex = 0; setOfRoutesIndex< newRoutes.size(); setOfRoutesIndex++){

			ArrayList<Double[]> fractionsForOD = new ArrayList<Double[]>();
			Double[] cumulativeOfRouteFractions = new Double[(int)(tEnd/tStep)];
			Arrays.fill(cumulativeOfRouteFractions, 0.0);
			
			//first go through the all routes and compute the new routeFractions for the non optimal and initialise for the optimal
			for(int fractionsIndex = 0; fractionsIndex < newRouteFractions.get(setOfRoutesIndex).size(); fractionsIndex++ ){

				//temp fractions
				Double[] tempFractions = new Double[newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex).length];

				//already seen route
				if(fractionsIndex < oldRouteFractions.get(setOfRoutesIndex).size()){

					for(int fracIndex = 0; fracIndex < newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex).length; fracIndex++ ){

						//compute correction only if the route fraction is not optimal
						if(newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex)[fractionsIndex] != 1){
							double delta = 0;

							//set context - as parameters take: current routeFraction?, current cost, cost of optimal route, previous changes, iteration

							//use GP to compute delta

							//here add feasibility checks

							//Add delta to previous changes

							//save the new route fraction
							tempFractions[fracIndex] = oldRouteFractions.get(setOfRoutesIndex).get(fractionsIndex)[fracIndex] - delta;

							//update the cumulativeOfRouteFractions
							cumulativeOfRouteFractions[fracIndex] = cumulativeOfRouteFractions[fracIndex] + tempFractions[fracIndex];
						}
					}

				}
				else{

					//new route
					for(int fracIndex = 0; fracIndex < newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex).length; fracIndex++ ){

						
						
						//
						//CHECK HERE THAT IT IS ALWAYS 1!!!
						// TODO
						
						
						//a new route that is unseen has no correction
						//tempFractions[fracIndex] =  (1/iteration)*newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex)[fracIndex];

					}

				}

				//already save the references to the temporal fractions - note that the tempFractions for optimal routes are uninitialized
				fractionsForOD.add(tempFractions);

				int[] numberOfOptimalRoutes = new int[(int)(tEnd/tStep)];
				Arrays.fill(numberOfOptimalRoutes, 0);

				//compute for this OD pair the number of optimal routes per instant
				for(fractionsIndex = 0; fractionsIndex < newRouteFractions.get(setOfRoutesIndex).size(); fractionsIndex++ ){
					for(int fracIndex = 0; fracIndex < newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex).length; fracIndex++ ){

						if(newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex)[fractionsIndex] == 1.0){
						
							numberOfOptimalRoutes[fracIndex] += 1;
						}
					}

				}

				//finally normalise routeFractions for optimal routes
				for(fractionsIndex = 0; fractionsIndex < newRouteFractions.get(setOfRoutesIndex).size(); fractionsIndex++ ){

					for(int fracIndex = 0; fracIndex < newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex).length; fracIndex++ ){

						//compute correction only if the route fraction is not optimal
						if(newRouteFractions.get(setOfRoutesIndex).get(fractionsIndex)[fractionsIndex] == 1){

							fractionsForOD.get(fractionsIndex)[fracIndex] = (1.0 - cumulativeOfRouteFractions[fracIndex]) / (double)(numberOfOptimalRoutes[fracIndex]);
						}
					}

				}

			}
			//save the routeFractions for this OD pair
			tempRouteFractions.add(fractionsForOD);
		}

		//save finalRoutes and finalRouteFractions
		finalRoutes = cloneRoutes(newRoutes);
		finalRouteFractions = tempRouteFractions;
		int x =1;
		int y =1;
		final double result = task.getTaskData().compute(new ExampleContext(x, y));




	}


	private void updateCostsPerRoute(){

		costPerRoute =  new ArrayList<ArrayList<Double[]>>();

		//for all routes compute and save the cost!
		for(int setOfRoutesIndex = 0; setOfRoutesIndex< oldRoutes.size(); setOfRoutesIndex++){
			for(int routeIndex = 0; routeIndex< oldRoutes.get(setOfRoutesIndex).size(); routeIndex++){


				PathRepresentation path = oldRoutes.get(setOfRoutesIndex).get(routeIndex);
				int startNodeIndex = path.nodeIndexes[0];
				int endNodeIndex = path.nodeIndexes[path.nodeIndexes.length-1];
				int[] linkIndexes = path.linkIndexes;

				for(int timeClick = 0; timeClick < (int)(tEnd/tStep); timeClick++){
					double routeTT = 0;
					for(int linkIndex : linkIndexes){
						double[] cost = new double[linkSpeeds.get(linkIndex).length];

						//compute instantaneous cost
						for(int costIndex = 0; costIndex < linkSpeeds.get(linkIndex).length; costIndex++ ){
							cost[costIndex] = tfData.links.get(linkIndex).length / linkSpeeds.get(linkIndex)[costIndex];
						}
						//compute complete routeTT (route cost or travel time)
						routeTT = routeTT + TravelTimeManager.computeTravelTimeForGivenCost(cost,timeClick*tStep + routeTT, tEnd, tStep);

					}
					//save the route travel time as cost for this route and time
					saveCostForARouteAndTime(setOfRoutesIndex, routeIndex, timeClick, routeTT);
				}

			}
		}

	}

	private void saveCostForARouteAndTime(int setOfRoutesIndex, int routeIndex, int timeClick, double routeTT){

		//initialisation 
		if(costPerRoute == null){
			costPerRoute = new  ArrayList<ArrayList<Double[]>>();
		}

		//required in case of new OD pair
		if(setOfRoutesIndex > costPerRoute.size()){
			costPerRoute.add(new ArrayList<Double[]>());
		}

		//required in case of new route that was not considered previously
		if(routeIndex > costPerRoute.get(setOfRoutesIndex).size()){
			costPerRoute.get(setOfRoutesIndex).add(new Double[(int)(tEnd/tStep)]);
		}

		//save the cost (route travel time)
		costPerRoute.get(setOfRoutesIndex).get(routeIndex)[timeClick] = routeTT;		

	}

	public void setupGenParams(EvoDTATask task){

		this.task = task;

	}

}
