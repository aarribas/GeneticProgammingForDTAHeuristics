package com.aarribas.evodta.ecj;

import static java.util.Arrays.asList;

import java.util.Collection;
import java.util.Scanner;

import org.jppf.task.storage.DataProvider;

import rinde.ecj.GPBaseNode;
import rinde.ecj.GPEvaluator;
import rinde.ecj.GPProgram;
import rinde.ecj.GPProgramParser;

import com.aarribas.dtasim.TrafficSimulator;
import com.aarribas.evodta.TrafficSwappingHeuristicGP;
import com.aarribas.evodta.TrafficSwappingHeuristicGP.GPStatus;
import com.aarribas.evodta.ecj.EvoDTAEvaluator.EvoDTAContext;
import com.aarribas.evodta.ecj.EvoDTAEvaluator.EvoDTATask;
import rinde.jppf.ComputationTask;
import ec.EvolutionState;
import ec.gp.GPTree;


public class EvoDTAEvaluator extends GPEvaluator<EvoDTATask, EvoDTAResult, GPProgram<EvoDTAContext>> {

	@Override
	protected Collection<EvoDTATask> createComputationJobs(DataProvider dataProvider, GPTree[] trees,
			EvolutionState state) {
		final GPProgram<EvoDTAContext> prog = GPProgramParser
				.convertToGPProgram((GPBaseNode<EvoDTAContext>) trees[0].child);
		return asList(new EvoDTATask(prog));
	}

	@Override
	protected int expectedNumberOfResultsPerGPIndividual(EvolutionState state) {
		return 1;
	}

	public static class EvoDTATask extends ComputationTask<EvoDTAResult, GPProgram<EvoDTAContext>> {
	
		private float fitness;

		public EvoDTATask(GPProgram<EvoDTAContext> p) {
			super(p);
			fitness = 0;
		}

		public void run() {

			//change this to avoid leaks
			TrafficSimulator sim;
			TrafficSwappingHeuristicGP heuristic;
			
			//first test
			sim  = new TrafficSimulator("/Users/andresaan/Documents/MAI/Thesis/matlab/Exercise Final/toy_parfix.mat", 1.2, 0.004, 50, TrafficSimulator.VERBOSITY.SILENT);
			heuristic = new TrafficSwappingHeuristicGP();
			heuristic.setupGenParams(sim, this);
			sim.runDTA(30, heuristic);
			float fitness1 = computeFitness(heuristic, sim); 
			
		//	sim.displayRouteFractionPerRouteInterval();
		//	sim.displayRouteTravelTimesPerRoute();
		


//			//2d test
//			if(heuristic.getGpStatus() != GPStatus.GP_STATUS_ABORTED){
//				sim  = new TrafficSimulator("/Users/andresaan/Documents/MAI/Thesis/matlab/Exercise Final/toy_par.mat", 3, 0.004);
//				heuristic = new TrafficSwappingHeuristicGP();
//				heuristic.setupGenParams(this);
//				sim.runDTA(1, heuristic);
//				updateFitness(2); 
//			}
//
//			//3rd test
//			if(heuristic.getGpStatus() != GPStatus.GP_STATUS_ABORTED){
//				sim  = new TrafficSimulator("/Users/andresaan/Documents/MAI/Thesis/matlab/Exercise Final/toy_par.mat", 3, 0.004);
//				heuristic = new TrafficSwappingHeuristicGP();
//				heuristic.setupGenParams(this);
//				sim.runDTA(1, heuristic);
//				updateFitness(3); 
//			}
			
			//temporal fitness
			fitness = fitness1;
			setResult(new EvoDTAResult(fitness, taskData.getId()));
		}

		public float computeFitness(TrafficSwappingHeuristicGP heuristic, TrafficSimulator sim ){
			
			//if aborted set the max fitness
			if(heuristic.getGpStatus() != GPStatus.GP_STATUS_NORMAL){
				
				return Float.MAX_VALUE;
			} 
			else{
				if (Double.isInfinite(sim.getGap()) || Double.isNaN(sim.getGap())) {
					return Float.MAX_VALUE;
				}
				else{
					
					
					//if the progression was negative assign max fitness
					if(sim.getGap() > heuristic.getFirstGap()){
						return Float.MAX_VALUE;
					}
					
//					System.out.println(heuristic.getFirstGap() + " " + sim.getIteration() + " " +  sim.getGap());
					
					//otherwise compute the fitness as the difference between the given progressino and best possible progression.		
					return (float) (sim.getIteration() / (heuristic.getFirstGap() - sim.getGap())  - (3.0/heuristic.getFirstGap()));
				}
			}

		}
	}

	public static class EvoDTAContext {
		public final double cDemand;
		public final double oDemand;
		public final double invIteration;
		public final double normCostDiff;
		public final double cumuDelta;

		public EvoDTAContext(double cDemand,
				double oDemand,
				double invIteration,
				double normCostDiff,
				double cumuDelta) {
			this.cDemand = cDemand;
			this.oDemand = oDemand;
			this.invIteration = invIteration;
			this.normCostDiff = normCostDiff;
			this.cumuDelta = cumuDelta;
		}
	}
}
