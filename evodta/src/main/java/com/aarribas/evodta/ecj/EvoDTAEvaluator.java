package com.aarribas.evodta.ecj;

import static java.util.Arrays.asList;

import java.util.Collection;

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
		private TrafficSimulator sim;
		TrafficSwappingHeuristicGP heuristic;
		private float fitness;

		public EvoDTATask(GPProgram<EvoDTAContext> p) {
			super(p);
			fitness = 0;
		}

		public void run() {

			//first test
			sim  = new TrafficSimulator("/Users/andresaan/Documents/MAI/Thesis/matlab/Exercise Final/toy_par.mat", 3, 0.004);
			heuristic = new TrafficSwappingHeuristicGP();
			heuristic.setupGenParams(this);
			sim.runDTA(1, heuristic);
			updateFitness(1); 

			//2d test
			if(heuristic.getGpStatus() != GPStatus.GP_STATUS_ABORTED){
				sim  = new TrafficSimulator("/Users/andresaan/Documents/MAI/Thesis/matlab/Exercise Final/toy_par.mat", 3, 0.004);
				heuristic = new TrafficSwappingHeuristicGP();
				heuristic.setupGenParams(this);
				sim.runDTA(1, heuristic);
				updateFitness(2); 
			}

			//3rd test
			if(heuristic.getGpStatus() != GPStatus.GP_STATUS_ABORTED){
				sim  = new TrafficSimulator("/Users/andresaan/Documents/MAI/Thesis/matlab/Exercise Final/toy_par.mat", 3, 0.004);
				heuristic = new TrafficSwappingHeuristicGP();
				heuristic.setupGenParams(this);
				sim.runDTA(1, heuristic);
				updateFitness(3); 
			}
			setResult(new EvoDTAResult(fitness, taskData.getId()));
		}

		public boolean updateFitness(int testNumber){
			
			//if aborted set the max fitness minus a correction depending on the error
			if(heuristic.getGpStatus() == GPStatus.GP_STATUS_ABORTED){
				
				fitness = (float) (Float.MAX_VALUE - 1.0/heuristic.getError());
				return false;

			} //otherwise the fitness is the iteration + the gap (max iteration might have being attained prior to convergence)
			else{
				if (Double.isInfinite(sim.getGap()) || Double.isNaN(sim.getGap())) {
					fitness =  Float.MAX_VALUE;
					return false;
				}
				else{
					switch(testNumber){
					case 1: fitness = (float) ((double) sim.getIteration() + sim.getGap());break; 
					case 2: 
					case 3: 
					default:
						fitness = fitness >  (float) ((double) sim.getIteration() + sim.getGap()) ? fitness :  (float) ((double) sim.getIteration() + sim.getGap()); break;
					}
					return true;
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
