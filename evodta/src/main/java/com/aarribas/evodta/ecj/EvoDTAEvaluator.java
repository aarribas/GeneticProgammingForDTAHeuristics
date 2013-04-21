package com.aarribas.evodta.ecj;

import static java.util.Arrays.asList;

import java.util.Collection;

import org.jppf.task.storage.DataProvider;

import rinde.ecj.DefaultResult;
import rinde.ecj.GPBaseNode;
import rinde.ecj.GPEvaluator;
import rinde.ecj.GPProgram;
import rinde.ecj.GPProgramParser;

import com.aarribas.dtasim.TrafficSimulator;
import com.aarribas.dtasim.TrafficSwappingHeuristic;
import com.aarribas.dtasim.TrafficSwappingHeuristicMSA;
import com.aarribas.evodta.TrafficSwappingHeuristicGP;
import com.aarribas.evodta.ecj.EvoDTAEvaluator.EvoDTAContext;
import com.aarribas.evodta.ecj.EvoDTAEvaluator.EvoDTATask;
import rinde.jppf.ComputationTask;
import ec.EvolutionState;
import ec.gp.GPTree;


public class EvoDTAEvaluator extends GPEvaluator<EvoDTATask, DefaultResult, GPProgram<EvoDTAContext>> {

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

	public static class EvoDTATask extends ComputationTask<DefaultResult, GPProgram<EvoDTAContext>> {

		public EvoDTATask(GPProgram<EvoDTAContext> p) {
			super(p);
		}

		public void run() {
			
			TrafficSimulator sim  = new TrafficSimulator("/Users/andresaan/Documents/MAI/Thesis/matlab/Exercise Final/toy_par.mat", 3, 0.004);
			
			
			//create a swapping heuristic
			TrafficSwappingHeuristicGP  heuristic = new TrafficSwappingHeuristicGP();
			heuristic.setupGenParams(this);
			sim.runDTA(1, heuristic);
			
//			double diff = 0;
//			for (int x = 0; x < 10; x++) {
//				for (int y = 0; y < 10; y++) {
//					final double goal = (x * x) + y;
//					final double result = taskData.compute(new ExampleContext(x, y));
//					diff += Math.abs(goal - result);
//				}
//			}
//			float fitness = (float) diff;
//			if (Float.isInfinite(fitness) || Float.isNaN(fitness)) {
//				fitness = Float.MAX_VALUE;
//			}
//			setResult(new DefaultResult(fitness, taskData.getId()));
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
