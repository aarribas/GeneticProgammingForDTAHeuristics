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
import com.aarribas.evodta.ecj.EvoDTAEvaluator.ExampleContext;
import com.aarribas.evodta.ecj.EvoDTAEvaluator.EvoDTATask;
import rinde.jppf.ComputationTask;
import ec.EvolutionState;
import ec.gp.GPTree;


public class EvoDTAEvaluator extends GPEvaluator<EvoDTATask, DefaultResult, GPProgram<ExampleContext>> {

	@Override
	protected Collection<EvoDTATask> createComputationJobs(DataProvider dataProvider, GPTree[] trees,
			EvolutionState state) {
		final GPProgram<ExampleContext> prog = GPProgramParser
				.convertToGPProgram((GPBaseNode<ExampleContext>) trees[0].child);
		return asList(new EvoDTATask(prog));
	}

	@Override
	protected int expectedNumberOfResultsPerGPIndividual(EvolutionState state) {
		return 1;
	}

	public static class EvoDTATask extends ComputationTask<DefaultResult, GPProgram<ExampleContext>> {

		public EvoDTATask(GPProgram<ExampleContext> p) {
			super(p);
		}

		public void run() {
			
			TrafficSimulator sim  = new TrafficSimulator("/Users/andresaan/Documents/MAI/Thesis/matlab/Exercise Final/toy_par.mat", 3, 0.004);
			
			
			//create a swapping heuristic
			TrafficSwappingHeuristic  heuristic = new TrafficSwappingHeuristicGP();
			
			sim.runDTA(2, heuristic);
			
			double diff = 0;
			for (int x = 0; x < 10; x++) {
				for (int y = 0; y < 10; y++) {
					final double goal = (x * x) + y;
					final double result = taskData.compute(new ExampleContext(x, y));
					diff += Math.abs(goal - result);
				}
			}
			float fitness = (float) diff;
			if (Float.isInfinite(fitness) || Float.isNaN(fitness)) {
				fitness = Float.MAX_VALUE;
			}
			setResult(new DefaultResult(fitness, taskData.getId()));
		}
	}

	public static class ExampleContext {
		public final int x;
		public final int y;

		public ExampleContext(int x, int y) {
			this.x = x;
			this.y = y;
		}
	}
}
