package com.aarribas.evodta.ecj;

import ec.*;
import ec.util.*;
import java.io.*;
import ec.vector.*;

public class EvoDTAStats extends Statistics{

	public static final String P_POPFILE = "pop-file";
	public int popLog;

	public static final String P_FINALFILE = "final-file";
	public int finalLog;

	public void setup(final EvolutionState state, final Parameter base)
	{
		super.setup(state,base);

		//prepare file to write statistics to.
		File popFile = state.parameters.getFile(
				base.push(P_POPFILE),null);
		if (popFile!=null) try
		{
			popLog = state.output.addLog(popFile,true);
		}
		catch (IOException i)
		{
			state.output.fatal("An IOException occurred while trying to create the log " + 
					popFile + ":\n" + i);
		}


		//prepare file to write the best individual at the end.
		File finalFile = state.parameters.getFile(
				base.push(P_FINALFILE),null);
		if (finalFile!=null) try
		{
			finalLog = state.output.addLog(finalFile,true);
		}
		catch (IOException i)
		{
			state.output.fatal("An IOException occurred while trying to create the log " + 
					finalFile + ":\n" + i);
		}

	}

	@Override
	public void postEvaluationStatistics(final EvolutionState state)
	{
		// be certain to call the hook on super!
		super.postEvaluationStatistics(state);

		//validate number of subpops
		if (state.population.subpops.length > 1) {
			throw new IllegalStateException("More than one subpop is not supported.");
		}

		//we save interesting numbers from which we can compute our statistics per generation
		float bestFitness = 0;
;
		int totalIndividuals = 0;
		float totalSum = 0;
		float totalSumSq2 = 0;

		Individual best_i = null; // quiets compiler complaints
		for (int y = 0; y < state.population.subpops[0].individuals.length; y++) {
			if (best_i == null || state.population.subpops[0].individuals[y].fitness.betterThan(best_i.fitness)) {
				best_i = state.population.subpops[0].individuals[y];
			}

			//the fitness is adjusted fitness, so no infinity or nan are expected.
			totalSum = totalSum + state.population.subpops[0].individuals[y].fitness.fitness();
			totalSumSq2 = (float) (totalSumSq2 + Math.pow(state.population.subpops[0].individuals[y].fitness.fitness(),2));
		}

		totalIndividuals = state.population.subpops[0].individuals.length-1;
		bestFitness = best_i.fitness.fitness();

		String lineToLog =  new String (bestFitness + " " + totalIndividuals + " " + totalSum + " " + totalSumSq2); 

		state.output.println(lineToLog, popLog);

	}

	@Override
	public void finalStatistics(final EvolutionState state, int result){
		
		//TODO: modify to save the whole last generation in case I want to re-use the individuals.
		
		//we call the hook
		super.finalStatistics(state, result);

		//validate number of subpops
		if (state.population.subpops.length > 1) {
			throw new IllegalStateException("More than one subpop is not supported.");
		}

		//save the best individual
		Individual best_i = null; // quiets compiler complaints
		for (int y = 0; y < state.population.subpops[0].individuals.length; y++) {
			if (best_i == null || state.population.subpops[0].individuals[y].fitness.betterThan(best_i.fitness)) {
				best_i = state.population.subpops[0].individuals[y];
			}
		}
		
		best_i.printIndividual(state,finalLog);
	}
}