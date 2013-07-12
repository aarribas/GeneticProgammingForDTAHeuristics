package com.aarribas1.evodta.ecj;

import java.io.Serializable;

import rinde.jppf.GPComputationResult;

public class EvoDTAResult implements GPComputationResult, Serializable{

	protected final float fitness;
	protected final String taskDataId;

	public EvoDTAResult(float fit, String id) {
		fitness = fit;
		taskDataId = id;
	}

	public float getFitness() {
		return fitness;
	}

	public String getTaskDataId() {
		return taskDataId;
	}

}