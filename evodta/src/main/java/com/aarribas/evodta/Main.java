package com.aarribas.evodta;

import ec.Evolve;

public class Main {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		//arguments meant to load the right params file, to the population every 50 iterations and to print
		//in latex frendly format the best individual at the end.
		Evolve.main(new String[] { "-file", "files/ec/simple-example.params" });
		

	}

}