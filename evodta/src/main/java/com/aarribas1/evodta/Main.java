package com.aarribas1.evodta;

import ec.Evolve;

public class Main {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		if(args.length < 2){
			Evolve.main(new String[] { "-file", "files/ec/simple-example.params" });
		}
		else{
			Evolve.main(args);
		}

	}

}