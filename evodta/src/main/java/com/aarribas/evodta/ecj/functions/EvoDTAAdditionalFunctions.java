package com.aarribas.evodta.ecj.functions;

import java.util.ArrayList;

import rinde.ecj.GPFunc;

import com.aarribas.evodta.ecj.functions.EvoDTAAdditionalFunctions.If2;


public class EvoDTAAdditionalFunctions {

	public static <T> If2<T> newIf2() {
		return new If2<T>();
	}

	public static class If2<T> extends GPFunc<T> {
		private static final long serialVersionUID = -8010536154985809677L;

		public If2() {
			super(2);
		}

		public double execute(double[] input, T context) {
			return 0.0;
		}

	}
}
