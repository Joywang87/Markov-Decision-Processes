package edu.brown.cs.burlap.tutorials.domain.simple;

import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.StateUtilities;
import burlap.mdp.core.state.UnknownKeyException;
import burlap.mdp.core.state.annotations.DeepCopyState;

import java.util.Arrays;
import java.util.List;

// Import the variable key names
import static edu.brown.cs.burlap.tutorials.domain.simple.TrafficLightGridWorld.VAR_X;
import static edu.brown.cs.burlap.tutorials.domain.simple.TrafficLightGridWorld.VAR_LIGHT;

/*
 * Based on http://burlap.cs.brown.edu/tutorials/bd/p3.html 
 */
@DeepCopyState
public class TrafficLightGridState implements MutableState{
	// The internal state variables
	public int x; // Position of the vehicle
	public int light_state;  // 0 = green, 1 = yellow, 2 = red

	// Everything below is just Java boilerplate code that constructs/sets/gets

	private final static List<Object> keys = Arrays.<Object>asList(VAR_X, VAR_LIGHT);

	public TrafficLightGridState() {
	}

	public TrafficLightGridState(int x, int light_state) {
		this.x = x;
		this.light_state = light_state;
	}

	@Override
	public MutableState set(Object variableKey, Object value) {
		if(variableKey.equals(VAR_X)){
			this.x = StateUtilities.stringOrNumber(value).intValue();
		}
		else if(variableKey.equals(VAR_LIGHT)){
			this.light_state = StateUtilities.stringOrNumber(value).intValue();
		}
		else{
			throw new UnknownKeyException(variableKey);
		}
		return this;
	}

	@Override
	public List<Object> variableKeys() {
		return keys;
	}

	@Override
	public Object get(Object variableKey) {
		if(variableKey.equals(VAR_X)){
			return x;
		}
		else if(variableKey.equals(VAR_LIGHT)){
			return light_state;
		}
		throw new UnknownKeyException(variableKey);
	}

	@Override
	public TrafficLightGridState copy() {
			return new TrafficLightGridState(x, light_state);
	}

	@Override
	public String toString() {
		return StateUtilities.stateToString(this);
	}
}
