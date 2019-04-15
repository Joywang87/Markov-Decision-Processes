package edu.brown.cs.burlap.tutorials.domain.simple;

import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.StateUtilities;
import burlap.mdp.core.state.UnknownKeyException;
import burlap.mdp.core.state.annotations.DeepCopyState;

import java.util.Arrays;
import java.util.List;

import static edu.brown.cs.burlap.tutorials.domain.simple.TrafficLight2DGridWorld.VAR_X;
import static edu.brown.cs.burlap.tutorials.domain.simple.TrafficLight2DGridWorld.VAR_Y;
import static edu.brown.cs.burlap.tutorials.domain.simple.TrafficLight2DGridWorld.VAR_LIGHT0;
import static edu.brown.cs.burlap.tutorials.domain.simple.TrafficLight2DGridWorld.VAR_LIGHT1;

/**
 * @author Huanhuan Wang
 */
@DeepCopyState
public class TrafficLight2DGridState implements MutableState{

	public int x = 0;
	public int y = 0;
	public int light0 = 0;
	public int light1 = 0;

	private final static List<Object> keys = Arrays.<Object>asList(VAR_X, VAR_Y, VAR_LIGHT0, VAR_LIGHT1);

	public TrafficLight2DGridState() {
	}

	public TrafficLight2DGridState(int x, int y) {
		this.x = x;
		this.y = y;
	}

	public TrafficLight2DGridState(int x, int y, int light0, int light1) {
		this.x = x;
		this.y = y;
		this.light0 = light0;
		this.light1 = light1;
	}

	@Override
	public MutableState set(Object variableKey, Object value) {
		if(variableKey.equals(VAR_X)){
			this.x = StateUtilities.stringOrNumber(value).intValue();
		}
		else if(variableKey.equals(VAR_Y)){
			this.y = StateUtilities.stringOrNumber(value).intValue();
		}
		else if(variableKey.equals(VAR_LIGHT0)){
			this.light0 = StateUtilities.stringOrNumber(value).intValue();
		}	
		else if(variableKey.equals(VAR_LIGHT1)){
			this.light1 = StateUtilities.stringOrNumber(value).intValue();
		}		
		else{
			throw new UnknownKeyException(variableKey);
		}
		return this;
	}

	public List<Object> variableKeys() {
		return keys;
	}

	@Override
	public Object get(Object variableKey) {
		if(variableKey.equals(VAR_X)){
			return x;
		}
		else if(variableKey.equals(VAR_Y)){
			return y;
		}
		else if(variableKey.equals(VAR_LIGHT0)){
			return light0;
		}
		else if(variableKey.equals(VAR_LIGHT1)){
			return light1;
		}
		throw new UnknownKeyException(variableKey);
	}

	@Override
	public TrafficLight2DGridState copy() {
		return new TrafficLight2DGridState(x, y, light0, light1);
	}

	@Override
	public String toString() {
		return StateUtilities.stateToString(this);
	}
}
