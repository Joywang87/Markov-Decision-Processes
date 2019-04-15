package edu.brown.cs.burlap.tutorials.domain.simple;

import burlap.mdp.auxiliary.DomainGenerator;
import burlap.mdp.core.StateTransitionProb;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.UniversalActionType;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.model.statemodel.FullStateModel;
import burlap.shell.visual.VisualExplorer;
import burlap.visualizer.StatePainter;
import burlap.visualizer.StateRenderLayer;
import burlap.visualizer.Visualizer;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

/*
 * Based on http://burlap.cs.brown.edu/tutorials/bd/p3.html 
 */
public class TrafficLightGridWorld implements DomainGenerator{

	// Names of state variables
	public static final String VAR_X = "x";
	public static final String VAR_LIGHT = "light";

	// Names of actions
	public static final String ACTION_REVERSE = "reverse";
	public static final String ACTION_FORWARD = "forward";
	public static final String ACTION_STOP = "stop";

	public static final int GREEN = 0;
	public static final int YELLOW = 1;
	public static final int RED = 2;

	// Map - 1 dimensional
	protected int [][] map = new int[][] {{0,0,0,0,0,0,0,0,0,0,0}};

	// Data and setter for the goal
	protected int goalx = 10;
	public void setGoalLocation(int goalx){
		this.goalx = goalx;
	}

	// Data and setter for the light
	protected int lightx = 6;
	public void setLightLocation(int lightx){
		this.lightx = lightx;
	}

	// State transition model
	protected class TrafficLightStateModel implements FullStateModel{
		// Transition probability for the light to turn green to yellow
		protected double greenToYellowTransitionProb = 0.4; // Low
		// Transition probability for the light to turn yellow to red.
		protected double yellowToRedTransitionProb = 0.8; // High
		// Transition probability for the light to turn red to green.
		protected double redToGreenTransitionProb = 0.8; // High

		public TrafficLightStateModel(double greenToYellowTransitionProb,
									  double yellowToRedTransitionProb,
									  double redToGreenTransitionProb) {
			this.greenToYellowTransitionProb = greenToYellowTransitionProb;
			this.yellowToRedTransitionProb = yellowToRedTransitionProb;
			this.redToGreenTransitionProb = redToGreenTransitionProb;
		}		
	
		// Default constructor. Use the default probabilities.
		public TrafficLightStateModel() {
		}

		// Map from an action direction to an integer
		protected int actionDir(Action a){
			int adir = 0;
			if(a.actionName().equals(ACTION_STOP)){
				adir = 0;
			}
			else if(a.actionName().equals(ACTION_FORWARD)){
				adir = 1;
			}
			else if(a.actionName().equals(ACTION_REVERSE)){
				adir = -1;
			}			
			return adir;
		}

		// Helper function for updating the position. It is deterministic.
		protected int moveResult(int currentX, int direction){
			int newX = currentX + direction;
			//make sure new position is valid
			int length = TrafficLightGridWorld.this.map[0].length;
			if(newX < 0 || newX >= length){
				newX = currentX;
			}
			return newX;
		}

		// Helper function that gives the probability of the light changing
		protected double lightChangeProbability(int currentLight) {
			switch (currentLight) {
				case GREEN: // green
					return greenToYellowTransitionProb;
				case YELLOW: // yellow
					return yellowToRedTransitionProb;
				case RED: // red
					return redToGreenTransitionProb;
			}
			return 0.0; // Invalid state
		}

		// Helper function that gives the next possible state
		protected int nextLightTransition(int currentLight) {
			switch (currentLight) {
				case GREEN: // green
					return YELLOW;
				case YELLOW: // yellow
					return RED;
				case RED: // red
					return GREEN;
			}
			return currentLight;
		}

		// Helper function for updating the traffic light
		protected int lightResult(int currentLight){
			// Sample the traffic light transition with a random roll
			double r = Math.random(); // Number from 0 to 1
			if (r <= lightChangeProbability(currentLight)) {
				return nextLightTransition(currentLight);
			}
			return currentLight;
		}		

		// Function to sample the state transitions
		@Override
		public State sample(State s, Action a) {
			// Get the current state variables
			s = s.copy();
			TrafficLightGridState gs = (TrafficLightGridState)s;
			int currentX = gs.x;
			int currentLight = gs.light_state;

			// Get the current action
			int actionDirection = actionDir(a);

			// Get the new position
			int newX = moveResult(currentX, actionDirection);

			// Update the light state randomly
			int newLight = lightResult(currentLight);

			//set the new position
			gs.x = newX;
			gs.light_state = newLight;

			//return the state we just modified
			return gs;
		}

		// Function to return all the state transitions
		@Override
		public List<StateTransitionProb> stateTransitions(State s, Action a) {
			// Get the current state variables
			TrafficLightGridState gs = (TrafficLightGridState)s;
			int currentX = gs.x;
			int currentLight = gs.light_state;

			// Get the current action
			int actionDirection = actionDir(a);

			// Get the new position
			int newX = moveResult(currentX, actionDirection);

			// Output list. There are 2 possible result states.
			List<StateTransitionProb> transitionProbabilities = new ArrayList<StateTransitionProb>(2);
			
			// State with current traffic light 
			if (lightChangeProbability(currentLight) != 1.0) {
				TrafficLightGridState newState = gs.copy();
				newState.x = newX;
				newState.light_state = currentLight;
				transitionProbabilities.add(new StateTransitionProb(newState, 
											1.0 - lightChangeProbability(currentLight)));
			}
			// State with transitioned traffic light 
			if (lightChangeProbability(currentLight) != 0.0) {
				TrafficLightGridState newState = gs.copy();
				newState.x = newX;
				newState.light_state = nextLightTransition(currentLight);
				transitionProbabilities.add(new StateTransitionProb(newState, 
											lightChangeProbability(currentLight)));
			}
			return transitionProbabilities;
		}
	}

	// Termination function
	public static class ExampleTF implements TerminalFunction {
		int goalX;
		public ExampleTF(int goalX){
			this.goalX = goalX;
		}

		@Override
		public boolean isTerminal(State s) {
			int x = (Integer)s.get(VAR_X);
			return (x == this.goalX);
		}

	}

	// Reward function
	public static class ExampleRF implements RewardFunction {
		double goalReward = 100.0; 
		double redLightReward = -100.0;
		double forwardReward = -1.0;
		double stopReward = -2.0;
		double reverseReward = -3.0;
		int goalX;
		int lightX;

		public ExampleRF(int goalX, int lightX) {
			this.goalX = goalX;
			this.lightX = lightX;
		}

		public ExampleRF(int goalX,
						 int lightX,
						 double goalReward,
						 double redLightReward,
						 double forwardReward,
						 double stopReward,
						 double reverseReward) {
			this.goalX = goalX;
			this.lightX = lightX;
			this.goalReward = goalReward;
			this.redLightReward = redLightReward;
			this.forwardReward = forwardReward;			
			this.stopReward = stopReward;
			this.reverseReward = reverseReward;
		}		

		@Override
		public double reward(State s, Action a, State sprime) {
			int resultX = (Integer)sprime.get(VAR_X);
			int resultLight = (Integer)sprime.get(VAR_LIGHT);

			double actionReward = 0.0;
			if (a.actionName().equals(ACTION_STOP)) {
				actionReward = this.stopReward;
			} else if (a.actionName().equals(ACTION_FORWARD)) { 
				actionReward = this.forwardReward;
			} else if (a.actionName().equals(ACTION_REVERSE)) {
				actionReward = this.reverseReward;
			} 

			if (resultX == this.goalX){
				return this.goalReward + actionReward;
			} else if (resultX == this.lightX && resultLight == RED) {
				return this.redLightReward + actionReward;
			}
			return actionReward;
		}
	}

	// Transition probabilities and setter
	protected double greenToYellowTransitionProb = 0.4; 
	protected double yellowToRedTransitionProb = 0.8;
	protected double redToGreenTransitionProb = 0.8; 
	public void setTransitionProbabilities(
		double greenToYellowTransitionProb,
		double yellowToRedTransitionProb,
		double redToGreenTransitionProb) {
		this.greenToYellowTransitionProb = greenToYellowTransitionProb; 
		this.yellowToRedTransitionProb = yellowToRedTransitionProb; 
		this.redToGreenTransitionProb = redToGreenTransitionProb; 
	}

	// Rewards and setter
	protected double goalReward = 100; 
	protected double redLightReward = -100;
	protected double forwardReward = -1.0; 
	protected double stopReward = -2.0; 
	protected double reverseReward = -3.0; 

	public void setRewards(
		double goalReward,
		double redLightReward,
		double forwardReward,
		double stopReward,
		double reverseReward) {
		this.goalReward = goalReward; 
		this.redLightReward = redLightReward; 
		this.forwardReward = forwardReward; 
		this.stopReward = stopReward; 
		this.reverseReward = reverseReward; 		
	}	

	// Generator
	public SADomain generateDomain() {
		SADomain domain = new SADomain();
		domain.addActionTypes(
				new UniversalActionType(ACTION_REVERSE),
				new UniversalActionType(ACTION_FORWARD),
				new UniversalActionType(ACTION_STOP));

		TrafficLightStateModel smodel = new TrafficLightStateModel(
			this.greenToYellowTransitionProb,
			this.yellowToRedTransitionProb,
			this.redToGreenTransitionProb);
		RewardFunction rf = new ExampleRF(this.goalx,
										  this.lightx,
										  this.goalReward,
										  this.redLightReward,
										  this.forwardReward,
										  this.stopReward,
										  this.reverseReward);
		TerminalFunction tf = new ExampleTF(this.goalx);
		domain.setModel(new FactoredModel(smodel, rf, tf));
		return domain;
	}


	// Visualization
	public class WallPainter implements StatePainter {
		public void paint(Graphics2D g2, State s, float cWidth, float cHeight) {
			g2.setColor(Color.BLACK);

			//set up floats for the width and height of our domain
			float fHeight = TrafficLightGridWorld.this.map.length;
			float fWidth = TrafficLightGridWorld.this.map[0].length;

			// determine the width of a single cell
			// on our canvas such that the whole map can be painted
			float width = cWidth / fWidth;
			float height = cHeight / fHeight;

			int light = (Integer)s.get(VAR_LIGHT);
			// Draw the intersection
			{	
				switch (light) { // Set the color
					case GREEN:
						g2.setColor(Color.GREEN);
						break;
					case YELLOW:
						g2.setColor(Color.YELLOW);
						break;
					case RED:
						g2.setColor(Color.RED);
						break;
				} 

				int i = TrafficLightGridWorld.this.lightx;
				int j = 0;

				//left coordinate of cell on our canvas
				float rx = i*width;

				//top coordinate of cell on our canvas
				//coordinate system adjustment because the java canvas
				//origin is in the top left instead of the bottom right
				float ry = cHeight - height - j*height;

				//paint the rectangle
				g2.fill(new Rectangle2D.Float(rx, ry, width, height));
			}

			// Draw the goal
			{	
				g2.setColor(Color.CYAN);

				int i = TrafficLightGridWorld.this.goalx;
				int j = 0;

				//left coordinate of cell on our canvas
				float rx = i*width;

				//top coordinate of cell on our canvas
				//coordinate system adjustment because the java canvas
				//origin is in the top left instead of the bottom right
				float ry = cHeight - height - j*height;

				//paint the rectangle
				g2.fill(new Rectangle2D.Float(rx, ry, width, height));
			}
		}
	}

	public class AgentPainter implements StatePainter {
		@Override
		public void paint(Graphics2D g2, State s,
								float cWidth, float cHeight) {

			//agent will be filled in gray
			g2.setColor(Color.GRAY);

			//set up floats for the width and height of our domain
			float fHeight = TrafficLightGridWorld.this.map.length;
			float fWidth = TrafficLightGridWorld.this.map[0].length;

			//determine the width of a single cell on our canvas
			//such that the whole map can be painted
			float width = cWidth / fWidth;
			float height = cHeight / fHeight;

			int ax = (Integer)s.get(VAR_X);
			int ay = 0;

			// left coordinate of cell on our canvas
			float rx = ax*width;

			//top coordinate of cell on our canvas
			//coordinate system adjustment because the java canvas
			//origin is in the top left instead of the bottom right
			float ry = cHeight - height - ay*height;

			//paint the rectangle
			g2.fill(new Ellipse2D.Float(rx, ry, width, height));
		}
	}

	public StateRenderLayer getStateRenderLayer(){
		StateRenderLayer rl = new StateRenderLayer();
		rl.addStatePainter(new TrafficLightGridWorld.WallPainter());
		rl.addStatePainter(new TrafficLightGridWorld.AgentPainter());

		return rl;
	}

	public Visualizer getVisualizer(){
		return new Visualizer(this.getStateRenderLayer());
	}

	// Build with `mvn package`
	// Run with `mvn exec:java -Dexec.mainClass="edu.brown.cs.burlap.tutorials.domain.simple.TrafficLightGridWorld" -e`
	public static void main(String [] args){
		TrafficLightGridWorld gen = new TrafficLightGridWorld();

		// Set the light transition probabilities
		double greenToYellowTransitionProb = 0.3; 
		double yellowToRedTransitionProb = 0.9;
		double redToGreenTransitionProb = 0.8; 
		gen.setTransitionProbabilities(
			greenToYellowTransitionProb,
			yellowToRedTransitionProb,
			redToGreenTransitionProb);		

		// Define the rewards
		double goalReward = 20; 
		double redLightReward = -20;
		double forwardReward = -1.0; 
		double stopReward = -2.0;
		double reverseReward = -3.0;
		gen.setRewards(goalReward, redLightReward, forwardReward, stopReward, reverseReward);

		// Define the locations of the goal and the light
		gen.setGoalLocation(10);
		gen.setLightLocation(6);

		// Generate the domain after all the settings are done
		SADomain domain = gen.generateDomain();

		// Define the initial state: location of the agent, and the initial light color
		int initialX = 0;
		int initialLight = GREEN;
		State initialState = new TrafficLightGridState(initialX, initialLight);

		SimulatedEnvironment env = new SimulatedEnvironment(domain, initialState);
		Visualizer v = gen.getVisualizer();
		VisualExplorer exp = new VisualExplorer(domain, env, v);

		exp.addKeyAction("d", ACTION_FORWARD, "");
		exp.addKeyAction("s", ACTION_STOP, "");
		exp.addKeyAction("a", ACTION_REVERSE, "");
		exp.initGUI();
	}
}

