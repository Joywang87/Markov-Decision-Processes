package edu.brown.cs.burlap.tutorials;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.QProvider;
import burlap.behavior.valuefunction.ValueFunction;
import edu.brown.cs.burlap.tutorials.domain.simple.TrafficLightGridWorld;
import edu.brown.cs.burlap.tutorials.domain.simple.TrafficLightGridState;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.vardomain.VariableDomain;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.FullModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.model.TransitionProb;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.mdp.singleagent.SADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;

import java.awt.*;
import java.util.List;

/* Based on http://burlap.cs.brown.edu/tutorials/bpl/p2.html 
Build with `mvn package`
Run with `mvn exec:java -Dexec.mainClass="edu.brown.cs.burlap.tutorials.TrafficLightsBehavior" -e`
*/
public class TrafficLightsBehavior {
    TrafficLightGridWorld gwdg;
    SADomain domain;
    // RewardFunction rf;
    TerminalFunction tf;
    StateConditionTest goalCondition;
    State initialState;
    HashableStateFactory hashingFactory;
    SimulatedEnvironment env;

    int goalX = 10;
    int lightX = 6;

    // Default constructor
    public TrafficLightsBehavior() {
        this(1.0, 1.0, 1.0); // Default is cycling lights
    }

    public TrafficLightsBehavior(double greenToYellowTransitionProb,
                                 double yellowToRedTransitionProb,
                                 double redToGreenTransitionProb) {
        gwdg = new TrafficLightGridWorld();
        tf = new TrafficLightGridWorld.ExampleTF(goalX);
        gwdg.setGoalLocation(goalX);
        goalCondition = new TFGoalCondition(tf);
        
        // Settings
        // Set the light transition probabilities
        gwdg.setTransitionProbabilities(
            greenToYellowTransitionProb,
            yellowToRedTransitionProb,
            redToGreenTransitionProb);      

        // Define the rewards
        double goalReward = 20.0; 
        double redLightReward = -20.0;
        double forwardReward = -1.0; 
        double stopReward = -2.0;
        double reverseReward = -3.0;
        gwdg.setRewards(goalReward, redLightReward, forwardReward, stopReward, reverseReward);

        // Define the locations of the goal and the light
        gwdg.setGoalLocation(goalX);
        gwdg.setLightLocation(lightX);
        domain = gwdg.generateDomain();

        // Define the initial state: location of the agent, and the initial light color
        int initialX = 0;
        int initialLight =TrafficLightGridWorld.GREEN;
        initialState = new TrafficLightGridState(initialX, initialLight);        
    
        //
        hashingFactory = new SimpleHashableStateFactory();
        env = new SimulatedEnvironment(domain, initialState);
    }


    public void visualize(String outputPath){
        Visualizer v = gwdg.getVisualizer();
        new EpisodeSequenceVisualizer(v, domain, outputPath);
    }

    public String averageDiscountedValue(Policy policy, int numTrials) {
        final double gamma = 0.99;
        double cumValue = 0.0;
        double worstCase = Double.POSITIVE_INFINITY;
        for (int i = 0; i < numTrials; ++i) {
            Episode e = PolicyUtils.rollout(policy, initialState, domain.getModel());
            final double discountedReturn = e.discountedReturn(gamma);
            cumValue += discountedReturn; 
            worstCase = Math.min(worstCase, discountedReturn);         
        }
        return Double.toString(cumValue / numTrials) + ", " + worstCase;
    }

    public void BFSExample(String outputPath){
            
        DeterministicPlanner planner = new BFS(domain, goalCondition, hashingFactory);
        Policy p = planner.planFromState(initialState);
        PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "bfs");
            
    }       

    public void DFSExample(String outputPath){
        DeterministicPlanner planner = new DFS(domain, goalCondition, hashingFactory);
        Policy p = planner.planFromState(initialState);
        PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "dfs");
    }

    public void AStarExample(String outputPath){
        Heuristic mdistHeuristic = new Heuristic() {
            public double h(State s) {
                int currentX = ((TrafficLightGridState)s).x;
                double mdist = Math.abs(currentX - goalX);
                return mdist;
            }
        };
        DeterministicPlanner planner = new AStar(domain, goalCondition, hashingFactory,
                                                 mdistHeuristic);
        Policy p = planner.planFromState(initialState);
        PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "astar");
    }               

    public void valueIterationExample(String outputPath, final int maxIterations){
        final double maxDelta = 0.001;
        final double gamma = 0.99;
        Planner planner = new ValueIteration(domain, gamma, hashingFactory, maxDelta, maxIterations);
        Policy p = planner.planFromState(initialState);
        Episode e = PolicyUtils.rollout(p, initialState, domain.getModel());
        e.write(outputPath + "vi");
        System.out.println("Value Iteration, " + maxIterations + ", " + averageDiscountedValue(p, 100));
    
        valueFunctionVisualizer((ValueFunction)planner, p);
    }

    public void policyIterationExample(String outputPath, final int maxIterations){
        int maxEvaluationIterations = maxIterations;
        int maxPolicyIterations = maxIterations;
        double maxDelta = 0.001;
        double gamma = 0.99;        
        Planner planner = new PolicyIteration(domain, gamma, hashingFactory, 0.001, maxEvaluationIterations, maxPolicyIterations);
        Policy p = planner.planFromState(initialState);
        PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "pi");
        System.out.println("Policy Iteration, " + maxIterations + ", " + averageDiscountedValue(p, 100));

        valueFunctionVisualizer((ValueFunction)planner, p);
    }    

    public void qLearningExample(String outputPath, final int maxIterations){  
        double gamma = 0.99;
        double qinit = 0.0;
        double learningRate = 0.3;
        QLearning agent = new QLearning(domain, gamma, hashingFactory, qinit, learningRate);

        //run learning for 200 episodes
        for(int i = 0; i < maxIterations; i++){
            Episode e = agent.runLearningEpisode(env);
            if ((i+1) == maxIterations) { // save only the last one
                e.write(outputPath + "ql_" + i);
            }
            env.resetEnvironment();
        }

        Policy p = agent.planFromState(initialState);
        System.out.println("QLearning, " + maxIterations + ", " + averageDiscountedValue(p, 100));
        valueFunctionVisualizer((ValueFunction)agent, p);
    }         

    public void sarsaLearningExample(String outputPath, final int maxIterations){
        final double gamma = 0.99;
        final double qinit = 0.0;
        final double learningRate = 0.3;
        final double lambda = 0.3;
            
        SarsaLam agent = new SarsaLam(domain, gamma, hashingFactory, qinit, learningRate, lambda);

        //run learning for 200 episodes
        for(int i = 0; i < maxIterations; i++){
            Episode e = agent.runLearningEpisode(env);

            if ((i+1) == maxIterations) { // save only the last one
                e.write(outputPath + "sarsa_" + i);
            }

            //reset environment for next learning episode
            env.resetEnvironment();
        }
        Policy p = agent.planFromState(initialState);
        System.out.println("Sarsa, " + maxIterations + ", " + averageDiscountedValue(p, 100));
    }         

    public void valueFunctionVisualizer(ValueFunction valueFunction, Policy p){
        List<State> allStates = StateReachability.getReachableStates(
                initialState, domain, hashingFactory);

        // define color function
        LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
        rb.addNextLandMark(0., Color.RED);
        rb.addNextLandMark(1., Color.BLUE);

        //define a 2D painter of state values, specifying 
        //which variables correspond to the x and y coordinates of the canvas
        StateValuePainter2D svp = new StateValuePainter2D(rb);
        svp.setXYKeys(TrafficLightGridWorld.VAR_X, TrafficLightGridWorld.VAR_LIGHT, 
            new VariableDomain(0, 11), new VariableDomain(0,3), 
            1, 1);

        //create our ValueFunctionVisualizer that paints for all states
        //using the ValueFunction source and the state value painter we defined
        ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(allStates, svp, valueFunction);

        //define a policy painter that uses arrow glyphs for each of the grid world actions
        PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
        spp.setXYKeys(TrafficLightGridWorld.VAR_X, TrafficLightGridWorld.VAR_LIGHT, 
            new VariableDomain(0, 11), new VariableDomain(0,3), 
            1, 1);

        spp.setActionNameGlyphPainter(TrafficLightGridWorld.ACTION_FORWARD, new ArrowActionGlyph(2));
        spp.setActionNameGlyphPainter(TrafficLightGridWorld.ACTION_REVERSE, new ArrowActionGlyph(3));
        spp.setRenderStyle(PolicyGlyphPainter2D.PolicyGlyphRenderStyle.DISTSCALED);

        //add our policy renderer to it
        gui.setSpp(spp);
        gui.setPolicy(p);

        //set the background color for places where states are not rendered to grey
        gui.setBgColor(Color.GRAY);

        //start it
        gui.initGUI();
    }

    public void experimenterAndPlotter(){
        /**
         * Create factories for Q-learning agent and SARSA agent to compare
         */
        LearningAgentFactory qLearningFactory = new LearningAgentFactory() {
            public String getAgentName() {
                return "Q-Learning";
            }

            public LearningAgent generateAgent() {
                double gamma = 0.99;
                double qinit = 0.0;
                double learningRate = 0.01;
                return new QLearning(domain, gamma, hashingFactory, qinit, learningRate);
            }
        };

        LearningAgentFactory sarsaLearningFactory = new LearningAgentFactory() {
            public String getAgentName() {
                return "SARSA";
            }
            public LearningAgent generateAgent() {
                return new SarsaLam(domain, 0.99, hashingFactory, 0.0, 0.5, 0.3);
            }
        };

        final int numTrials = 100;
        final int trialLength = 1000;
        LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter(env, numTrials, trialLength,
                qLearningFactory, sarsaLearningFactory);

        exp.setUpPlottingConfiguration(500, 250, 2, 1000,
                TrialMode.MOST_RECENT_AND_AVERAGE,
                PerformanceMetric.CUMULATIVE_STEPS_PER_EPISODE,
                PerformanceMetric.AVERAGE_EPISODE_REWARD);

        exp.startExperiment();
        exp.writeStepAndEpisodeDataToCSV("expData");
    }    

/*
mvn exec:java -Dexec.mainClass="edu.brown.cs.burlap.tutorials.TrafficLightsBehavior" -Dexec.args="200" -e
*/
    public static void main(String[] args) {
        int maxIterations = 1000;
        if (args.length == 1) {
            maxIterations = Integer.parseInt(args[0]);
        }
        double greenToYellowTransitionProb = 0.3; 
        double yellowToRedTransitionProb = 0.9;
        double redToGreenTransitionProb = 0.8; 
        boolean deterministic = false; // Required for DFS and BFS
        if (deterministic == true) {        
            greenToYellowTransitionProb = 1.0; 
            yellowToRedTransitionProb = 1.0;
            redToGreenTransitionProb = 1.0; 
        }

        TrafficLightsBehavior example = new TrafficLightsBehavior(
            greenToYellowTransitionProb,
            yellowToRedTransitionProb,
            redToGreenTransitionProb);
        String outputPath = "output/"; //directory to record results
        
        if (deterministic == true) { // Algorithms that only work determinstically
            example.BFSExample(outputPath);
            example.DFSExample(outputPath); // DFS is not optimal
            example.AStarExample(outputPath);           
        }
        // Stochastic functions
        example.valueIterationExample(outputPath, maxIterations);            
        example.policyIterationExample(outputPath, maxIterations);            
        example.qLearningExample(outputPath, maxIterations);
        //example.sarsaLearningExample(outputPath, maxIterations);
        
        //run the visualizer
        example.visualize(outputPath);

        //example.experimenterAndPlotter();        
    }
}
                
