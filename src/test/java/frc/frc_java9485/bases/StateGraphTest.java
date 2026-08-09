package frc.frc_java9485.bases;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

class StateGraphTest {

    private enum State {
        OFF,
        SHOOTING,
        PASSING,
        COLLECTING,
        ISOLATED
    }


    private static StateGraph<State> robotLikeGraph() {
        StateGraph<State> graph = new StateGraph<>(State.class);
        graph.addBidirectional(State.OFF, State.SHOOTING, 0.25);
        graph.addBidirectional(State.OFF, State.PASSING, 0.25);
        graph.addBidirectional(State.OFF, State.COLLECTING, 0.25);
        graph.addBidirectional(State.SHOOTING, State.PASSING, 0.15);
        return graph;
    }

    @Test
    void directTransitionIsTakenInOneStep() {
        StateGraph<State> graph = robotLikeGraph();
        assertEquals(State.SHOOTING, graph.nextStepToward(State.OFF, State.SHOOTING));
    }


    @Test
    void indirectTransitionRoutesThroughTheSafeState() {
        StateGraph<State> graph = robotLikeGraph();

        assertEquals(State.OFF, graph.nextStepToward(State.SHOOTING, State.COLLECTING));
        assertEquals(List.of(State.SHOOTING, State.OFF, State.COLLECTING),
                graph.pathBetween(State.SHOOTING, State.COLLECTING));
    }

    @Test
    void cheaperShortcutIsPreferredOverRoutingThroughTheHub() {
        StateGraph<State> graph = robotLikeGraph();


        assertEquals(State.PASSING, graph.nextStepToward(State.SHOOTING, State.PASSING));
        assertEquals(List.of(State.SHOOTING, State.PASSING), graph.pathBetween(State.SHOOTING, State.PASSING));
    }

    @Test
    void alreadyAtGoalStaysPut() {
        StateGraph<State> graph = robotLikeGraph();
        assertEquals(State.SHOOTING, graph.nextStepToward(State.SHOOTING, State.SHOOTING));
    }


    @Test
    void unreachableGoalDoesNotMove() {
        StateGraph<State> graph = robotLikeGraph();

        assertEquals(State.OFF, graph.nextStepToward(State.OFF, State.ISOLATED));
        assertNull(graph.pathBetween(State.OFF, State.ISOLATED));
        assertFalse(graph.isReachable(State.OFF, State.ISOLATED));
    }

    @Test
    void connectAllThroughWiresEveryStateToTheHub() {
        StateGraph<State> graph = new StateGraph<>(State.class);
        graph.connectAllThrough(State.OFF, 0.25);

        for (State state : State.values()) {
            assertTrue(graph.isReachable(state, State.OFF), state + " should reach OFF");
            assertTrue(graph.isReachable(State.OFF, state), "OFF should reach " + state);
        }
        assertEquals(State.OFF, graph.nextStepToward(State.SHOOTING, State.COLLECTING));
    }

    @Test
    void guardedTransitionIsSkippedWhileBlocked() {
        StateGraph<State> graph = new StateGraph<>(State.class);
        boolean[] allowShortcut = {false};

        graph.addBidirectional(State.OFF, State.SHOOTING, 1.0);
        graph.addBidirectional(State.OFF, State.PASSING, 1.0);
        graph.addTransition(State.SHOOTING, State.PASSING, 0.1, () -> allowShortcut[0]);


        assertEquals(State.OFF, graph.dynamicNextStepToward(State.SHOOTING, State.PASSING));

        allowShortcut[0] = true;
        assertEquals(State.PASSING, graph.dynamicNextStepToward(State.SHOOTING, State.PASSING));
    }

    @Test
    void isDirectTransitionReportsSingleEdges() {
        StateGraph<State> graph = robotLikeGraph();

        assertTrue(graph.isDirectTransition(State.OFF, State.SHOOTING));
        assertFalse(graph.isDirectTransition(State.SHOOTING, State.COLLECTING));
    }

    @Test
    void negativeCostIsRejected() {
        StateGraph<State> graph = new StateGraph<>(State.class);
        assertThrows(
                IllegalArgumentException.class,
                () -> graph.addTransition(State.OFF, State.SHOOTING, -1.0));
    }
}
