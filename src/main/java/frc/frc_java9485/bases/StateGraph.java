package frc.frc_java9485.bases;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.function.BooleanSupplier;


public class StateGraph<S extends Enum<S>> {


    public record Transition<S extends Enum<S>>(S from, S to, double costSeconds, BooleanSupplier guard) {

        public boolean isAllowed() {
            return guard == null || guard.getAsBoolean();
        }
    }

    private final Class<S> stateType;
    private final S[] allStates;
    private final List<Transition<S>> transitions = new ArrayList<>();


    private final Map<S, Map<S, List<S>>> staticPaths = new HashMap<>();

    private boolean dirty = true;

    public StateGraph(Class<S> stateType) {
        this.stateType = stateType;
        this.allStates = stateType.getEnumConstants();
    }


    public StateGraph<S> addTransition(S from, S to, double costSeconds) {
        return addTransition(from, to, costSeconds, null);
    }


    public StateGraph<S> addTransition(S from, S to, double costSeconds, BooleanSupplier guard) {
        if (costSeconds < 0.0) {
            throw new IllegalArgumentException("Transition cost must not be negative: " + from + " -> " + to);
        }
        transitions.add(new Transition<>(from, to, costSeconds, guard));
        dirty = true;
        return this;
    }


    public StateGraph<S> addBidirectional(S a, S b, double costSeconds) {
        return addTransition(a, b, costSeconds).addTransition(b, a, costSeconds);
    }


    public StateGraph<S> connectAllThrough(S hub, double costSeconds) {
        for (S state : allStates) {
            if (state != hub) {
                addBidirectional(state, hub, costSeconds);
            }
        }
        return this;
    }


    public S nextStepToward(S from, S goal) {
        if (from == goal) {
            return goal;
        }
        ensureComputed();

        List<S> path = staticPaths.getOrDefault(from, Map.of()).get(goal);
        if (path == null || path.size() < 2) {
            return from;
        }
        return path.get(1);
    }


    public S dynamicNextStepToward(S from, S goal) {
        if (from == goal) {
            return goal;
        }
        List<S> path = shortestPath(from, goal, true);
        return (path == null || path.size() < 2) ? from : path.get(1);
    }


    public List<S> pathBetween(S from, S goal) {
        ensureComputed();
        List<S> path = staticPaths.getOrDefault(from, Map.of()).get(goal);
        return path == null ? null : List.copyOf(path);
    }


    public boolean isReachable(S from, S goal) {
        return from == goal || pathBetween(from, goal) != null;
    }


    public boolean isDirectTransition(S from, S goal) {
        return transitions.stream().anyMatch(t -> t.from() == from && t.to() == goal);
    }

    public List<Transition<S>> getTransitions() {
        return Collections.unmodifiableList(transitions);
    }



    private void ensureComputed() {
        if (!dirty) {
            return;
        }
        staticPaths.clear();
        for (S from : allStates) {
            Map<S, List<S>> fromHere = new EnumMap<>(stateType);
            for (S to : allStates) {
                if (from == to) {
                    continue;
                }
                List<S> path = shortestPath(from, to, false);
                if (path != null) {
                    fromHere.put(to, path);
                }
            }
            staticPaths.put(from, fromHere);
        }
        dirty = false;
    }


    private List<S> shortestPath(S from, S goal, boolean respectGuards) {
        Map<S, Double> best = new EnumMap<>(stateType);
        Map<S, S> previous = new EnumMap<>(stateType);
        PriorityQueue<S> queue = new PriorityQueue<>(
                (a, b) -> Double.compare(best.getOrDefault(a, Double.MAX_VALUE), best.getOrDefault(b, Double.MAX_VALUE)));

        best.put(from, 0.0);
        queue.add(from);

        while (!queue.isEmpty()) {
            S current = queue.poll();
            if (current == goal) {
                break;
            }

            double currentCost = best.getOrDefault(current, Double.MAX_VALUE);
            for (Transition<S> transition : transitions) {
                if (transition.from() != current) {
                    continue;
                }
                if (respectGuards && !transition.isAllowed()) {
                    continue;
                }

                double candidate = currentCost + transition.costSeconds();
                if (candidate < best.getOrDefault(transition.to(), Double.MAX_VALUE)) {
                    best.put(transition.to(), candidate);
                    previous.put(transition.to(), current);

                    queue.remove(transition.to());
                    queue.add(transition.to());
                }
            }
        }

        if (!best.containsKey(goal)) {
            return null;
        }

        List<S> path = new ArrayList<>();
        for (S at = goal; at != null; at = previous.get(at)) {
            path.add(at);
            if (at == from) {
                break;
            }
        }
        Collections.reverse(path);
        return path.get(0) == from ? path : null;
    }
}
