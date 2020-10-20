# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def searchFrontier(frontier, problem):
    # Frontier structure contains tuple: (PreviousStep, CurrentStep, Direction of move)
    steps_history = []
    visited = set()

    # Init frontier
    start_state = problem.getStartState()
    frontier.push((start_state, start_state, None)) 

    while not frontier.isEmpty(): 
        previous_step, current_step, dir = frontier.pop()
        
        # Discard initial move and add move to history
        if current_step != start_state: 
            steps_history.append((previous_step, current_step, dir))

        # Check goal
        if problem.isGoalState(current_step):
            break
        
        # Mark step as visited and search for new steps from frontier
        visited.add(current_step)
        for future_state, dir, _ in problem.getSuccessors(current_step):
            if future_state not in visited:
                frontier.push((current_step, future_state, dir))

    return steps_history

def searchFrontierUCS(frontier, problem):
    # Frontier structure contains tuple: (PreviousStep, CurrentStep, Direction of move)
    steps_history = []
    visited = set()

    # Init frontier
    start_state = problem.getStartState()
    frontier.push((start_state, start_state, None), 0) 

    while not frontier.isEmpty(): 
        previous_step, current_step, dir = frontier.pop()
        # Discard initial move and add move to history
        if current_step != start_state: 
            steps_history.append((previous_step, current_step, dir))

        # Check goal
        if problem.isGoalState(current_step):
            break
        
        # Mark step as visited and search for new steps from frontier
        visited.add(current_step)
        for future_state, dir, _ in problem.getSuccessors(current_step):
            if future_state not in visited:
                frontier.push((current_step, future_state, dir), 
                            problem.getCostOfActions(backtrackSearch(steps_history)))

    return steps_history

def backtrackSearch(steps_history):
    final_path = []
    # key_step will help us remember what previous step made us reach the current step
    key_step = None
    for previous_step, step, dir in reversed(steps_history):
        if not key_step:
            key_step = step
        # When the current step is the key step we update the key step
        if key_step == step:
            final_path.append(dir)
            key_step = previous_step
    
    return list(reversed(final_path))

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    frontier = util.Stack()

    steps_history = searchFrontier(frontier, problem)
    
    return backtrackSearch(steps_history)


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    frontier = util.Queue()
    
    steps_history = searchFrontier(frontier, problem)
    
    return backtrackSearch(steps_history)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    
    steps_history = searchFrontierUCS(frontier, problem)
    
    return backtrackSearch(steps_history)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
