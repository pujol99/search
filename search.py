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

class Node:

   
    def __init__(self, state, parent=None, action=None, path_cost=0):
        "Create a search tree Node, derived from a parent by an action."
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1
    

    def amplify(self, problem):
        "List the nodes reachable in one step from this node."
        return [self.child_node(problem, action)
                for action in problem.getSuccessors(self.state)]

    def child_node(self, problem, action):
       
        next = action[0]
        return Node(next, self, action[1], self.path_cost+action[2])

    def result(self):

        return [node.action for node in self.path()[1:]]

    def path(self):

        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def hash(self):
        return hash(self.state)


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

class Node():
    """test"""
    def __init__(self, state, parent, action, cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost

def searchFrontier(frontier, problem):
    # Frontier structure contains tuple: (State, Parent, Action, Cost)
    visited = set()

    # Init frontier
    start_state = problem.getStartState()
    frontier.push(Node(start_state, None, "Stop", 0))

    while not frontier.isEmpty(): 
        node = frontier.pop()
        
        # Check goal
        if problem.isGoalState(node.state):
            path = []
            while node.parent is not None:
                path.append(node.action)
                node = node.parent
            return list(reversed(path))
        
        # Check if its not visited
        if node.state not in visited:
            visited.add(node.state)
        
            # Search for new steps from frontier
            for next_state, action, cost in problem.getSuccessors(node.state):
                if next_state not in visited:
                    frontier.push(Node(next_state, node, action, cost))

    return False

"""
        # Check goal
        if problem.isGoalState(current_step):
            break
        
        # Mark step as visited and search for new steps from frontier
        visited.add(current_step)
        for future_state, dir, _ in problem.getSuccessors(current_step):
            if future_state not in visited:
                frontier.push((current_step, future_state, dir), 
                            problem.getCostOfActions(backtrackSearch(steps_history)))

    return steps_history """

def searchFrontierA(frontier, problem):
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
                            problem.getCostOfActions(backtrackSearchA(steps_history)))

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

def backtrackSearchA(steps_history):
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
    """Search the deepest nodes in the search tree first."""
    
    return searchFrontier(util.Stack(), problem)


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    """
    frontier = util.Queue()
    
    steps_history = searchFrontier(frontier, problem)
    
    return backtrackSearch(steps_history)
"""
    node = Node(problem.getStartState())
    if problem.isGoalState(problem.getStartState()): return node.result()
    frontier = util.Queue()
    frontier.push(node)
    explored = set()
    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node.state): return node.result()
        explored.add(node.state)
        for child in node.amplify(problem):
            if (child.state not in explored) and (child not in frontier.list):
                frontier.push(child)
    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    frontier = util.PriorityQueue()
    
    steps_history = searchFrontierUCS(frontier, problem)
    
    return backtrackSearch(steps_history)


    return searchFrontier(util.Queue(), problem)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    return searchFrontier(util.PriorityQueue(), problem) #aqui ha estat el pol



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):

    node = Node(problem.getStartState())

    if problem.isGoalState(problem.getStartState()): 
        return node.result()
    
    frontier = util.PriorityQueue()
    frontier.update(node, node.path_cost+heuristic(node.state, problem))
    explored = set()
    
    while not frontier.isEmpty():
        node = frontier.pop()
        
        if problem.isGoalState(node.state): 
            return node.result()

        explored.add(node.state)
        
        for child in node.amplify(problem):
            if (child.state not in explored) and (child not in frontier.heap):
                frontier.update(child, child.path_cost+heuristic(child.state, problem))

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
