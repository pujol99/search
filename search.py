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

class Node():
    def __init__(self, state, parent, action, cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost

def backtrack(node):
    path = []
    while node.parent is not None:
        path.append(node.action)
        node = node.parent
    return list(reversed(path))

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
            return backtrack(node)
        
        # Check if its not visited
        if node.state not in visited:
            visited.add(node.state)
        
            # Search for new steps from frontier
            for next_state, action, cost in problem.getSuccessors(node.state):
                if next_state not in visited:
                    frontier.push(Node(next_state, node, action, cost))
    return False


def searchFrontierPriority(frontier, problem):
    # Frontier structure contains tuple: (State, Parent, Action, Cost)
    visited = set()
    # Init frontier
    start_state = problem.getStartState()
    frontier.push(Node(start_state, None, "Stop", 0), 0)
    while not frontier.isEmpty(): 
        node = frontier.pop()
        
        # Check goal
        if problem.isGoalState(node.state):
            return backtrack(node)
        
        # Check if its not visited
        if node.state not in visited:
            visited.add(node.state)
        
            # Search for new steps from frontier
            for next_state, action, cost in problem.getSuccessors(node.state):
                if next_state not in visited:
                    if node.parent:
                        cost += node.cost
                    frontier.push(Node(next_state, node, action, cost), cost)
    return False

def searchFrontierA(frontier, problem, heuristic):
    # Frontier structure contains tuple: (State, Parent, Action, Cost)
    visited = set()
    # Init frontier
    start_state = problem.getStartState()
    frontier.push(Node(start_state, None, "Stop", 0), 0)
    while not frontier.isEmpty(): 
        node = frontier.pop()
        
        # Check goal
        if problem.isGoalState(node.state):
            return backtrack(node)
        
        # Check if its not visited
        if node.state not in visited:
            visited.add(node.state)
        
            # Search for new steps from frontier
            for next_state, action, cost in problem.getSuccessors(node.state):
                if next_state not in visited:
                    if node.parent:
                        cost += node.cost
                    new_cost = cost + heuristic(next_state, problem)
                    frontier.push(Node(next_state, node, action, cost), new_cost)
    return False

def depthFirstSearch(problem):
    """Search the deepest nodes in the search tree first."""
    
    return searchFrontier(util.Stack(), problem)


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    
    return searchFrontier(util.Queue(), problem)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""


    return searchFrontierPriority(util.PriorityQueue(), problem)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):

    return searchFrontierA(util.PriorityQueue(), problem, heuristic)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
