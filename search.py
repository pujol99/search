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

    def initFrontier(self, frontierType):
        """Init frontier pushing the first node"""
        frontier = frontierType()
        if "Priority" in frontierType.__name__:
            frontier.push(self, 0)
        else:
            frontier.push(self)
        return frontier
    
    def frontierPush(self, frontier, priority):
        """Push this node to the frontier adding priority if necessary"""
        if priority == None:
            frontier.push(self)
        else:
            frontier.push(self, priority)

    def backtrack(self):
        """Keep backtracking node from parents till no parent
            keeping track of actions done"""
        path = []
        node = self
        while node.parent:
            path.append(node.action)
            node = node.parent
        return list(reversed(path))


def searchFrontier(frontierType, problem, priority):
    visited = set()

    # Init frontier from first start state node
    # Node structure contains tuple: (State, Parent, Action, Cost)
    frontier = Node(problem.getStartState(), None, "Stop", 0).initFrontier(
        frontierType)

    while not frontier.isEmpty(): 
        node = frontier.pop()
        
        # Check goal
        if problem.isGoalState(node.state):
            return node.backtrack()
        
        # Check if its not visited
        if node.state not in visited:
            visited.add(node.state)
        
            # Search for new steps from frontier that are not visited
            for next_state, action, cost in problem.getSuccessors(node.state):
                if next_state not in visited:

                    # update cost of node adding parent's cost
                    if node.parent:
                        cost += node.cost

                    # push child to frontier with corresponding priority
                    child = Node(next_state, node, action, cost)
                    child.frontierPush(
                        frontier,
                        priority(next_state, cost))
    # No goal found
    return False

def depthFirstSearch(problem):
    """Search the deepest nodes in the search tree first."""
    # In DFS there is no priority between nodes
    priority = lambda next_state, cost: None
    
    return searchFrontier(util.Stack, problem, priority)


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # In BrFS there is no priority between nodes
    priority = lambda next_state, cost: None
    
    return searchFrontier(util.Queue, problem, priority)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    # In UCS the priority is the cost of reaching the next node
    priority = lambda next_state, cost: cost

    return searchFrontier(util.PriorityQueue, problem, priority)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    # In A* the priority is the cost of reaching the next node
    # plus the heuristic function on the next node
    priority = lambda next_state, cost: cost + heuristic(next_state, problem)

    return searchFrontier(util.PriorityQueue, problem, priority)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
