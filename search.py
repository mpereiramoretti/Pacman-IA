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
import game

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

# funcao que retorna o vetor de acoes necessarias para do no inicial atingir o no final
def getPath(problem, initialNode, finalNode):
    path = []
    currentNode = finalNode
    parentNode = currentNode[3] # no pai
    while (parentNode != None): # no inicial tem no pai None
        action = currentNode[1] # action
        path.append(action)
        # no parente vira o atual e voltamos no caminho
        currentNode = parentNode
        parentNode = currentNode[3]
    path.reverse()
    return path

# funcao geral de busca
def generalSearch(problem, boundaryType=util.Stack, evaluationFct=None):
    boundary = boundaryType()
    initialNode = (problem.getStartState(), None, 0, None)

    # adicionamos o no inicial na fronteira
    if evaluationFct:
        boundary.push(initialNode, evaluationFct(problem, initialNode[0]))
    else:
        boundary.push(initialNode)
    finalNode = None
    visitedStates = []

    while True:
        if (boundary.isEmpty()):
            raise Exception('No path found!')
        # pegamos o proximo no
        node = boundary.pop()
        
        # evitar que no ja visitado seja expandido novamente
        if (node[0] in visitedStates):
            continue

        currentState = node[0]
        visitedStates.append(currentState)
        if (problem.isGoalState(currentState)):
            finalNode = node
            break
        # obtemos os sucessores e adicionamos na fronteira
        successors = problem.getSuccessors(currentState)
        for successor in successors:
            parentNode = node
            # (estado, acao, custo, no pai) 
            successorNode = (successor[0], successor[1], successor[2], parentNode)
            if evaluationFct:
                boundary.push(successorNode, evaluationFct(problem, successor[0]))
            else:
                boundary.push(successorNode)

    # reconstruimos o caminho
    path = getPath(problem, initialNode, finalNode)
    return path

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    """
    # O generalSearch padrão faz um DFS
    return generalSearch(problem)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # BFS usa fila como fronteira
    return generalSearch(problem, boundaryType=util.Queue)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    # avaliacao no UCS eh o custo para chegar no no atual
    def ucsEvaluationFunction(problem, state):
        return problem.costFn(state)

    return generalSearch(problem, boundaryType=util.PriorityQueue, evaluationFct=ucsEvaluationFunction)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    # avaliacao no A* eh o custo para chegar no no atual mais a heuristica para chegar no objetivo
    def astarEvaluationFunction(problem, state):
        return problem.costFn(state) + heuristic(state, problem)
    
    return generalSearch(problem, boundaryType=util.PriorityQueue, evaluationFct=astarEvaluationFunction)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
