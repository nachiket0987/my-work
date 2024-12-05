

import util

class SearchProblem:
  
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
    
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
    
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
 
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
 
  util.raiseNotDefined()

def breadthFirstSearch(problem):

  util.raiseNotDefined()
      
def uniformCostSearch(problem):
 
  util.raiseNotDefined()

def nullHeuristic(state, problem=None):

  return 0

def aStarSearch(problem, heuristic=nullHeuristic):

  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
