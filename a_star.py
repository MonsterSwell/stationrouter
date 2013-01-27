"""An implementation of the A* searching algorithm.

dyoo@hkn.eecs.berkeley.edu

I got so disgusted at my previous attempt at A*, so here I go again.
Hopefully this version will be easier on the eyes.

A* is a search algorithm that's similar to Dijkstra's algorithm: given
a graph, a start node, and a goal, A* will search for the shortest
path toward the goal.

To help it get there faster, we can provide a heuristic that evaluates
how far we are from that goal.  With a good heuristic, finding an
optimal solution takes MUCH less time.

The main function that one would use is aStar().  Take a look at
_testAStar() to see how it's used.


This code is under MIT license.


The MIT License

Copyright (c) 2008 Danny Yoo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

"""

from priorityqueue import PriorityQueue

def aStar(start, goal, neighbor_func, distance_func, heuristic_func):
    """Returns a sequence of nodes that optmizes for the least cost
    from the start node to the goal.

    Let's describe the data that we pass to this function:

    start: the start of the search.
    goal: the goal of the search.
    neighbor_func: a function that, given a state, returns a list of
                   neighboring states.
    distance_func: a function that takes two nodes, and returns
                   the distance between them.
    heuristic_func: a function that takes two nodes, and returns
                    the heuristic distance between them.

    Each state mush be hashable --- each state must support the
    hash() function.
    """
    pqueue = PriorityQueue()
    g_costs = {start : 1}
    parents = {start : start}
    
    pqueue.push(heuristic_func(start, goal), start)
    while not pqueue.isEmpty():
        next_cost, next_node = pqueue.pop()
        g_costs[next_node] = g_costs[parents[next_node]] \
                             + distance_func(next_node, parents[next_node])
        if next_node == goal: break
        children = neighbor_func(next_node)
        for child in children:
            updateChild(goal, distance_func, heuristic_func,
                        child, next_node, parents, g_costs, pqueue)
    return getPathToGoal(start, goal, parents)

def updateChild(goal, distance_func, heuristic_func,
                child, next_node, parents, g_costs, pqueue):
    """Appropriately update the parents, g_costs, and pqueue structures.

    This is a helper function, since aStar() was getting a bit bulky."""
    if g_costs.has_key(child): return
    f = g_costs[next_node] + distance_func(next_node, child) \
        + heuristic_func(child, goal)
    if pqueue.push(f, child):
        parents[child] = next_node

def getPathToGoal(start, goal, parents):
    """Given the hash of parental links, follow back and return the
    chain of ancestors."""
    try:
        results = []
        while goal != start:
            results.append(goal)
            goal = parents[goal]
        results.append(start)
        results.reverse()
        return results
    except KeyError: return []

def _testGetPathToGoal():
    parents = { 1 : 2,
                2 : 3,
                3 : 4,
                4 : 5 }
    goal = 1
    print getPathToGoal(goal, parents)




######################################################################
###  The rest of this is implementation scaffolding and test stuff.
###  Take a look at it to see how to use this implementation.
######################################################################

class _DistanceDictWrapper:
    """_DistanceDictWrapper is a class wrapper over a dictionary of
    (node1,node2) to distances.  It's set up to make a dictionary look
    like a callable function.  Futhermore, it assumes that distance is
    transitive, so that finding __call__(node1, node2) is the same as
    __call__(node2, node1)."""
    def __init__(self, dict):
        self.dict = dict
    def __call__(self, m, n):
        """Return the distance between m and n.

        Since distance is symmetric, we'll try from m->n, or n->m"""
        if m == n: return 0
        if self.dict.has_key((m, n)):
            return self.dict[(m, n)]
        if self.dict.has_key((n, m)):
            return self.dict[(n, m)]
        return None

class _Neighbor:
    """A quicky class that lets us get the neighbors of a graph,
    given a distance function, and a list of all nodes."""
    def __init__(self, nodes, dist_func):
        self.nodes = nodes
        self.dist_func = dist_func

    def __call__(self, node):
        """Return the neighbors of a node."""
        results = []
        for n in self.nodes:
##            if n == node: continue  # superfluous, since zero
##                                    # is a false value
            if self.dist_func(n, node): results.append(n)
        return results
        
def _testAStar():
    """Given the graph:

        3    1
     /-----b----\
    a      |1    d
     \-----c----/
        1     2

    I do a few tests to make sure that A* looks like it works ok.

    Note: as of now, I don't have a heuristic function set up ---
    basically, I'm testing to make sure that it works as Dijkstra's
    algorithm.  If you want to see the heuristic stuff in action, see
    eight_puzzle.py.
    """

    # There might be better suited A* algorithms (and representations for this)
    # http://scriptogr.am/jdp/post/pathfinding-with-python-graphs-and-a-star
    # http://brandon.sternefamily.net/files/astar.txt
    # https://hkn.eecs.berkeley.edu/~dyoo/python/astar/a_star_search_2.py
    # https://gist.github.com/1687840#file_astar.py
    # http://stackoverflow.com/questions/4159331/python-speed-up-an-a-star-pathfinding-algorithm
    # Or we could use the GPS coordinates as grid coordinates
    # http://stackoverflow.com/questions/4159331/python-speed-up-an-a-star-pathfinding-algorithm
    # https://github.com/elemel/python-astar/blob/master/src/astar_demo.py
    # https://www.google.com/fusiontables/embedviz?viz=MAP&q=select+col7+from+1yHx-9xRpcVU5B4oboTfiSSR4y8so6cV1zAALoU0&h=false&lat=52.49216510989638&lng=13.370220703651414&z=17&t=1&l=col7&y=2&tmplt=3

    dist_func = _DistanceDictWrapper({('GG', 'S1NordS') : 100, ('S1NordS', 'S1NordN') : 0,
                                      ('S1NordS', 'U7S') : 20, ('S1NordN', 'U7N') : 20,
                                      ('S1NordN', 'S1NordS'): 20, ('U7N', 'U7N2'): 40,
                                      ('U7S', 'S25') : 50, ('U7N2', 'S25'): 20,
                                      ('U7N2', 'S25'): 20
                                      })
    nodes = ['GG', 'S1NordS', 'S1NordN', 'U7N', 'U7S', 'U7N2', 'S25']
    neighbor_func = _Neighbor(nodes, dist_func)
    null_heuristic = lambda x, y: 0

    print "neighbors of S1NordS", neighbor_func('S1NordS')
    
    print "path from GG to U7S"
    print aStar('GG', 'U7S', neighbor_func, dist_func, null_heuristic)

    print "path from GG to S25"
    print aStar('GG', 'S25', neighbor_func, dist_func, null_heuristic)

if __name__ == '__main__':
    _testAStar()