# Assignment 4

In this assignment, you will complete a series of exercises with graphs.

## Your Task

You are given an implementation of a `Graph` class that provides various functionality related to graphs. It is more or less the same `graph.py` file that was given in the lessons.

Tasks 1 and 2 will ask you to show the output of a depth first traversal and breadth first traversal in a graph.

Tasks 3 and 4 will ask you to add functionality to the `Graph` class by writing algorithms.

## Steps to Complete

Complete the following steps to the best of your ability, and with the following guidelines:

* The use of generative AI tools (e.g., ChatGPT) is not permitted.
* You must only use concepts and programming constructs covered in this class or previous Kibo classes.
* The algorithms you write should be as efficient as possible.

1. Consider the following graph of cities and the distances between them:

    <center>
    <img
      src="/images/dsa2-week7-graph-cities.svg"
      alt="A graph showing cities and the distances between them."
      style="width:500px;"
    />
    </center>

    (Note that this version of the graph may differ slightly from versions seen in the lessons. You should use this version in answering the questions here.)

    In the `Graph` class, there is a function named `df_trav_abuja()`. Modify it so that it returns a list that represents the order in which cities would be visited in a depth-first traversal that starts from Abuja.

    Notes:

    * The function should just have one line: a `return` statement that returns the cities as a list of strings.
    * Since there are nine cities in the graph, the list that is returned should be of length nine.
    * When a city has multiple unvisited neighbors, the algorithm should choose to visit the unvisited neighbor that is closest by distance first. Having a consistent way of choosing the next city makes it so that there is only one correct answer to the question.
    * There are some unit tests that simply check whether the returned list has nine elements and contains only the names of cities that are in the graph. It doesn't check that the order is correct, though that will be checked in the Gradescope tests after submission.

2. In the `Graph` class, there is also a function named `bf_trav_abuja()`. Modify it so that it returns a list that represents the order in which cities would be visited in a breadth-first traversal that starts from Abuja. The same notes from Task 1 applies to this problem as well.

3. Implement the `is_connected(start, end)` method, which should return `True` if the vertex `start` is connected to the vertex `end` through a series of edges, and `False` if they are not connected.

    Notes:

    * You may choose to write any algorithm you wish to implement the method, but it should be as efficient as possible.
    * The parameters `start` and `end` are the IDs of the vertices, which may or may not exist in the graph. If either does not exist, the method should return `False`.
    * Note that a vertex is considered connected to itself.
    * Remember to call `reinit_vertices()` at the start of the method to clear any state from previous algorithms.

4. Implement the `is_bipartite()` method, which should return `True` if the graph is bipartite and false otherwise.

    Notes:

    * You may choose to write any algorithm you wish to implement the method, but it should be as efficient as possible.
    * One approach to this algorithm could be to add a *color* field to each vertex. In a bipartite graph, there would be exactly two colors of vertices, so you could process the graph one vertex at a time, "coloring" vertices as you go. For example, the first vertex you start from could be given the "color" 0, and all of the vertices that are neighbors of the start vertex can be given a "color" 1. You can then work your way through the graph from there, checking to make sure that no "0" vertices are connected to other "0" vertices and no "1" vertices are connected to other "1" vertices.
    * Remember to call `reinit_vertices()` at the start of the method to clear any state from previous algorithms.
    * You should also add code to `reinit()` in the `Vertex` class as needed to reset any state that you add to the `Graph` class. For example, if you add a `color` field, the field should be reset in `reinit_vertices()`.

## Testing

In `test_graph.py`, there are unit tests for the above pieces of functionality. Run the tests to check your code.

When you upload your submission to Gradescope, you will see the results for some tests, but there may be other edge cases we test during grading that you are not able to see when you submit. So be sure to test thoroughly!
