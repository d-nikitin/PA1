from p1_support import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush


def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    p_q = []
    heappush(p_q,(0.0,initial_position))
    visted = {}
    cost = {}
    visted[initial_position] = None
    cost[initial_position] = 0
    path = []
    while len(p_q) != 0:
        current = heappop(p_q)
        node = current[1]
        if node == destination:
            while node is not None:
                path.append(node)
                node = visted[node]
            return path
        test = navigation_edges(graph, current[1])
        print(test)
        for next_cell in navigation_edges(graph, current[1]):
            new_cost = current[0] + next_cell[1]
            if next_cell[0] not in cost or new_cost < cost[next_cell[0]]:
                cost[next_cell[0]] = new_cost
                heappush(p_q,(new_cost, next_cell[0]))
                p_q.sort()
                visted[next_cell[0]] = current[1]
    return None

def dijkstras_shortest_path_to_all(initial_position, graph, adj):
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """
    p_q = []
    heappush(p_q,(0.0,initial_position))
    cost = {}
    cost[initial_position] = 0
    while p_q:
        current = heappop(p_q)
        for next_cell in navigation_edges(graph, current[1]):
            new_cost = cost[current[1]] + next_cell[1]
            if next_cell[0] not in cost or new_cost < cost[next_cell[0]]:
                cost[next_cell[0]] = new_cost
                heappush(p_q,(new_cost, next_cell[0]))
                p_q.sort()
    return cost


def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    edges = []
    if cell not in level['walls']:
        bors = [(1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        for cord in bors:
            newloc = (cell[0]+cord[0], cell[1]+cord[1])
            if newloc in level['spaces']:
                if cord is bors[0] or cord is bors[1] or cord is bors[2] or cord is bors[3]:
                    edges.append((newloc, (0.5*level['spaces'].get(cell) + 0.5*level['spaces'].get(newloc))))
                else: # if is diagonal so we gotta do math
                    m = (0.5*sqrt(2)*level['spaces'].get(cell)) + (0.5*sqrt(2)*level['spaces'].get(newloc))
                    edges.append((newloc, m))
    return edges

def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from 
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """
    
    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]
    
    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'test_maze.txt', 'a','d'

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    #cost_to_all_cells(filename, src_waypoint, 'my_costs.csv')
