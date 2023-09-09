class Graph:
    def __init__(self, V=None, E=None):
        """
        Inititializes graph with respective vertices and edges
        """
        self.V = set()
        self.adjacent = dict()

        if V is not None:
            for v in V: 
                self.add_vertex(v)
        if E is not None:
            for e in E: 
                self.add_edge(e)
        
    def add_vertex(self, v):
        """
        Adds vertex to self.V
        """
        self.V.add(v)

    def remove_vertex(self, v):
        """
        Inititializes graph with vertices and edges
        """
        if v not in self.V: 
            raise KeyError("Vertex not found in Graph")
        else:
            self.V.remove(v)

    def add_edge(self, u, v, wt):
        """
        Adds edge to self.adjacent
        """
        if u not in self.adjacent:
            self.adjacent[u] = {(v, wt)} 
        else:
            self.adjacent[u].add((v, wt))

    def remove_edge(self, u, v, wt):
        """
        Removes edge from self.adjacent
        """
        if (v, wt) not in self.adjacent[u]: 
            raise KeyError("No neighbor found")
        else:
            self.adjacent[u].remove((v,wt))

        if len(self.adjacent[u]) == 0: 
            self.adjacent.pop(u)

    def nbrs(self, u):
        """
        Set of neighbors, returns iteration of adjacent u values
        """
        if u not in self.adjacent:
            raise KeyError("No neighbors found")
        return iter(self.adjacent[u])
    
    def fewest_flights(self, city):
        """
        Uses BFS Algorithm to find the fewest flights
        """
        tree = {}
        not_visit = [[city, None]]
        num_flights = {} #Initializes values

        for v in self.V:
            num_flights[v] = 0 #Assigns number of flights to 0

        while not_visit:
            child, parent = not_visit.pop(0) #Pops the first city in the queue
            if child not in tree:
                if child != city:
                    num_flights[child] += num_flights[parent] + 1 #If city not added, number of flights will be updated accordingly
                tree[child] = parent
                for neighbor in self.nbrs(child): #Find neighbors and add to queue
                    not_visit.append((neighbor, child))

        return (tree, num_flights)

    def shortest_path(self, city):
        """
        Returns path with lowest weight values
        """
        paths = set()
        for v in self.V:
            if v != city:
                path, dist = (self.shortest_path_two(city, v)) #Calls two city version for extra parameter
                paths.add((tuple(path), dist))

        return paths
    
    def shortest_path_two(self, start, end):
        """
        Uses Dijkstra's algorithim to return the shortest path between two cities
        """
        dist = {v: float('inf') for v in self.V}
        dist[start] = 0
        to_visit = list(self.V) #List of vertices to be visited
        path = {start: []} #Path so far
        
        while to_visit:
            min_dist = float('inf')
            u = None
            for v in to_visit: #Loops through to find shortest distance
                if dist[v] < min_dist:
                    min_dist = dist[v]
                    u = v
            if dist[u] == float('inf'): #Skip if lesser value already reached
                break
            if u == end: #If end is reached, return the path
                return (path[u] + [end], dist[end])
            for (v, wt) in self.adjacent[u]: #Loops through adjacent and updates distances
                new_dist = dist[u] + wt
                if new_dist < dist[v]:
                    dist[v] = new_dist
                    path[v] = path[u] + [u]
                    
            # Remove the visited vertex from the list of vertices to visit
            to_visit.remove(u)
    
        return (None, float('inf')) #If nothing works, return None

    def minimum_salt(self, city):
        """
        Uses Prim's algorithm to connect city to 
        every other city in the graph with the fewest 
        total number of miles.
        """
        if city not in self.V: #If not found, raise error
            raise ValueError("City not found")
        
        visited = {city}
        edges = []
        
        while len(visited) < len(self.V): #Iterates through all vertices
            min_edge = None 
            for v in visited:
                for neighbor, weight in self.adjacent[v]:
                    if neighbor not in visited and (min_edge is None or weight < min_edge[2]):
                        min_edge = (v, neighbor, weight) #Looks for min weight edge from visited to not yet visited vertex
            edges.append(min_edge)
            visited.add(min_edge[1])
        

        # Create a new graph containing only the edges in the minimum spanning tree
        min_salt = Graph()
        for u, v, weight in edges:
            min_salt.add_vertex(u)
            min_salt.add_vertex(v)
            min_salt.add_edge(u, v, weight)
            min_salt.add_edge(v,u,weight)

        return min_salt