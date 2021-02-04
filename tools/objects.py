"""
Module with objects definitions.
"""

from dataclasses import dataclass, field
from typing import Dict, Set, Iterator, Union, List, Any
from tools.setup_loggers import logger as _logger
from tools import ros_comms as _ros_comms

__all__ = ["Coords", "Vertex", "Graph", "Agent"]


@dataclass(frozen=True)
class Coords:
    "Class of coords for different objects."

    x: float
    y: float


@dataclass
class Arc:
    "Class of weighted arc to some vertex."
    to_vertex: "Vertex"
    weight: float


@dataclass(unsafe_hash=True)
class Vertex:
    "Vertex class which contains it's id, coordinates, value and adjaced vertexes."

    id_: int
    coords: Coords
    value: int = field(default=-1, hash=False)
    adjacent: Set["Vertex"] = field(default_factory=set, repr=False, hash=False)
    # TODO: Remake dict to set of Arcs
    arcs: Dict["Vertex", float] = field(default_factory=dict, repr=False, hash=False)

    def copy(self) -> "Vertex":
        "Return copy of the vertex without neighbors and arcs."
        return Vertex(id_=self.id_,
                      coords=self.coords,
                      value=self.value)

    def add_neighbor(self, neighbor: "Vertex") -> None:
        """
        Create adjacent to neighbor vertex.
        """
        self.adjacent.add(neighbor)
        neighbor.adjacent.add(self)

    def add_arc(self, to_vertex: "Vertex", weight: float = 0) -> None:
        """
        Create arc to vertex.
        """
        self.arcs[to_vertex] = weight


@dataclass
class Graph:
    """
    Graph class.

    Support:
    * __setitem__
    * __getitem__
    * __iter__
    * __contains__ by vertex or vertex.id_.
    """

    vertexes: Dict[int, Vertex] = field(default_factory=dict)

    def add_vertex(self, vertex: Vertex) -> None:
        """
        Adding vertex to graph.
        """
        self.vertexes[vertex.id_] = vertex

    def deepcopy(self,
                 with_adjacent: bool = True,
                 with_arcs: bool = True,
                 with_values: bool = True) -> "Graph":
        """
        Create deepcopy of graph.
        If `with_adjacent` is False not creating edges between vertexes.
        If `with_arcs` is False not creating arcs between vertexes.
        If `with_values` is False setting vertex's values to None.
        """

        new_graph = Graph()

        for vertex in self:
            new_vertex = vertex.copy()
            if not with_values:
                new_vertex.value = -1

            new_graph.add_vertex(new_vertex)

        if not with_adjacent and not with_arcs:
            return new_graph

        for vertex in self:
            if with_adjacent:
                for neighbor in vertex.adjacent:
                    from_vertex = new_graph.vertexes[vertex.id_]
                    to_vertex = new_graph.vertexes[neighbor.id_]

                    from_vertex.add_neighbor(to_vertex)

            if with_arcs:
                for neighbor, weight in vertex.arcs.items():
                    from_vertex = new_graph.vertexes[vertex.id_]
                    to_vertex = new_graph.vertexes[neighbor.id_]

                    from_vertex.add_arc(to_vertex, weight)

        return new_graph

    def __setitem__(self, vertex_id: int, value: Vertex) -> None:
        self.vertexes[vertex_id] = value

    def __getitem__(self, vertex_id: int) -> Vertex:
        return self.vertexes[vertex_id]

    def __iter__(self) -> Iterator[Vertex]:
        return (vertex for vertex in self.vertexes.values())

    def __contains__(self, item: Union[int, Vertex]) -> bool:
        if isinstance(item, int):
            return item in self.vertexes

        return item in self.vertexes.values()


@dataclass
class Agent:
    """
    Agent class.

    Each agent contains:
    * id
    * his current position (In which Vertex)
    * link to global field
    * personal graph
    * chanels with a history of messages from other agents
    """

    id_: int
    cur_position: Vertex
    global_graph: Graph
    graph: Graph = field(default_factory=Graph)
    publisher_obj: Any = None
    chanels_history: Dict[int, List[str]] = field(default_factory=dict)

    def move_to(self, vertex: Vertex) -> None:
        "Moving agent to vertex."

        _logger.info(f"Agent {self.id_} moved to {vertex}")
        _ros_comms.move_agent(self.id_,
                              self.cur_position.coords.x,
                              self.cur_position.coords.y,
                              vertex.coords.x,
                              vertex.coords.y)

        self.cur_position = vertex

    def post_to_chanel(self, message: str) -> None:
        "Posting message to agent's chanel."

        _logger.info(f"Agent {self.id_} posted {message}")
        _ros_comms.send_message(self.publisher_obj, message)


del dataclass, field
del Dict, Set, Iterator, Union, List, Any
