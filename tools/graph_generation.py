"""
Module that generate graph of some parallelogram like field
and write it as json to file with name `SAVE_TO_FILE_NAME` for further
reading and initializing.

First write info about vertexes. Then write info about edges.


Output file content:
{
    "vertexes": [
        {
            "id": 1,
            "x coord": 0,
            "y coord": 0,
        },
        {
            "id": 2,
            "x coord": 0,
            "y coord": 1,
        },
        {
            "id": 3,
            "x coord": 1,
            "y coord": 0,
        },
    ],
    "edges": [
        {
            "from id": 1,
            "to id": 2,
        },
        {
            "from id": 1,
            "to id": 3,
        },
    ],
}

which initialize next graph:
(1) - (2)
 |
(3)
"""

import argparse
import json
from typing import Tuple, Dict, List, Any
from tools.objects import Graph, Vertex, Coords


def save_graph_to_file(size: Tuple[int, int], file_name: str) -> None:
    """
    Generate graph of size: `size` and save it as json to file named `file_name`.
    """
    data: Dict[str, List[Dict[str, Any]]] = {
        "vertexes": [],
        "edges": [],
    }

    for x in range(size[0]):
        for y in range(size[1]):
            data["vertexes"].append({
                "id": x * size[1] + y,
                "x coord": x,
                "y coord": y,
            })

            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                if 0 <= x + dx < size[0] and 0 <= y + dy < size[1]:
                    data["edges"].append({
                        "from id": x * size[1] + y,
                        "to id": (x + dx) * size[1] + (y + dy),
                    })

    with open(file_name, "w") as file:
        json.dump(data, file, indent=4)


def initialize_graph(file_name: str) -> Graph:
    """
    Initialize graph from file with name `file_name` and retrun it.
    """
    graph = Graph()

    with open(file_name) as file:
        data = json.load(file)

    for vertex_data in data["vertexes"]:
        vertex = Vertex(id_=vertex_data["id"],
                        coords=Coords(x=vertex_data["x coord"], y=vertex_data["y coord"]))
        graph.add_vertex(vertex)

    for edge_data in data["edges"]:
        from_vertex = graph.vertexes[edge_data["from id"]]
        to_vertex = graph.vertexes[edge_data["to id"]]

        from_vertex.add_neighbor(to_vertex)

    return graph


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate graph and save it to file.")
    parser.add_argument("--width", default=5, type=int, help="Field width")
    parser.add_argument("--height", default=5, type=int, help="Field height")
    parser.add_argument("--filename", default="field vertexes.json", help="Name of file where to save field")

    args = parser.parse_args()

    save_graph_to_file((args.width, args.height), args.filename)
