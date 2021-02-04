#!/usr/bin/env python

"Main module of monitoring for agent"

import json
from time import sleep
import argparse
from itertools import count
from typing import Set
from tools.objects import Agent, Graph, Vertex
from tools import graph_generation
from tools.setup_loggers import logger
from tools import ros_comms

AGENTS_INITIAL_POSITIONS = {
    0: 0,
    1: 20,
    2: 24
}
AGENTS_COUNT = 3


def fill_agent_values(cur_agent: Agent) -> None:
    "Filing values of vertexes in agent personal graph."
    cur_agent.graph = cur_agent.global_graph.deepcopy()

    cur_vertex = cur_agent.graph[cur_agent.cur_position.id_]

    cur_vertex.value = 0
    need_to_look = [cur_vertex]

    while need_to_look:
        cur_vertex = need_to_look.pop(0)
        for neighbor in cur_vertex.adjacent:
            if neighbor.value == -1:
                neighbor.value = cur_vertex.value + 1
                need_to_look.append(neighbor)


def fill_global_values(cur_agent: Agent) -> None:
    "Filing values of vertexes in global graph"
    global_graph = cur_agent.global_graph

    for vertex in global_graph:
        vertex.value = 1

    for agent_id in range(AGENTS_COUNT):
        chanel_history = cur_agent.chanels_history[agent_id]
        while not chanel_history[-1].startswith("Values:"):
            if chanel_history[-1] == "Global graph configuration finished":  # TODO: fix this infinity loop.
                chanel_history.pop()
            sleep(0.1)

        message_data = chanel_history[-1].lstrip("Values:")
        values = json.loads(message_data)

        for vertex_id, value in values:
            global_vertex = global_graph.vertexes[vertex_id]
            global_vertex.value *= value


def split_graph(cur_agent: Agent) -> None:
    "Spliting global graph to subgraphs."

    not_selected_vertexes: Set[Vertex] = set(cur_agent.global_graph)
    could_choose: Set[Vertex] = set()

    cur_agent.graph = Graph()
    cur_vertex = cur_agent.cur_position

    new_vertex = cur_vertex.copy()

    cur_agent.cur_position = new_vertex
    cur_agent.graph.add_vertex(new_vertex)

    could_choose = cur_vertex.adjacent.copy()

    for spliting_step in count():
        if not not_selected_vertexes:
            break

        for agent_id in range(AGENTS_COUNT):
            if agent_id == cur_agent.id_:

                could_choose.intersection_update(not_selected_vertexes)
                if not could_choose:
                    cur_agent.post_to_chanel(f"Spliting step {spliting_step}:-1")
                    continue

                chosen_vertex = max(could_choose, key=lambda vertex: (vertex.value, -(abs(vertex.coords.x - cur_vertex.coords.x) +
                                                                                      abs(vertex.coords.y - cur_vertex.coords.y))))
                new_vertex = chosen_vertex.copy()

                cur_agent.graph.add_vertex(new_vertex)
                not_selected_vertexes.remove(chosen_vertex)

                for neighbor in chosen_vertex.adjacent:
                    if neighbor.id_ in cur_agent.graph:
                        new_vertex.add_neighbor(cur_agent.graph[neighbor.id_])
                    else:
                        could_choose.add(neighbor)

                cur_agent.post_to_chanel(f"Spliting step {spliting_step}:{chosen_vertex.id_}")

            else:
                chanel_history = cur_agent.chanels_history[agent_id]
                logger.debug(chanel_history[-5])
                while not chanel_history[-1].startswith(f"Spliting step {spliting_step}:"):
                    sleep(0.1)

                message_data = chanel_history[-1].lstrip(f"Spliting step {spliting_step}:")
                chosen_vertex_id = int(message_data)

                if chosen_vertex_id != -1:
                    not_selected_vertexes.remove(cur_agent.global_graph[chosen_vertex_id])


def calculate_arcs(cur_vertex: Vertex, not_checked: Set[Vertex]) -> None:
    "Going through vertexes and calculating weights of arcs."

    def calculate_arc(from_vertex: Vertex, to_vertex: Vertex) -> float:
        path.add(to_vertex)

        for neighbor in to_vertex.adjacent:
            if (neighbor not in path and
                    neighbor.value >= to_vertex.value and
                    neighbor not in to_vertex.arcs):
                weight = calculate_arc(to_vertex, neighbor)
                to_vertex.add_arc(neighbor, weight)

        path.remove(to_vertex)

        if to_vertex in not_checked:
            weight = to_vertex.value - from_vertex.value + 2 * max([0, *to_vertex.arcs.values()]) - sum(to_vertex.arcs.values()) + 1

        else:
            weight = max([float("-inf"), *to_vertex.arcs.values()]) - 1

        logger.debug(f"Created arc \n"
                     f"\tfrom {from_vertex}\n"
                     f"\tto {to_vertex}\n"
                     f"\tweight {weight}")

        return weight

    path = {cur_vertex}

    for neighbor in cur_vertex.adjacent:
        if neighbor.value >= cur_vertex.value:
            weight = calculate_arc(cur_vertex, neighbor)
            cur_vertex.add_arc(neighbor, weight)


def recalculate_values(cur_vertex: Vertex, not_checked: Set[Vertex]) -> None:
    "Recalculating values of vertexes depending on whether the vertex is checked."
    cur_vertex.value = 0
    recalculated = {cur_vertex}
    need_to_look = [(cur_vertex, 0)]

    while need_to_look:
        cur_vertex, depth = need_to_look.pop(0)
        cur_vertex.arcs = {}
        for neighbor in cur_vertex.adjacent:
            if neighbor in recalculated:
                continue

            if neighbor in not_checked:
                neighbor.value += depth + 1
            else:
                neighbor.value = depth + 1

            need_to_look.append((neighbor, depth + 1))
            recalculated.add(neighbor)
            logger.debug(f"Recalculated value of {neighbor}")


def walk_graph(agent: Agent) -> None:
    """
    Walk prioritized through all vertexes in agent's graph.
    Agent always move to not checked vertex with maximum value.
    """
    not_checked: Set[Vertex] = set(agent.graph)
    not_checked.remove(agent.cur_position)

    while not_checked:
        calculate_arcs(agent.cur_position, not_checked)

        while agent.cur_position.arcs:
            new_position = max(agent.cur_position.arcs.items(), key=lambda arc: arc[1])[0]
            if new_position in not_checked:
                not_checked.remove(new_position)
            agent.move_to(new_position)

        recalculate_values(agent.cur_position, not_checked)


def main(cur_agent_id: int) -> None:
    "Main function"
    global_graph = graph_generation.initialize_graph("field vertexes.json")

    cur_agent = Agent(id_=cur_agent_id,
                      cur_position=global_graph[AGENTS_INITIAL_POSITIONS[cur_agent_id]],
                      global_graph=global_graph,
                      publisher_obj=ros_comms.initialize_publisher(cur_agent_id),
                      chanels_history={agent_id: ros_comms.initialize_publisher_messages_history(agent_id)
                                       for agent_id in range(AGENTS_COUNT)})

    cur_agent.post_to_chanel("Ready")

    for agent_id in range(AGENTS_COUNT):
        chanel_history = cur_agent.chanels_history[agent_id]
        while not chanel_history:
            sleep(0.1)

    fill_agent_values(cur_agent)

    message_data = list(map(lambda vertex: (vertex.id_, vertex.value),
                            cur_agent.graph.vertexes.values()))
    cur_agent.post_to_chanel("Values:" + json.dumps(message_data))

    fill_global_values(cur_agent)

    # Each agent post "ok" to it's chanel after finishing
    # configuration global_graph
    cur_agent.post_to_chanel("Global graph configuration finished")

    for agent_id in range(AGENTS_COUNT):
        chanel_history = cur_agent.chanels_history[agent_id]
        while chanel_history[-1] != "Global graph configuration finished":
            sleep(0.1)

    split_graph(cur_agent)

    walk_graph(cur_agent)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start agent program.")
    parser.add_argument("agent_id", type=int, help="ID of agent.")

    args = parser.parse_args()

    main(args.agent_id)
