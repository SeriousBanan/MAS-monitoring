import json
from typing import Iterable, Dict, Set
from tools.objects import Agent, Graph, Vertex
from tools.graph_generation import initialize_graph
from tools.setup_loggers import logger


def fill_agent_values(agent: Agent) -> None:
    "Filing values of vertexes in agent personal graph."
    agent.graph = agent.global_graph.deepcopy()

    cur_vertex = agent.graph[agent.cur_position.id_]

    cur_vertex.value = 0
    need_to_look = [cur_vertex]

    while need_to_look:
        cur_vertex = need_to_look.pop(0)
        for neighbor in cur_vertex.adjacent:
            if neighbor.value == -1:
                neighbor.value = cur_vertex.value + 1
                need_to_look.append(neighbor)


def fill_global_values(global_graph: Graph, cur_agent: Agent, agents: Iterable[Agent]) -> None:
    "Filing values of vertexes in global graph"
    for vertex in global_graph.vertexes.values():
        vertex.value = 1

    for agent in agents:
        if agent is cur_agent:
            values = map(lambda vetrex: (vetrex.id_, vetrex.value),
                         agent.graph.vertexes.values())
        else:
            values = json.loads(agent.chanel[-1])

        for vertex_id, value in values:
            global_vertex = global_graph.vertexes[vertex_id]
            global_vertex.value *= value


def split_graph(global_graph: Graph, cur_agent: Agent, agents: Dict[int, Agent]) -> None:
    "Spliting global graph to subgraphs."
    # TODO: Rewrite to single agent which waiting others after choosing.
    # TODO: В цикле если не cur_agent ждать сообщения от других и редачить, иначе редачить и отправлять сообщение

    not_selected_vertexes: Set[Vertex] = set(global_graph)
    could_choose: Dict[int, Set[Vertex]] = {}

    for agent in agents.values():
        not_selected_vertexes.remove(agent.cur_position)
        agent.graph = Graph()

        cur_vertex = agent.cur_position
        new_vertex = cur_vertex.copy()

        agent.cur_position = new_vertex
        agent.graph.add_vertex(new_vertex)

        could_choose[agent.id_] = cur_vertex.adjacent.copy()

    while not_selected_vertexes:
        for agent in agents.values():
            could_choose[agent.id_].intersection_update(not_selected_vertexes)
            if not could_choose[agent.id_]:
                agent.post_to_chanel("Done")
                continue

            chosen_vertex = max(could_choose[agent.id_],
                                key=lambda vertex: (vertex.value, -(abs(vertex.coords.x - agent.cur_position.coords.x) +
                                                                    abs(vertex.coords.y - agent.cur_position.coords.y))))

            new_vertex = chosen_vertex.copy()

            agent.graph.add_vertex(new_vertex)
            not_selected_vertexes.remove(chosen_vertex)

            for neighbor in chosen_vertex.adjacent:
                if neighbor.id_ in agent.graph:
                    new_vertex.add_neighbor(agent.graph[neighbor.id_])
                else:
                    could_choose[agent.id_].add(neighbor)

            agent.post_to_chanel(f"{agent.id_} {chosen_vertex.id_}")


def calculate_arcs(cur_vertex: Vertex, not_checked: Set[Vertex]):
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
    not_checked = set(agent.graph)
    not_checked.remove(agent.cur_position)

    while not_checked:
        calculate_arcs(agent.cur_position, not_checked)

        while agent.cur_position.arcs:
            new_position = max(agent.cur_position.arcs.items(), key=lambda arc: arc[1])[0]
            if new_position in not_checked:
                not_checked.remove(new_position)
            agent.move_to(new_position)

        recalculate_values(agent.cur_position, not_checked)


def main(agent_id: int) -> None:
    "Main function"
    global_graph = initialize_graph("field vertexes.json")

    # TODO: move initial info about agents to config file.

    # TODO: change chanels.
    agents = {
        id_: Agent(id_=id_,
                   chanel=[],
                   global_graph=global_graph,
                   cur_position=global_graph[position_id])
        for id_, position_id in ((0, 0), (1, 20), (2, 24))
    }
    cur_agent = agents[agent_id]

    # TODO: remove loop.
    for agent in agents.values():
        fill_agent_values(agent)

        message_data = list(map(lambda vetrex: (vetrex.id_, vetrex.value),
                                agent.graph.vertexes.values()))

        agent.post_to_chanel(json.dumps(message_data))

    # TODO: add waiting until every one post info about it's values.

    fill_global_values(global_graph, cur_agent, agents.values())

    # Each agent post "ok" to it's chanel after finishing
    # configuration global_graph
    for agent in agents.values():
        agent.post_to_chanel("ok")

    # TODO: delete this vvv.
    # Симулируем пустоту новых сообщений.
    for agent in agents.values():
        agent.chanel = []
    # TODO: delete this ^^^.

    # TODO: Переделать.
    # Костыль для начала выборов по кругу.
    agents[2].post_to_chanel("ok")

    # Spliting graph to subgraphs for each agent.
    split_graph(global_graph, cur_agent, agents)

    for agent in agents.values():
        walk_graph(agent)


if __name__ == "__main__":
    main(0)
