"""Discrete stock-access planning, independent of ROS and physics.

Search the occupancy states for the fewest physical moves. Rear stock requires
an empty front slot in the same lane. Each order slot accepts its requested
color; green parts finish on the existing open buffer tray.
"""

from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class StockMove:
    item: str
    source: str
    destination: str


def plan_orders(locations, colors, requirements, *, lanes=3, buffer_slots=2, buffer_color="green"):
    sources = tuple(f"stock_{lane}_{row}" for lane in range(lanes) for row in range(2))
    buffers = tuple(f"buffer_{index}" for index in range(buffer_slots))
    slots = (*sources, *buffers, *requirements)
    if len(set(locations.values())) != len(locations):
        raise ValueError("Two stock items occupy the same slot.")
    if any(slot not in slots for slot in locations.values()) or locations.keys() != colors.keys():
        raise ValueError("Stock locations and colors must describe the same known items.")
    start = tuple(next((item for item, location in locations.items() if location == slot), "")
                  for slot in slots)
    order_start = len(sources) + len(buffers)

    def finished(state):
        return (all(state[index] and colors[state[index]] == requirements[slots[index]]
                    for index in range(order_start, len(slots)))
                and not any(state[:len(sources)])
                and all(not item or colors[item] == buffer_color
                        for item in state[len(sources):order_start]))

    queue = deque([start])
    previous = {start: None}
    while queue:
        state = queue.popleft()
        if finished(state):
            result = []
            while previous[state] is not None:
                state, move = previous[state]
                result.append(move)
            return list(reversed(result))
        # Delivery moves first break equal-cost ties in favor of useful work.
        for destination in (*range(order_start, len(slots)), *range(len(sources), order_start)):
            if state[destination]:
                continue
            for source in range(order_start):
                item = state[source]
                if not item or source == destination:
                    continue
                if source < len(sources) and source % 2 and state[source - 1]:
                    continue
                if destination < order_start and source >= len(sources):
                    continue  # Buffer-to-buffer moves cannot improve access.
                if destination < order_start and colors[item] != buffer_color:
                    continue
                if destination >= order_start and colors[item] != requirements[slots[destination]]:
                    continue
                next_state = list(state)
                next_state[source], next_state[destination] = "", item
                next_state = tuple(next_state)
                if next_state not in previous:
                    previous[next_state] = (state, StockMove(item, slots[source], slots[destination]))
                    queue.append(next_state)
    raise ValueError("Cannot fulfill the orders with the available stock and buffer capacity.")
