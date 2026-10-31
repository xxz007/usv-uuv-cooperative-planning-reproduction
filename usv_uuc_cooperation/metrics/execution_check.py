"""Audit actual saved displacements, water speed, waypoint cursor and service timing."""
import math

from usv_uuc_cooperation.env.obstacle import forbidden_nodes
from usv_uuc_cooperation.metrics.grid_check import segment_safe


def audit(result):
    config, route = result["config"], result["route"]
    assert config["dt_s"] == 1
    steps, controls = result["steps"], result["controls"]
    horizon = config["cutoff_time_s"]
    assert len(steps) == len(controls) == 2 * horizon
    assert len(result["currents_m_per_s"]) == horizon
    if result["flow_case"] == "NO_FLOW":
        assert all(vector == [0, 0] for vector in result["currents_m_per_s"])
    elif result["flow_case"] == "P4_FLOW":
        for t, vector in enumerate(result["currents_m_per_s"]):
            matching = [r for r in config["current_schedule"] if r["start_s"] <= t < r["end_s"]]
            assert len(matching) == 1 and vector == matching[0]["vector_m_per_s"]
    blocked = forbidden_nodes(config)
    path = route["path"]
    goal = config["tasks_m"]["T03"]
    assert path[0] == config["starts_m"]["U1"] and path[-1] == goal
    geometry = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    assert math.isclose(geometry, route["distance_m"], abs_tol=1e-9)
    lookup = {(r["boat_id"], r["time_s"]): r for r in controls}
    assert len(lookup) == len(controls)
    close = lambda a, b: math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
    peak = 0.0
    task = result["tasks"]["T03"]
    arrivals, service_times, waypoint_times = [], [], []
    for boat in ("U1", "USV"):
        rows = [r for r in steps if r["boat_id"] == boat]
        assert [r["time_s"] for r in rows] == list(range(horizon))
        previous = config["starts_m"][boat]
        distance = energy = 0.0
        cursor = 0
        coefficient = config["energy"]["uuc_coefficient_NEU_per_m" if boat == "U1" else "usv_coefficient_NEU_per_m"]
        for row in rows:
            t = row["time_s"]
            control = lookup[boat, t]
            assert row["next_time_s"] == control["next_time_s"] == t + 1
            start = [row["start_x_m"], row["start_y_m"]]
            end = [row["end_x_m"], row["end_y_m"]]
            assert start == previous and segment_safe(start, end, config, blocked)
            current = result["currents_m_per_s"][t]
            ground = [end[k] - start[k] for k in (0, 1)]
            water = [ground[k] - current[k] for k in (0, 1)]
            speed = math.hypot(*water)
            assert speed <= config["speed_m_per_s"][boat] + 1e-9
            assert close(control["water_speed_m_per_s"], speed)
            for k, axis in enumerate(("x", "y")):
                assert close(control[f"current_{axis}_m_per_s"], current[k])
                assert close(control[f"ground_{axis}_m_per_s"], ground[k])
                assert close(control[f"water_{axis}_m_per_s"], water[k])
            actual = math.dist(start, end)
            assert close(row["actual_distance_m"], actual) and close(row["energy_before_NEU"], energy)
            distance += actual
            energy += coefficient * actual
            assert close(row["delta_energy_NEU"], coefficient * actual)
            assert close(row["cumulative_distance_m"], distance)
            assert close(row["cumulative_energy_NEU"], energy)
            assert energy <= row["budget_NEU"] + 1e-9
            assert row["rejection_reason"] == "", "Main subcase must execute without rejection"
            if boat == "U1":
                peak = max(peak, speed)
                assert control["plan_index_before"] == cursor
                expected = cursor
                if row["action"] == "move":
                    assert cursor < len(path) - 1
                    waypoint = path[cursor + 1]
                    assert [control["waypoint_x_m"], control["waypoint_y_m"]] == waypoint
                    # Position stays on this one segment; no leftover time crosses a waypoint.
                    assert close(math.dist(start, end) + math.dist(end, waypoint), math.dist(start, waypoint))
                    if end == waypoint:
                        expected += 1
                        waypoint_times.append({"path_index": cursor + 1, "arrival_s": t + 1})
                    if row["event_at_end"] == "arrived":
                        assert end == goal and expected == len(path) - 1
                        arrivals.append(t + 1)
                elif row["action"] == "service":
                    assert cursor >= len(path) - 1 and start == end == goal
                    assert task["arrival_s"] is not None and t >= task["arrival_s"]
                    service_times.append(t)
                    assert row["service_progress_s"] == len(service_times)
                    expected += 1
                else:
                    assert row["action"] == "finished_hold" and end == start
                assert control["plan_index_after"] == expected
                cursor = expected
            previous = end
        total = result["summary"]["boat_totals"][boat]
        assert close(distance, total["distance_m"]) and close(energy, total["energy_NEU"])
        assert previous == total["position_m"]
    assert arrivals == [task["arrival_s"]]
    assert service_times == list(range(task["arrival_s"], task["arrival_s"] + config["service_s_per_task"]))
    assert task["status"] == "completed" and task["completion_s"] == service_times[-1] + 1
    assert close(result["summary"]["boat_totals"]["U1"]["distance_m"], geometry)
    if result["flow_case"] == "NO_FLOW":
        expected_time = sum(math.ceil(math.dist(a, b) / config["speed_m_per_s"]["U1"] - 1e-10)
                            for a, b in zip(path, path[1:]))
        assert task["arrival_s"] == expected_time
    return {"status": "PASS", "scope": "teacher mechanical audit; learner explanation separate",
            "actual_distance_m": result["summary"]["boat_totals"]["U1"]["distance_m"],
            "energy_NEU": result["summary"]["boat_totals"]["U1"]["energy_NEU"],
            "arrival_s": task["arrival_s"], "first_service_interval_s": [service_times[0], service_times[0] + 1],
            "completion_s": task["completion_s"], "peak_water_speed_m_per_s": peak,
            "waypoint_arrivals": waypoint_times,
            "checks": ["independent closed-square geometry", "reconstructed water-speed limit",
                       "actual displacement and E=cL ledger", "one waypoint per dt for both modes",
                       "actual arrival before full continuous service", "no-flow per-edge discrete timing"]}
