"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L1/源码/p3_l1_communication.py
"""



def require(condition, message):
    if not condition:
        raise ValueError(message)


def metrics(rows):
    """An outage event is one consecutive disconnected run inside the window."""
    observed = sum(row['interval_end_s'] - row['time_s'] for row in rows)
    require(observed > 0, 'Empty observation window')
    connected_time = sum(row['interval_end_s'] - row['time_s'] for row in rows if row['connected'])
    episodes = []
    start = None
    for row in rows:
        if not row['connected'] and start is None:
            start = row['time_s']
        if row['connected'] and start is not None:
            episodes.append({'start_s': start, 'end_s': row['time_s'], 'duration_s': row['time_s'] - start})
            start = None
    if start is not None:
        end = rows[-1]['interval_end_s']
        episodes.append({'start_s': start, 'end_s': end, 'duration_s': end - start})
    for episode in episodes:
        episode['starts_at_window_boundary'] = episode['start_s'] == rows[0]['time_s']
        episode['continues_to_window_end'] = episode['end_s'] == rows[-1]['interval_end_s']
    return {'sample_count': len(rows), 'connected_sample_count': sum(row['connected'] for row in rows),
            'observed_boat_seconds': observed, 'connected_boat_seconds': connected_time,
            'retention_rate': connected_time / observed, 'outage_event_count': len(episodes),
            'longest_outage_s': max((episode['duration_s'] for episode in episodes), default=0),
            'outage_intervals': episodes}

