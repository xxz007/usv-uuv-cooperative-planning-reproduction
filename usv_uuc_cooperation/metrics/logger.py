"""Framework V01; functions extracted unchanged from accepted lesson sources.
课时资料/P3/P3-L4/源码/p3_l4_sim.py
"""
import csv
import hashlib



def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_csv(path, rows):
    with path.open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)

