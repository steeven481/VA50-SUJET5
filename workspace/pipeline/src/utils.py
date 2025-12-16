# src/utils.py

import os, json

def read_json(path):
    with open(path, "r") as f:
        return json.load(f)

def write_json(obj, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)

def pixels_to_table(u, v):
    """
    Pour l'instant, x = u et y = v.
    Tu remplaceras plus tard par une vraie
    homographie caméra -> table.
    """
    return float(u), float(v)
