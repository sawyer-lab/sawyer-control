"""
Title       : Experimental I/O Utilities
Module      : sawyer_control.experiments.common
Description : Shared JSON, session, and protobuf helpers for Sawyer experiments.

Author      : Oscar Federico García Castro
Email       : oscar.garciac@cinvestav.mx
Affiliation : CINVESTAV, Robotics
Advisor     : Dr. Gustavo Arechavaleta

Created     : 2026-10-03
Updated     : 2026-10-03
Version     : 0.1.0
Python      : 3.8+
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

from google.protobuf.json_format import MessageToDict


def proto_dict(message: Any) -> dict[str, Any]:
    return MessageToDict(message, preserving_proto_field_name=True)


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as handle:
        for row in rows:
            handle.write(json.dumps(row, allow_nan=False) + '\n')


def next_session_dir(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    indexes: list[int] = []
    for path in root.glob('session-*'):
        if not path.is_dir():
            continue
        prefix = path.name.split('_', maxsplit=1)[0]
        try:
            indexes.append(int(prefix.split('-', maxsplit=1)[1]))
        except (IndexError, ValueError):
            continue
    session = root / f'session-{max(indexes, default=-1) + 1:03d}'
    session.mkdir()
    return session


def slugify(value: str) -> str:
    return (
        value.lower()
        .replace(' hz', 'hz')
        .replace(' lpf', '')
        .replace(', ', '_')
        .replace(' ', '-')
        .replace('/', '-')
    )
