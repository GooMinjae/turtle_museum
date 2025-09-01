# visit_db.py
import os, sqlite3
from datetime import datetime
from typing import Optional, Dict
import requests

DB_PATH = "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/visit_log.db"
API_BASE = os.getenv("VISIT_API_BASE", "http://192.168.0.2:8000")

SCHEMA = """
CREATE TABLE IF NOT EXISTS visit (
  id            BIGSERIAL PRIMARY KEY,
  date          DATE        NOT NULL,          -- "YYYYMMDD"를 DATE로 저장
  planned_count INTEGER     NOT NULL,
  gift_moo      INTEGER     NOT NULL,
  gift_pinga    INTEGER     NOT NULL,
  gift_haowl    INTEGER     NOT NULL,
  gift_pingu    INTEGER     NOT NULL,
  counted_count INTEGER     NOT NULL DEFAULT 0,
  created_at    TIMESTAMPTZ NOT NULL DEFAULT now()
);
"""

def _connect():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH, check_same_thread=False)
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute(SCHEMA)
    return conn

_CONN = _connect()


class VisitDbError(Exception):
    pass

def parse_gift_string(s: str):
    # 예: "1203" → moo=1, pinga=2, haowl=0, pingu=3
    s = (s or "").ljust(4, "0")[:4]
    return tuple(int(ch) for ch in s)


def insert_visit(date_str: str, planned_count: int, gift_str: str) -> int:
    """POST /visit -> visit_id 반환"""
    url = f"{API_BASE}/visit"
    try:
        r = requests.post(url, json={
            "date_str": date_str,
            "planned_count": planned_count,
            "gift_str": gift_str
        }, timeout=5)
        r.raise_for_status()
        return r.json()["visit_id"]
    except Exception as e:
        raise VisitDbError(f"insert_visit failed: {e}")

def update_counted_count(visit_id: int, counted: int) -> None:
    """PUT /visit/{id}/counted"""
    url = f"{API_BASE}/visit/{visit_id}/counted"
    try:
        r = requests.put(url, json={"counted": counted}, timeout=5)
        r.raise_for_status()
    except Exception as e:
        raise VisitDbError(f"update_counted_count failed: {e}")

def get_visit_by_id(visit_id: int) -> Optional[Dict]:
    """GET /visit/{id}"""
    url = f"{API_BASE}/visit/{visit_id}"
    try:
        r = requests.get(url, timeout=5)
        if r.status_code == 404:
            return None
        r.raise_for_status()
        return r.json()
    except Exception as e:
        raise VisitDbError(f"get_visit_by_id failed: {e}")

def get_latest_visit() -> Optional[Dict]:
    """GET /visit/latest"""
    url = f"{API_BASE}/visit/latest"
    try:
        r = requests.get(url, timeout=5)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        raise VisitDbError(f"get_latest_visit failed: {e}")