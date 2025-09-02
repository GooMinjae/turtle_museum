# visit_db.py
import os, sqlite3
from datetime import datetime
from typing import Optional, Dict

DB_PATH = "/home/rokey/turtlebot4_ws/src/turtle_musium/resource/visit_log.db"

SCHEMA = """
CREATE TABLE IF NOT EXISTS visit (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  date TEXT NOT NULL,
  planned_count INTEGER NOT NULL,
  gift_moo INTEGER NOT NULL,
  gift_pinga INTEGER NOT NULL,
  gift_haowl INTEGER NOT NULL,
  gift_pingu INTEGER NOT NULL,
  counted_count INTEGER,
  created_at TEXT DEFAULT CURRENT_TIMESTAMP
);
"""

def _connect():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH, check_same_thread=False)
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute(SCHEMA)
    return conn

_CONN = _connect()

def parse_gift_string(s: str):
    # 예: "1203" → moo=1, pinga=2, haowl=0, pingu=3
    s = (s or "").ljust(4, "0")[:4]
    return tuple(int(ch) for ch in s)

def insert_visit(date_str: str, planned_count: int, gift_str: str) -> int:
    ymd = datetime.strptime(date_str, "%Y%m%d").strftime("%Y-%m-%d")
    moo, pinga, haowl, pingu = parse_gift_string(gift_str)
    cur = _CONN.cursor()
    cur.execute("""
        INSERT INTO visit(date, planned_count, gift_moo, gift_pinga, gift_haowl, gift_pingu)
        VALUES (?, ?, ?, ?, ?, ?)
    """, (ymd, int(planned_count), moo, pinga, haowl, pingu))
    _CONN.commit()
    return cur.lastrowid

def update_counted_count(rowid: int, counted: int):
    _CONN.execute(
        "UPDATE visit SET counted_count=? WHERE id=?",
        (int(counted), int(rowid))
    )
    _CONN.commit()

# 스키마에 맞춰 컬럼을 정확히 선택
def get_visit_by_id(visit_id: int) -> Optional[Dict]:
    """
    visit 테이블에서 단일 방문 레코드를 dict로 반환
    컬럼: id, date, planned_count, counted_count,
          gift_moo, gift_pinga, gift_haowl, gift_pingu, created_at
    """
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute("""
            SELECT
              id, date, planned_count, counted_count,
              gift_moo, gift_pinga, gift_haowl, gift_pingu,
              created_at
            FROM visit
            WHERE id = ?
        """, (visit_id,))
        row = cur.fetchone()
        return dict(row) if row else None
    finally:
        conn.close()

# 최근 방문 1건 편의 함수(옵션)
def get_latest_visit() -> Optional[Dict]:
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute("""
            SELECT
              id, date, planned_count, counted_count,
              gift_moo, gift_pinga, gift_haowl, gift_pingu,
              created_at
            FROM visit
            ORDER BY id DESC
            LIMIT 1
        """)
        row = cur.fetchone()
        return dict(row) if row else None
    finally:
        conn.close()

def log_artwork_state(visit_id: int, piece_id: str, state: str, ts: str) -> None:
    """
    예: state = 'arrived' | 'explaining' | 'done' ...
    """

def get_gift_counts_for_visit(visit_id: int) -> dict:
    """
    return {"moo": int, "pinga": int, "haowl": int, "pingu": int}
    # visit 테이블의 gift_* 컬럼을 그대로 꺼내면 됩니다.
    """