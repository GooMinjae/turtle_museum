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

CREATE TABLE IF NOT EXISTS artwork_log (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  visit_id INTEGER NOT NULL,
  piece_id TEXT NOT NULL,
  state TEXT NOT NULL,  -- 예: 'arrived' | 'explaining' | 'done'
  ts TEXT NOT NULL,     -- ISO 문자열 "YYYY-MM-DD HH:MM:SS"
  FOREIGN KEY (visit_id) REFERENCES visit(id) ON DELETE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_artwork_log_visit_id ON artwork_log(visit_id);
"""

def _connect():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH, check_same_thread=False)
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute("PRAGMA foreign_keys = ON;")  # 외래키 활성화 (권장)
    conn.executescript(SCHEMA)                 # 여러 문장을 한 번에 실행
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


def log_artwork_state(visit_id: int, piece_id: str, state: str, ts: str) -> None:
    """
    예: state = 'arrived' | 'explaining' | 'done' ...
    """
    cur = _CONN.cursor()
    cur.execute("""
        INSERT INTO artwork_log(visit_id, piece_id, state, ts)
        VALUES (?, ?, ?, ?)
    """, (int(visit_id), str(piece_id), str(state), str(ts)))
    _CONN.commit()

def get_gift_counts_for_visit(visit_id: int) -> dict:
    """
    {"gift_moo": int, "gift_pinga": int, "gift_haowl": int, "gift_pingu": int}
    """
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute("""
            SELECT gift_moo, gift_pinga, gift_haowl, gift_pingu
            FROM visit
            WHERE id = ?
        """, (int(visit_id),))
        row = cur.fetchone()
        if not row:
            # 방문이 없으면 0으로 리턴(혹은 예외 발생시켜도 됨)
            return {
                "gift_moo": 0, "gift_pinga": 0, "gift_haowl": 0, "gift_pingu": 0
            }
        return {
            "gift_moo": int(row["gift_moo"]),
            "gift_pinga": int(row["gift_pinga"]),
            "gift_haowl": int(row["gift_haowl"]),
            "gift_pingu": int(row["gift_pingu"]),
        }
    finally:
        conn.close()

def get_daily_counts():
    """
    날짜별 planned_count 합계와 counted_count 합계를 반환.
    예: [{"date":"2025-09-01","planned":30,"counted":28}, ...]
    """
    conn = sqlite3.connect(DB_PATH)
    try:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute("""
            SELECT
              date,
              SUM(planned_count) AS planned,
              SUM(COALESCE(counted_count, 0)) AS counted
            FROM visit
            GROUP BY date
            ORDER BY date
        """)
        rows = cur.fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()