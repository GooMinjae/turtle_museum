# migrate_sqlite_to_pg.py
import os, sqlite3
from datetime import datetime
from pathlib import Path
import psycopg
from dotenv import load_dotenv

# 경로/환경
SQLITE_PATH = Path("/home/rokey/turtlebot4_ws/src/turtle_musium/resource/visit_log.db")
from dotenv import load_dotenv
from urllib.parse import urlparse
import os, socket, psycopg

load_dotenv("/home/rokey/turtlebot4_ws/src/turtle_musium/resource/.env")

PG_URL = os.getenv("DATABASE_URL")
if not PG_URL:
    raise SystemExit("DATABASE_URL not set (확인: resource/.env)")

u = urlparse(PG_URL)
print(f"[INFO] Connecting to {u.hostname}:{u.port} db={u.path.lstrip('/')}", flush=True)

# DNS 확인
try:
    ip = socket.gethostbyname(u.hostname)
    print(f"[INFO] DNS -> {u.hostname} = {ip}", flush=True)
except Exception as e:
    raise SystemExit(f"DNS resolution failed: {e}")

# TCP 사전검사(3초)
try:
    socket.create_connection((u.hostname, u.port or 5432), timeout=3).close()
    print("[INFO] TCP reachable", flush=True)
except Exception as e:
    raise SystemExit(f"TCP connect failed: {e}")

# 실제 DB 연결(5초 제한, SSL 필요한 서비스면 sslmode='require' 추가)
try:
    conn = psycopg.connect(PG_URL, connect_timeout=5)  # 필요시 , sslmode='require'
    print("[INFO] PG connected", flush=True)
except Exception as e:
    raise SystemExit(f"[CONNECT ERROR] {e}")

# SQLite 읽기
src = sqlite3.connect(str(SQLITE_PATH))
src.row_factory = sqlite3.Row
rows = src.execute("SELECT * FROM visit ORDER BY id ASC").fetchall()

print(f"[INFO] Read {len(rows)} rows from SQLite")

# PG 쓰기
with psycopg.connect(PG_URL) as conn:
    print("Dddddd")
    with conn.cursor() as cur:
        inserted = 0
        for r in rows:
            # 컬럼 존재 가드(스키마 조금씩 다른 경우 대비)
            def get(k, default=None):
                return r[k] if k in r.keys() else default

            id_            = get("id")
            date_          = to_date(get("date"))
            planned_count  = get("planned_count", 0)
            gift_moo       = get("gift_moo", 0)
            gift_pinga     = get("gift_pinga", 0)
            gift_haowl     = get("gift_haowl", 0)
            gift_pingu     = get("gift_pingu", 0)
            counted_count  = get("counted_count", 0)
            created_at     = get("created_at") or get("createdAt")  # 컬럼명 변형 대비

            # created_at 파싱 시도(문자열이면)
            if isinstance(created_at, str):
                for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%dT%H:%M:%S", "%Y/%m/%d %H:%M:%S"):
                    try:
                        created_at = datetime.strptime(created_at, fmt)
                        break
                    except Exception:
                        pass

            # PG insert (id를 유지/보존)
            cur.execute("""
                INSERT INTO visit (id, date, planned_count, gift_moo, gift_pinga, gift_haowl, gift_pingu, counted_count, created_at)
                VALUES (%s, %s, %s, %s, %s, %s, %s, %s, COALESCE(%s, now()))
                ON CONFLICT (id) DO NOTHING
            """, (id_, date_, planned_count, gift_moo, gift_pinga, gift_haowl, gift_pingu, counted_count, created_at))
            inserted += cur.rowcount

        # 시퀀스 리셋 (BIGSERIAL이 다음 값부터 잘 나오도록)
        cur.execute("""
            SELECT setval(pg_get_serial_sequence('visit','id'), COALESCE((SELECT MAX(id) FROM visit), 1), TRUE)
        """)
    conn.commit()
print(f"[INFO] Inserted {inserted} new row(s) into PostgreSQL")
print("[DONE] Migration completed.")
