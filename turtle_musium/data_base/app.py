# app.py
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime
import os
from sqlalchemy import create_engine, text
from sqlalchemy.exc import SQLAlchemyError
from fastapi.responses import RedirectResponse

from dotenv import load_dotenv
from pathlib import Path

# # data_base/app.py 기준으로 ../resource/.env 를 로드
# dotenv_path = Path(__file__).resolve().parent.parent / "resource" / ".env"
# load_dotenv(dotenv_path=dotenv_path)
dotenv_path = Path(__file__).resolve().parent.parent / "resource" / ".env"
load_dotenv(dotenv_path=dotenv_path)

DB_URL = os.getenv("DATABASE_URL")
if not DB_URL:
    raise RuntimeError("DATABASE_URL not set")

engine = create_engine(DB_URL, pool_pre_ping=True, pool_size=5, max_overflow=5)

app = FastAPI(title="Visit API", version="1.0.0")

# ---------- Pydantic Schemas ----------
class VisitCreate(BaseModel):
    date_str: str = Field(..., description='"YYYYMMDD"')
    planned_count: int
    gift_str: str = Field(..., description='예: "1203" (moo, pinga, haowl, pingu)')

class CountUpdate(BaseModel):
    counted: int

def parse_gift_str(s: str):
    # "1203" -> [1,2,0,3]
    if not (len(s) == 4 and s.isdigit()):
        raise ValueError("gift_str must be 4 digits, e.g., '1203'")
    moo, pinga, haowl, pingu = [int(c) for c in s]
    return moo, pinga, haowl, pingu

def to_date(date_str: str):
    # "YYYYMMDD" -> date
    try:
        return datetime.strptime(date_str, "%Y%m%d").date()
    except ValueError:
        raise HTTPException(400, "date_str must be 'YYYYMMDD'")

# ---------- Endpoints ----------
@app.post("/visit")
def create_visit(body: VisitCreate):
    d = to_date(body.date_str)
    try:
        moo, pinga, haowl, pingu = parse_gift_str(body.gift_str)
    except ValueError as e:
        raise HTTPException(400, str(e))

    sql = text("""
        INSERT INTO visit (date, planned_count, gift_moo, gift_pinga, gift_haowl, gift_pingu)
        VALUES (:date, :planned_count, :gift_moo, :gift_pinga, :gift_haowl, :gift_pingu)
        RETURNING id
    """)
    try:
        with engine.begin() as conn:
            new_id = conn.execute(sql, {
                "date": d, "planned_count": body.planned_count,
                "gift_moo": moo, "gift_pinga": pinga,
                "gift_haowl": haowl, "gift_pingu": pingu
            }).scalar_one()
        return {"visit_id": new_id}
    except SQLAlchemyError as e:
        raise HTTPException(500, f"DB error: {e}")

@app.put("/visit/{visit_id}/counted")
def set_counted(visit_id: int, body: CountUpdate):
    sql = text("""
        UPDATE visit SET counted_count = :counted WHERE id = :id
        RETURNING id
    """)
    try:
        with engine.begin() as conn:
            row = conn.execute(sql, {"id": visit_id, "counted": body.counted}).fetchone()
            if not row:
                raise HTTPException(404, "visit_id not found")
        return {"ok": True}
    except SQLAlchemyError as e:
        raise HTTPException(500, f"DB error: {e}")

@app.get("/visit/{visit_id}")
def get_visit(visit_id: int):
    sql = text("""
        SELECT id, date, planned_count, counted_count,
               gift_moo, gift_pinga, gift_haowl, gift_pingu, created_at
        FROM visit WHERE id = :id
    """)
    try:
        with engine.begin() as conn:
            row = conn.execute(sql, {"id": visit_id}).mappings().fetchone()
            if not row:
                raise HTTPException(404, "not found")
            return dict(row)
    except SQLAlchemyError as e:
        raise HTTPException(500, f"DB error: {e}")

@app.get("/visit/latest")
def get_latest_visit():
    sql = text("""
        SELECT id, date, planned_count, counted_count,
               gift_moo, gift_pinga, gift_haowl, gift_pingu, created_at
        FROM visit
        ORDER BY id DESC
        LIMIT 1
    """)
    try:
        with engine.begin() as conn:
            row = conn.execute(sql).mappings().fetchone()
            return dict(row) if row else None
    except SQLAlchemyError as e:
        raise HTTPException(500, f"DB error: {e}")

@app.get("/", include_in_schema=False)
def root():
    return RedirectResponse(url="/docs")

@app.get("/healthz")
def healthz():
    return {"status": "ok"}