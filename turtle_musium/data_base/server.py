# pip install fastapi uvicorn pydantic requests
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
# from turtle_musium_ui.visit_db import insert_visit, update_counted_count, get_visit_by_id, get_latest_visit
from visit_db import insert_visit, update_counted_count, get_visit_by_id, get_latest_visit

app = FastAPI()

class VisitCreate(BaseModel):
    date_str: str        # "YYYYMMDD"
    planned_count: int
    gift_str: str        # "1203" 같은 4자리

class CountUpdate(BaseModel):
    counted: int

@app.post("/visit")
def create_visit(body: VisitCreate):
    try:
        vid = insert_visit(body.date_str, body.planned_count, body.gift_str)
        return {"visit_id": vid}
    except Exception as e:
        raise HTTPException(400, str(e))

@app.put("/visit/{visit_id}/counted")
def set_counted(visit_id: int, body: CountUpdate):
    try:
        update_counted_count(visit_id, body.counted)
        return {"ok": True}
    except Exception as e:
        raise HTTPException(400, str(e))

@app.get("/visit/{visit_id}")
def read_visit(visit_id: int):
    row = get_visit_by_id(visit_id)
    if not row:
        raise HTTPException(404, "not found")
    return row

@app.get("/visit/latest")
def read_latest():
    row = get_latest_visit()
    if not row:
        raise HTTPException(404, "empty")
    return row
