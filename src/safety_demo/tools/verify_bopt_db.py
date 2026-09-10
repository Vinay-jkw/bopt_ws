#!/usr/bin/env python3
import sqlite3
from pathlib import Path

DB = Path(__file__).resolve().parents[1] / "config" / "bopt_ws_body.db"
EXPECTED = {
    1: "/lidar/left/scan",
    2: "/lidar/right/scan",
    3: "/lidar/front/scan",
    4: "/lidar/back/scan",
    5: "/Lidar_LFT",
    6: "/Lidar_RFT",
}

con = sqlite3.connect(DB)
rows = dict(con.execute("select lidar_id, topic from lidar"))
missing = {k: v for k, v in EXPECTED.items() if rows.get(k) != v}
counts = dict(con.execute("select lidar_id, count(*) from policy group by lidar_id"))
con.close()

print(f"DB: {DB}")
for lid, topic in EXPECTED.items():
    print(f"ID {lid}: {rows.get(lid)}  policies={counts.get(lid, 0)}")

if missing:
    print("ERROR: topic mismatch", missing)
    raise SystemExit(1)
print("OK: BOPT LiDAR topic mapping is correct.")
