"""옛 교통량의 생성 파일이 바뀌지 않았는지 본다: PYTHONPATH=. python research/snail/build_check.py <출력 폴더> quiet baseline busy heavy
(두 worktree에서 각각 돌려 routes.rou.xml·scenario.sumocfg 해시를 비교한다)."""
import sys, json, os
from env.road_config import configure_map, ROAD, EGO, TRAFFIC
from env.road_builder import build
out = sys.argv[1]
for sc in sys.argv[2:]:
    configure_map(sc, seed=7, map_id="hapjeong")
    d = os.path.join(out, sc); build(ROAD, EGO, TRAFFIC, d)
    m = json.load(open(os.path.join(d, "manifest.json")))
    print(sc, m["generated_sha256"]["routes.rou.xml"][:16], m["generated_sha256"]["scenario.sumocfg"][:16])
