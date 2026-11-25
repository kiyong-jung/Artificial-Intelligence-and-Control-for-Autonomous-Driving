"""두 모델을 같은 seed(같은 교통)에서 바퀴끼리 짝지어 비교한다. 바퀴마다 끝난 이유를 읽어 완주·안전 사건을 센다.

    PYTHONPATH=. python research/snail/paired.py it16 rd3 --seed 61000 quiet baseline busy heavy rich_lo

  arrived       완주 바퀴 수          safety  충돌·보행자 충돌·적색 통과 바퀴 수
  only_a/only_b 한 모델만 완주한 바퀴 수(나머지는 둘 다 완주 또는 둘 다 실패)
"""
import argparse
import gzip
import json
import os

SAFETY = ("collision", "pedestrian_collision", "red_light")


def endings(path):
    out = {}
    with gzip.open(path, "rt") as f:
        for line in f:
            r = json.loads(line)
            out[r["episode"]] = r["event"]["termination_reason"] or "unfinished"
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("a")
    parser.add_argument("b")
    parser.add_argument("scenarios", nargs="+")
    parser.add_argument("--seed", default="61000")
    args = parser.parse_args()
    print(f"scenario n arrived({args.a}/{args.b}) safety({args.a}/{args.b}) only_{args.a} only_{args.b} reasons")
    for s in args.scenarios:
        paths = [f"data/research/eval/{label}_s{args.seed}_{s}.jsonl.gz" for label in (args.a, args.b)]
        if not all(os.path.exists(p) for p in paths):
            print(s, "missing", [p for p in paths if not os.path.exists(p)])
            continue
        a, b = (endings(p) for p in paths)
        common = sorted(set(a) & set(b))
        arrived = [sum(x[e] == "arrived" for e in common) for x in (a, b)]
        safety = [sum(x[e] in SAFETY for e in common) for x in (a, b)]
        only_a = sum(a[e] == "arrived" and b[e] != "arrived" for e in common)
        only_b = sum(b[e] == "arrived" and a[e] != "arrived" for e in common)
        reasons = [{r: list(x[e] for e in common).count(r) for r in sorted(set(x[e] for e in common))} for x in (a, b)]
        print(s, len(common), f"{arrived[0]}/{arrived[1]}", f"{safety[0]}/{safety[1]}", only_a, only_b,
              json.dumps(reasons, separators=(",", ":")))


if __name__ == "__main__":
    main()
