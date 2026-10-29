#!/usr/bin/env python3
"""Summarize STD/TEASER/GICP logs without treating uncomputed quality fields as zeros."""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

ANSI = re.compile(r"\x1b\[[0-9;]*m")
QUALITY_STAGES = {"accepted", "low_overlap", "high_inlier_mse", "degenerate_geometry", "missing_covariances"}
COARSE_EARLY = {"teaser_not_built", "few_points", "few_features", "few_fpfh_pairs",
                "degenerate_pairs", "invalid_teaser_transform", "empty_relocation_snapshot"}

def quantile(values, p):
    a = sorted(values)
    pos = (len(a) - 1) * p
    lo, hi = math.floor(pos), math.ceil(pos)
    return a[lo] + (a[hi] - a[lo]) * (pos - lo)

def stats(values):
    a = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not a:
        return {"n": 0, "median": None}
    return dict(n=len(a), min=min(a), p10=quantile(a, .1), p25=quantile(a, .25),
                median=statistics.median(a), p75=quantile(a, .75), p90=quantile(a, .9), max=max(a))

def parse(path):
    raw = path.read_bytes()
    text = ANSI.sub("", raw.decode(errors="replace"))
    candidates, registrations, gicp = {}, {}, {}
    matched_per_frame = collections.Counter()
    for line in text.splitlines():
        m = re.search(r"STD (\d+) -> (\d+) matched:", line)
        if m:
            key = tuple(map(int, m.groups()))
            pairs = re.search(r"\bstd_pairs=(\d+)", line)
            candidates[key] = int(pairs.group(1)) if pairs else None
            matched_per_frame[key[1]] += 1
        m = re.search(r"Registration (\d+) -> (\d+) (accepted|passed|rejected):", line)
        if m:
            h, k, outcome = m.groups()
            fields = re.search(r"coarse_inliers=(\d+)/(\d+).*?coarse_rmse=(\S+) m.*?gicp=(\w+).*?stage=(\w+)", line)
            if fields:
                n, p, rmse, executed, stage = fields.groups()
                rmse = None if rmse == "n/a" else float(rmse)
                valid = int(p) > 0 and stage not in COARSE_EARLY and rmse is not None
                registrations[(int(h), int(k))] = dict(
                    outcome=outcome, stage=stage, coarse_fpfh_pairs=int(p),
                    coarse_inliers=int(n) if valid else None,
                    coarse_ratio=int(n)/int(p) if valid else None,
                    coarse_rmse_m=rmse if valid else None,
                    coarse_mse_m2=rmse**2 if valid else None,
                    gicp_executed=executed == "executed")
        m = re.search(r"GICP (\d+) -> (\d+): mse=(\S+) m2 overlap=(\S+) current_overlap=(\S+) inliers=(\d+) info_ratio=(\S+)", line)
        if m:
            h, k, mse, source, current, n, info = m.groups()
            gicp[(int(h), int(k))] = dict(
                gicp_mse_m2=float(mse), gicp_source_overlap=float(source),
                gicp_current_overlap=float(current), gicp_inliers=int(n),
                gicp_information_ratio=float(info))
    rows = []
    for h, k in sorted(set(candidates) | set(registrations), key=lambda a: (a[1], a[0])):
        key = (h, k)
        r = dict(historical_index=h, current_index=k, std_pairs=candidates.get(key),
                 registration_recorded=key in registrations)
        r.update(registrations.get(key, {}))
        valid = r.get("gicp_executed", False) and r.get("stage") in QUALITY_STAGES and key in gicp
        r["gicp_quality_evaluated"] = valid
        if valid:
            r.update(gicp[key])
            mse = r["gicp_mse_m2"]
            r["gicp_rmse_m"] = math.sqrt(mse) if math.isfinite(mse) and mse >= 0 else None
            if not math.isfinite(mse):
                r["gicp_mse_m2"] = None
            if r.get("stage") in {"low_overlap", "high_inlier_mse"}:
                # Information is computed after these gates; logged zero is not a measured value.
                r["gicp_information_ratio"] = None
        rows.append(r)
    metrics = ("std_pairs", "coarse_fpfh_pairs", "coarse_inliers", "coarse_ratio",
               "coarse_rmse_m", "coarse_mse_m2", "gicp_source_overlap",
               "gicp_current_overlap", "gicp_mse_m2", "gicp_rmse_m")
    measured = [r for r in rows if r.get("registration_recorded")]
    summary = dict(
        log=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest(),
        counts=dict(std_candidates=len(candidates), frames_with_std_candidates=len(matched_per_frame),
                    registration_decisions=len(registrations),
                    gicp_executed=sum(r.get("gicp_executed", False) for r in measured),
                    gicp_quality_evaluated=sum(r["gicp_quality_evaluated"] for r in measured),
                    solver_accepted=sum(r.get("stage") == "accepted" for r in measured),
                    graph_loops_accepted=len(re.findall(r"Loop \d+ -> \d+ accepted:", text)),
                    skipped_registration_candidates=len(set(candidates)-set(registrations)),
                    dropped_tasks=max([int(x) for x in re.findall(r"dropped_tasks=(\d+)", text)] or [0])),
        stages=dict(collections.Counter(r.get("stage") for r in measured)),
        metrics={key:stats(r.get(key) for r in rows) for key in metrics},
        accepted_metrics={key:stats(r.get(key) for r in measured if r.get("stage") == "accepted") for key in metrics},
        std_candidates_per_nonempty_frame=stats(matched_per_frame.values()))
    return rows, summary

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="append", required=True, help="LABEL=LOG_PATH")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    summaries = {}
    for item in args.run:
        label, value = item.split("=", 1)
        if not re.fullmatch(r"[A-Za-z0-9_-]+", label):
            parser.error("Labels must contain only letters, digits, _ and -")
        rows, summary = parse(Path(value))
        summaries[label] = summary
        fields = list(dict.fromkeys(key for row in rows for key in row))
        with (args.out / (label + "_candidates.csv")).open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    result = dict(methodology={
        "std_pairs": "Raw STD triangle pairs for a selected time/distance-eligible candidate; not FPFH point pairs.",
        "coarse_ratio": "Computed as inliers / FPFH pairs, not the rounded ratio in the log.",
        "coarse_mse_m2": "Square each candidate's logged coarse RMSE, then compute statistics.",
        "gicp": "Conditional on coarse gates and the task budget. Unexecuted or early-return fields are excluded.",
        "labels": "Solver acceptance is not a ground-truth true-loop label; these summaries alone do not prove optimal thresholds.",
        "std_candidates_per_frame": "Only frames with at least one selected candidate; zero-candidate frames are not logged."},
        runs=summaries)
    (args.out / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    titles = {
        "std_pairs": "STD 三角形匹配对数", "coarse_fpfh_pairs": "FPFH 匹配对数",
        "coarse_inliers": "粗配准内点数", "coarse_ratio": "粗配准内点比例",
        "coarse_rmse_m": "粗配准 RMSE (m)", "coarse_mse_m2": "粗配准 MSE (m²)",
        "gicp_source_overlap": "GICP 子图覆盖率", "gicp_current_overlap": "GICP 当前帧覆盖率",
        "gicp_mse_m2": "GICP MSE (m²)", "gicp_rmse_m": "GICP RMSE (m)"}
    lines = ["# 回环统计", "", "每个指标均保留有效样本数；未执行的质量检查不按零计入。", ""]
    for label, summary in summaries.items():
        lines += ["## " + label, "", "计数：" + json.dumps(summary["counts"], ensure_ascii=False), "",
                  "| 指标 | N | P10 | 中位数 | P90 |", "|---|---:|---:|---:|---:|"]
        for key, title in titles.items():
            s = summary["metrics"][key]
            fmt = lambda name: "—" if s.get(name) is None else format(s[name], ".6g")
            lines.append(f"| {title} | {s['n']} | {fmt('p10')} | {fmt('median')} | {fmt('p90')} |")
        lines += ["", "拒绝/接受原因：" + json.dumps(summary["stages"], ensure_ascii=False), "",
                  "原始日志：" + summary["log"], ""]
    lines += ["## 口径和限制", "", "STD 三角形匹配对与 FPFH 匹配对分别统计。GICP 分布受粗配准门限筛选，不能代表全部候选。",
              "已接受的回环不等于真值确认的正确回环。中位数用于观察分布，不直接当作通用接受门限。", ""]
    (args.out / "report.md").write_text("\n".join(lines))
    print(json.dumps({label: {"counts":s["counts"], "medians":{k:v["median"] for k,v in s["metrics"].items()}} for label,s in summaries.items()}, indent=2))

if __name__ == "__main__":
    main()
