# -*- coding: utf-8 -*-
"""
单车多任务路径规划（TSP）仿真脚本
================================
用途：验证「一台车从站点出发，送 N 个送达点，怎么排序走最短」
     对比三种策略：随机顺序 / 最近邻贪心 / 最近邻+2opt改进

运行方式（不需要装任何第三方库）：
    python tsp_demo.py

输出：
    1) 控制台：每个点数下的平均路程与改进率
    2) tsp_结果.csv：所有算例的原始数据（可直接用 Excel 打开画图）
    3) 如果装了 matplotlib，还会存一张对比图
"""

import math
import random
import csv
import os

# ==================== 1. 场景参数（改这里就行）====================
SITE = (0.0, 0.0)            # 站点坐标（小车出发点 = 取货点）
FIELD = 30.0                 # 送达点随机分布范围（米，30 = 一个 30x30 米的区域）
SPEED = 0.5                  # 小车速度 m/s（用来把路程换算成时间）
REPEAT = 30                  # 每个点数下，随机生成多少组算例（取平均更可信）
POINT_COUNTS = [3, 5, 8]     # 要测的送达点数（3个点/5个点/8个点）
RANDOM_SEED = 42             # 随机种子（固定住，结果可复现）

OUT_CSV = "tsp_结果.csv"

# ==================== 2. 基础计算 ====================
def dist(a, b):
    """两点直线距离"""
    return math.hypot(a[0] - b[0], a[1] - b[1])

def route_len(route, pts):
    """
    计算一条路径的总路程
    route: 送达点的访问顺序（存的是索引），例如 [2, 0, 1]
    路径形状: 站点 -> 第1个点 -> 第2个点 -> ... -> 最后一个点 -> 回到站点
    """
    if not route:
        return 0.0
    total = dist(SITE, pts[route[0]])
    for i in range(len(route) - 1):
        total += dist(pts[route[i]], pts[route[i + 1]])
    total += dist(pts[route[-1]], SITE)
    return total

# ==================== 3. 三种策略 ====================
def strategy_random(n, rng):
    """策略A：随机顺序（基准，用来当对照组）"""
    r = list(range(n))
    rng.shuffle(r)
    return r

def strategy_nearest(pts):
    """
    策略B：最近邻贪心
    做法：从站点出发，每次挑「离当前位置最近的、还没送过的」点
    特点：快，但容易越走越绕（贪心只看眼前）
    """
    n = len(pts)
    unvisited = set(range(n))
    route = []
    cur = SITE
    while unvisited:
        nxt = min(unvisited, key=lambda i: dist(cur, pts[i]))
        route.append(nxt)
        unvisited.remove(nxt)
        cur = pts[nxt]
    return route

def strategy_nearest_2opt(pts):
    """
    策略C：最近邻 + 2-opt 局部改进
    2-opt 做法：反复尝试「翻转路径中的一段」，只要变短就保留
    特点：比纯贪心好，是经典的改进手段
    """
    best = strategy_nearest(pts)
    best_len = route_len(best, pts)
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 1):
            for j in range(i + 1, len(best)):
                # 把 best[i..j] 这一段反过来
                cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                cand_len = route_len(cand, pts)
                if cand_len < best_len - 1e-9:
                    best, best_len = cand, cand_len
                    improved = True
    return best

# ==================== 4. 跑实验 ====================
def run_experiment():
    rng = random.Random(RANDOM_SEED)
    rows = []          # 原始数据
    summary = []       # 汇总

    for n in POINT_COUNTS:
        lens = {"random": [], "nearest": [], "2opt": []}
        for k in range(REPEAT):
            # 随机生成 n 个送达点
            pts = [(rng.uniform(0, FIELD), rng.uniform(0, FIELD)) for _ in range(n)]

            r_a = strategy_random(n, rng)
            r_b = strategy_nearest(pts)
            r_c = strategy_nearest_2opt(pts)

            la, lb, lc = route_len(r_a, pts), route_len(r_b, pts), route_len(r_c, pts)
            lens["random"].append(la)
            lens["nearest"].append(lb)
            lens["2opt"].append(lc)

            rows.append({
                "送达点数": n, "算例": k + 1,
                "随机_路程m": round(la, 3), "随机_时间s": round(la / SPEED, 2),
                "最近邻_路程m": round(lb, 3), "最近邻_时间s": round(lb / SPEED, 2),
                "2opt_路程m": round(lc, 3), "2opt_时间s": round(lc / SPEED, 2),
                "2opt比随机省%": round((la - lc) / la * 100, 2),
                "2opt比最近邻省%": round((lb - lc) / lb * 100, 2) if lb > 0 else 0.0,
            })

        avg_a = sum(lens["random"]) / REPEAT
        avg_b = sum(lens["nearest"]) / REPEAT
        avg_c = sum(lens["2opt"]) / REPEAT
        summary.append({
            "n": n, "random": avg_a, "nearest": avg_b, "2opt": avg_c,
            "improve_vs_random": (avg_a - avg_c) / avg_a * 100,
            "improve_vs_nearest": (avg_b - avg_c) / avg_b * 100 if avg_b > 0 else 0.0,
        })

    # 写 CSV
    with open(OUT_CSV, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # 控制台汇总
    print("=" * 78)
    print("单车多任务路径规划（TSP）仿真结果")
    print(f"参数：送达点分布范围 {FIELD}m ｜ 车速 {SPEED}m/s ｜ 每档算例数 {REPEAT}")
    print("=" * 78)
    print(f"{'送达点数':<10}{'随机(米)':<14}{'最近邻(米)':<14}{'2opt(米)':<14}"
          f"{'2opt比随机省':<14}{'2opt比最近邻省':<16}")
    print("-" * 78)
    for s in summary:
        print(f"{s['n']:<10}{s['random']:<14.1f}{s['nearest']:<14.1f}{s['2opt']:<14.1f}"
              f"{s['improve_vs_random']:<14.1f}{s['improve_vs_nearest']:<16.1f}")
    print("-" * 78)
    print(f"原始数据已写入：{os.path.abspath(OUT_CSV)}")
    print("（用 Excel 打开，可以按『送达点数』做分组对比图）")
    print()

    # 顺便打印一个示例：5 个点的路径顺序
    rng2 = random.Random(RANDOM_SEED + 1)
    demo_pts = [(rng2.uniform(0, FIELD), rng2.uniform(0, FIELD)) for _ in range(5)]
    demo = strategy_nearest_2opt(demo_pts)
    print("【示例】5 个送达点，2-opt 给出的访问顺序：")
    print("   站点 -> " + " -> ".join(f"点{i+1}" for i in demo) + " -> 站点")
    print("   总路程：%.1f 米 ｜ 预计耗时：%.1f 秒" % (route_len(demo, demo_pts),
                                                    route_len(demo, demo_pts) / SPEED))

    # 可选画图（装了 matplotlib 才画）
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        xs = [s["n"] for s in summary]
        plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei"]
        plt.rcParams["axes.unicode_minus"] = False
        plt.figure(figsize=(7, 4.5))
        plt.plot(xs, [s["random"] for s in summary], "o--", label="随机顺序")
        plt.plot(xs, [s["nearest"] for s in summary], "s--", label="最近邻贪心")
        plt.plot(xs, [s["2opt"] for s in summary], "^-", label="最近邻+2opt")
        plt.xlabel("送达点数")
        plt.ylabel("平均总路程（米）")
        plt.title("三种策略的平均路径长度对比")
        plt.grid(alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig("tsp_对比图.png", dpi=150)
        print("\n对比图已保存：tsp_对比图.png")
    except ImportError:
        print("\n（未安装 matplotlib，跳过画图；CSV 用 Excel 画也一样）")


if __name__ == "__main__":
    run_experiment()
