// 機体設計 自動最適化CLI
// オート操縦(性能計測モード)でヘッドレス飛行させ、飛距離を目的関数として
// 進化的探索(エリート+ガウス変異、世代ごとにステップ縮小)で機体パラメータを最適化する。
//
// 使い方:
//   optimize [--power 270] [--pilot 60] [--gens 30] [--pop 64] [--seed 1]
//            [--base <designs.jsonの設計名>] [--name <保存名>]
// 結果は save/designs.json に追記され、ゲームの「保存・比較」から読み込める。
#include "core/Aircraft.hpp"
#include "core/Physics.hpp"
#include "core/DesignIO.hpp"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <random>
#include <thread>
#include <atomic>
#include <vector>
#include <algorithm>
#include <functional>
#include <chrono>
#include <ctime>

using namespace bm;

// ---- 最適化対象パラメータの定義 ----
struct PSpec {
    const char* name;
    double mn, mx, snap;                       // 範囲と刻み
    std::function<double(const AircraftParams&)> get;
    std::function<void(AircraftParams&, double)> set;
};

static std::vector<PSpec> paramSpecs() {
    auto D = [](double AircraftParams::*m) {
        return std::make_pair(
            std::function<double(const AircraftParams&)>([m](const AircraftParams& s) { return s.*m; }),
            std::function<void(AircraftParams&, double)>([m](AircraftParams& s, double v) { s.*m = v; }));
    };
    std::vector<PSpec> v;
    auto add = [&](const char* n, double mn, double mx, double snap, double AircraftParams::*m) {
        auto gs = D(m);
        v.push_back({n, mn, mx, snap, gs.first, gs.second});
    };
    add("span",      18, 38, 0.5,  &AircraftParams::span);
    add("rootChord", 0.6, 1.4, 0.05, &AircraftParams::rootChord);
    add("tipChord",  0.2, 1.1, 0.05, &AircraftParams::tipChord);
    add("washout",   0, 6, 0.5,   &AircraftParams::washout);
    add("ribPitch",  0.12, 0.4, 0.01, &AircraftParams::ribPitch);
    add("plankTop",  0, 70, 5,    &AircraftParams::plankTop);
    add("plankBot",  0, 70, 5,    &AircraftParams::plankBot);
    add("propDia",   2.0, 3.8, 0.1, &AircraftParams::propDia);
    add("seatX",     0.2, 2.5, 0.05, &AircraftParams::seatX);
    add("tailArm",   3.0, 8.0, 0.1, &AircraftParams::tailArm);
    add("hSpan",     2.0, 5.0, 0.1, &AircraftParams::hSpan);
    add("hChord",    0.35, 0.9, 0.05, &AircraftParams::hChord);
    add("vHeight",   0.8, 2.2, 0.05, &AircraftParams::vHeight);
    add("vChord",    0.35, 0.9, 0.05, &AircraftParams::vChord);
    add("rootDia",   70, 150, 5,  &AircraftParams::rootDia);
    add("tipDia",    30, 150, 5,  &AircraftParams::tipDia);
    add("wingX",     0.8, 3.0, 0.05, &AircraftParams::wingX);
    return v;
}

// ---- 評価 ----
struct EvalOut {
    double fitness = -1;        // 飛距離 m (無効設計は<0)
    double dist = 0;
    const char* reject = nullptr;
    Analysis a;
};

static EvalOut evaluate(const AircraftParams& st, const SimParams& basePrm) {
    EvalOut r;
    r.a = analyze(st);
    // ---- 妥当性フィルタ(3自由度物理では表現されない要件を静解析で担保) ----
    if (r.a.SM < 2)  { r.reject = "SM<2% (不安定)"; return r; }
    if (r.a.SM > 20) { r.reject = "SM>20% (舵が重すぎ)"; return r; }
    if (r.a.Vh < 0.30 || r.a.Vh > 0.80) { r.reject = "Vh範囲外"; return r; }
    if (r.a.Vv < 0.003) { r.reject = "Vv不足(方向安定)"; return r; }
    if (st.tipChord > st.rootChord) { r.reject = "逆テーパー"; return r; }
    if (st.tipDia > st.rootDia) { r.reject = "桁逆テーパー"; return r; }
    SimParams prm = basePrm;
    AircraftConstants c = aeroPack(st, r.a, prm);
    if (c.nFail < 1.5) { r.reject = "桁強度不足(nFail<1.5)"; return r; }
    // 突風1.0m/sでの桁安全率
    SimParams gp = prm; gp.gust = 1.0;
    GustResult g = gustCalc(st, r.a, gp);
    if (g.sfGust < 1.0) { r.reject = "突風時に桁折損"; return r; }
    // ---- 手動発進の生存チェック ----
    // オートは高度ホールドで失速発進を救ってしまうため、手動(無入力)でも
    // プラットフォームから発進できる設計だけを許す(プレイアブル保証)
    {
        SimParams mp = prm; mp.auto_ = false; mp.hold = false;
        FlightState Lm = makeInitialState(c, mp, 0);
        double minH = 99;
        while (!Lm.done && Lm.t < 30) {
            if (mp.sixdof) stepSim6(Lm, c, mp, 0.02); else stepSim(Lm, c, mp, 0.02);
            if (Lm.t > 1.0) minH = std::min(minH, Lm.h);
        }
        if (Lm.splash) { r.reject = "手動発進不能(発進速度に対しVs過大)"; return r; }
        if (minH < 0.8) { r.reject = "発進ダイブが水面スレスレ(余裕なし)"; return r; }
    }
    // ---- オート飛行(決定的: 乱流・サーマルなし) ----
    FlightState L = makeInitialState(c, prm, 0);
    while (!L.done && L.t < 3650) {
        if (prm.sixdof) stepSim6(L, c, prm, 0.02); else stepSim(L, c, prm, 0.02);
    }
    r.dist = L.path;
    r.fitness = L.path;
    // 30km打切りに達した設計同士は「速く着いた方が優秀」:
    // 残り時間を平均対地速度で外挿した仮想距離で順位付けする
    if (L.path >= 29999 && L.t > 1 && L.t < 3600)
        r.fitness = 30000.0 + (3600.0 - L.t) * (L.path / L.t);
    return r;
}

int main(int argc, char** argv) {
    double power = 270, pilotW = 60, v0 = 7.0;
    int gens = 30, pop = 64;
    unsigned seed = 1;
    bool sixdof = false, pareto = false;
    std::string baseName, outName;
    for (int i = 1; i < argc; i++) {
        auto arg = [&](const char* k) { return !std::strcmp(argv[i], k) && i + 1 < argc; };
        if (arg("--power")) power = std::atof(argv[++i]);
        else if (arg("--pilot")) pilotW = std::atof(argv[++i]);
        else if (arg("--gens")) gens = std::atoi(argv[++i]);
        else if (arg("--pop")) pop = std::atoi(argv[++i]);
        else if (arg("--seed")) seed = (unsigned)std::atoi(argv[++i]);
        else if (arg("--base")) baseName = argv[++i];
        else if (arg("--name")) outName = argv[++i];
        else if (arg("--v0")) v0 = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--sixdof")) sixdof = true;
        else if (!std::strcmp(argv[i], "--pareto")) pareto = true;
        else if (!std::strcmp(argv[i], "--help")) {
            std::printf("usage: optimize [--power W] [--pilot kg] [--gens N] [--pop N] [--seed N]\n"
                        "                [--base designName] [--name saveName] [--v0 m/s] [--sixdof]\n");
            return 0;
        }
    }

    // ベース機体: デフォルト or designs.jsonの既存設計
    AircraftParams base;
    if (!baseName.empty()) {
        bool found = false;
        for (const auto& d : loadDesigns())
            if (d.name == baseName) { base = d.st; found = true; break; }
        if (!found) { std::printf(u8"設計 '%s' が見つかりません\n", baseName.c_str()); return 1; }
    }
    base.powerMax = power;
    base.pilotW = pilotW;

    SimParams prm;                       // 静穏・オート計測条件
    prm.summer = false; prm.wind = 0; prm.xwind = 0;
    prm.turb = 0; prm.thermal = 0; prm.gust = 0; prm.pjit = false;
    prm.auto_ = true; prm.temp = 20; prm.hTgt = 2.0;
    prm.V0 = v0; prm.sixdof = sixdof;

    const auto specs = paramSpecs();
    const int NP = (int)specs.size();
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uni(0, 1);
    std::normal_distribution<double> gauss(0, 1);

    auto snap = [&](int i, double v) {
        v = std::round(v / specs[i].snap) * specs[i].snap;
        return clamp(v, specs[i].mn, specs[i].mx);
    };
    auto toParams = [&](const std::vector<double>& x) {
        AircraftParams st = base;
        for (int i = 0; i < NP; i++) specs[i].set(st, x[i]);
        return st;
    };
    auto fromParams = [&](const AircraftParams& st) {
        std::vector<double> x(NP);
        for (int i = 0; i < NP; i++) x[i] = specs[i].get(st);
        return x;
    };

    const unsigned NT = std::max(1u, std::thread::hardware_concurrency());
    std::printf(u8"=== 機体最適化: power=%.0fW pilot=%.0fkg V0=%.1fm/s 物理=%s gens=%d pop=%d threads=%u ===\n",
                power, pilotW, v0, sixdof ? u8"6DOF" : u8"3DOF", gens, pop, NT);

    std::vector<double> bestX = fromParams(base);
    EvalOut bestE = evaluate(toParams(bestX), prm);
    std::printf(u8"ベース設計: %s (dist=%.0fm)\n",
                bestE.reject ? bestE.reject : "OK", bestE.dist);

    const auto t0 = std::chrono::steady_clock::now();
    long evals = 0, rejects = 0;
    // --pareto用: 有効評価のアーカイブ(飛距離 vs 桁安全率nFail)
    struct ArchEntry { std::vector<double> x; double fit, nFail; };
    std::vector<ArchEntry> archive;
    for (int gen = 0; gen < gens; gen++) {
        const double sigma = 0.28 * std::pow(0.90, gen);   // 変異幅(範囲比)を縮小
        // 候補生成: エリート変異 + 初期世代はランダムサンプル多め
        std::vector<std::vector<double>> cand(pop);
        for (int k = 0; k < pop; k++) {
            std::vector<double> x(NP);
            const bool randomSample = gen == 0 && k >= pop / 4;
            for (int i = 0; i < NP; i++) {
                const double range = specs[i].mx - specs[i].mn;
                double v = randomSample
                    ? specs[i].mn + uni(rng) * range
                    : bestX[i] + gauss(rng) * sigma * range * (uni(rng) < 0.35 ? 1.0 : 0.0);
                x[i] = snap(i, v);
            }
            cand[k] = std::move(x);
        }
        // 並列評価
        std::vector<EvalOut> res(pop);
        std::vector<std::thread> ths;
        std::atomic_int next{0};
        for (unsigned t = 0; t < NT; t++)
            ths.emplace_back([&] {
                int k;
                while ((k = next.fetch_add(1)) < pop)
                    res[k] = evaluate(toParams(cand[k]), prm);
            });
        for (auto& th : ths) th.join();
        // 更新
        int genBest = -1;
        for (int k = 0; k < pop; k++) {
            evals++;
            if (res[k].reject) { rejects++; continue; }
            if (pareto) {
                AircraftParams cs = toParams(cand[k]);
                Analysis ca = analyze(cs);
                archive.push_back({cand[k], res[k].fitness, computeLoads(cs, ca, 1).SF});
            }
            if (res[k].fitness > bestE.fitness) { bestE = res[k]; bestX = cand[k]; genBest = k; }
        }
        std::printf(u8"gen %2d/%d  best=%7.0f m (fit %.0f) %s  (σ=%.2f)\n", gen + 1, gens,
                    bestE.dist, bestE.fitness, genBest >= 0 ? "*" : " ", sigma);
    }
    const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf(u8"\n%ld評価 (%ld棄却) / %.1f秒\n", evals, rejects, sec);

    if (bestE.reject) { std::printf(u8"有効な設計が見つかりませんでした\n"); return 1; }

    // ---- 結果表示 ----
    AircraftParams bst = toParams(bestX);
    std::printf(u8"\n=== 最良設計: 飛距離 %.0f m ===\n", bestE.dist);
    std::printf(u8"  %-10s %8s %8s\n", "param", "base", "best");
    for (int i = 0; i < NP; i++) {
        const double b = specs[i].get(base), v = bestX[i];
        std::printf("  %-10s %8.2f %8.2f%s\n", specs[i].name, b, v,
                    std::abs(b - v) > 1e-9 ? "  *" : "");
    }
    std::printf(u8"  全備 %.1fkg  SM %.1f%%  所要 %.0fW  S=%.1fm2 AR=%.0f\n",
                bestE.a.W, bestE.a.SM, bestE.a.Preq, bestE.a.S, bestE.a.AR);

    // ---- designs.json に保存 ----
    DesignEntry e;
    e.st = bst;
    if (outName.empty()) {
        char nm[64];
        std::time_t tt = std::time(nullptr);
        std::tm* lt = std::localtime(&tt);
        std::snprintf(nm, sizeof(nm), u8"opt-%02d%02d_%02d%02d (%.0fW)",
                      lt->tm_mon + 1, lt->tm_mday, lt->tm_hour, lt->tm_min, power);
        e.name = nm;
    } else e.name = outName;
    e.dist = bestE.dist;
    fillMetrics(e, prm);
    auto designs = loadDesigns();
    designs.push_back(e);
    saveDesigns(designs);
    std::printf(u8"\nsave/designs.json に '%s' として保存しました。\n"
                u8"ゲームの設計モード→保存・比較 から読み込めます。\n", e.name.c_str());

    // ---- パレートフロント: 飛距離 vs 桁安全率の非劣解を保存 ----
    if (pareto && !archive.empty()) {
        std::vector<ArchEntry> front;
        for (const auto& a : archive) {
            bool dominated = false;
            for (const auto& b : archive)
                if (b.fit >= a.fit && b.nFail >= a.nFail &&
                    (b.fit > a.fit || b.nFail > a.nFail)) { dominated = true; break; }
            if (!dominated) front.push_back(a);
        }
        std::sort(front.begin(), front.end(),
                  [](const ArchEntry& a, const ArchEntry& b) { return a.nFail > b.nFail; });
        // 安全率でほぼ同じ解は間引いて最大5つ
        std::vector<ArchEntry> picked;
        for (const auto& f : front) {
            if (!picked.empty() && std::abs(picked.back().nFail - f.nFail) < 0.12) continue;
            picked.push_back(f);
            if (picked.size() >= 5) break;
        }
        std::printf(u8"\n=== パレートフロント(記録重視 ⇔ 安全重視) %d解 ===\n", (int)picked.size());
        auto ds2 = loadDesigns();
        for (size_t i = 0; i < picked.size(); i++) {
            DesignEntry pe;
            pe.st = toParams(picked[i].x);
            char nm[64];
            std::snprintf(nm, sizeof(nm), u8"pareto%d SF%.1f", (int)i + 1, picked[i].nFail);
            pe.name = nm;
            pe.dist = std::min(30000.0, picked[i].fit);
            fillMetrics(pe, prm);
            ds2.push_back(pe);
            std::printf(u8"  %-16s 飛距離%7.0fm  桁安全率 %.2f\n", nm,
                        std::min(30000.0, picked[i].fit), picked[i].nFail);
        }
        saveDesigns(ds2);
        std::printf(u8"designs.json に保存しました(保存・比較タブで選べます)\n");
    }
    return 0;
}
