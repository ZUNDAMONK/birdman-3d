#include "game/Career.hpp"
#include "core/Material.hpp"
#include "core/Aircraft.hpp"
#include "core/DesignIO.hpp"
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cmath>

namespace bm {

// ---- 保存/読込 (save/career.json) ----
static double jn(const std::string& s, const char* k, double def) {
    std::string key = std::string("\"") + k + "\":";
    size_t p = s.find(key);
    return p == std::string::npos ? def : std::atof(s.c_str() + p + key.size());
}

void Career::load() {
    std::ifstream f(savePath("career.json"));
    if (!f) return;
    std::stringstream ss; ss << f.rdbuf();
    const std::string s = ss.str();
    st.year = (int)jn(s, "year", 1);
    st.money = jn(s, "money", 150);
    st.rep = (int)jn(s, "rep", 0);
    st.hist.clear();
    size_t p = s.find("\"hist\":[");
    if (p != std::string::npos) {
        int depth = 0; size_t start = 0;
        for (size_t i = p; i < s.size(); i++) {
            if (s[i] == '{') { if (!depth) start = i; depth++; }
            else if (s[i] == '}') {
                if (--depth == 0) {
                    const std::string o = s.substr(start, i - start + 1);
                    CareerYearRec r;
                    r.year = (int)jn(o, "y", 0);
                    r.dist = jn(o, "d", 0);
                    r.prize = jn(o, "p", 0);
                    size_t w = o.find("\"w\":\"");
                    if (w != std::string::npos) {
                        size_t e = o.find('"', w + 5);
                        if (e != std::string::npos) r.wx = o.substr(w + 5, e - w - 5);
                    }
                    st.hist.push_back(r);
                }
            } else if (s[i] == ']' && !depth) break;
        }
    }
}

void Career::save() const {
    std::ofstream f(savePath("career.json"));
    if (!f) return;
    f << "{\"year\":" << st.year << ",\"money\":" << st.money << ",\"rep\":" << st.rep << ",\"hist\":[";
    for (size_t i = 0; i < st.hist.size(); i++) {
        if (i) f << ",";
        f << "{\"y\":" << st.hist[i].year << ",\"w\":\"" << st.hist[i].wx
          << "\",\"d\":" << st.hist[i].dist << ",\"p\":" << st.hist[i].prize << "}";
    }
    f << "]}";
}

double Career::budget() const {
    // 初年度からデフォルト機(≈154万円)がぎりぎり作れる水準に設定
    return 160 + st.rep * 25 + (st.year - 1) * 5 + std::max(0.0, st.money - 150) * 0.5;
}

std::vector<Career::CostItem> Career::costBreakdown(const AircraftParams& p, const Analysis& a) {
    std::vector<CostItem> v;
    const MaterialGrade& mat = materialOf(p.sparMat);
    v.push_back({u8"CFRP主桁", (a.wSpar + a.wJoints) * 3.5 * mat.costFactor});
    v.push_back({u8"リブ・プランク", a.nRibs * 0.35 + a.S * ((p.plankTop + p.plankBot) / 2) / 100 * 1.6});
    v.push_back({u8"フィルム", a.S * 0.55});
    v.push_back({u8"プロペラ", (0.45 + p.propDia * 0.18) * (p.propMat == "carbon" ? 32 : 9) * (0.8 + 0.1 * p.propBlades)});
    v.push_back({u8"駆動系", a.driveDist * 3.5 + (p.drive == "shaft" ? 12 : 6)});
    v.push_back({u8"尾翼", (a.Sh + a.Sv) * 5.5 + (p.elevRatio < 1 ? 3 : 0) + (p.rudRatio < 1 ? 3 : 0)});
    v.push_back({u8"コックピット", 14.0 + (p.fairing ? 12 : 0) + (p.gear != "none" ? 4 : 0)});
    v.push_back({u8"桁接合部", (p.segments - 1) * 3.0});
    if (p.ailMode != "none") v.push_back({u8"エルロン機構", 6 + p.ailSpanFrac * 20});
    return v;
}

double Career::totalCost(const AircraftParams& p, const Analysis& a) {
    double t = 0;
    for (const auto& it : costBreakdown(p, a)) t += it.cost;
    return t;
}

std::vector<std::string> Career::lockViolations(const AircraftParams& p) const {
    std::vector<std::string> v;
    if (p.sparMat == "m40j" && st.rep < 2) v.push_back(u8"M40J高弾性CFRPは評判2で解禁");
    if (p.propMat == "carbon" && st.rep < 1) v.push_back(u8"CFRPプロペラは評判1で解禁");
    if (p.powerMax >= 320 && st.rep < 3) v.push_back(u8"エースパイロットは評判3で解禁");
    if (p.span > 32 && st.rep < 2) v.push_back(u8"翼幅32m超は評判2で解禁");
    return v;
}

std::string Career::recordContest(const std::string& wxName, double dist, double cost,
                                  int rank, int field) {
    double prize = dist >= 10000 ? 300 : dist >= 5000 ? 150 : dist >= 1000 ? 60 : dist >= 100 ? 20 : 0;
    int repGain = dist >= 10000 ? 3 : dist >= 5000 ? 2 : dist >= 1000 ? 1 : 0;
    if (rank == 1) { prize += 80; repGain += 1; }             // 優勝ボーナス
    CareerYearRec r;
    r.year = st.year; r.wx = wxName; r.dist = dist; r.prize = prize;
    st.hist.push_back(r);
    st.money = std::max(0.0, st.money - cost) + prize + 60;   // 賞金+スポンサー料60
    st.rep += repGain;
    st.year++;
    save();
    char b[200];
    if (rank > 0)
        std::snprintf(b, sizeof(b), u8"第%d回大会 %d位/%d機 %.0fm — 賞金%.0f万円 評判+%d%s",
                      r.year, rank, field, dist, prize, repGain, rank == 1 ? u8" 優勝!!" : "");
    else
        std::snprintf(b, sizeof(b), u8"第%d回大会 %.0fm — 賞金%.0f万円 評判+%d", r.year, dist, prize, repGain);
    return b;
}

// ---- チャレンジミッション ----
const std::vector<Mission>& missions() {
    static const std::vector<Mission> M = {
        {"calm1k", u8"朝凪の1km", u8"快晴・朝凪の絶好コンディションで基本を確認",
         u8"1km以上飛行",
         [](SimParams& p, AircraftParams&) { p.summer = true; p.weather = 0; p.tod = 6.0; },
         [](const SimResult& r, double) { return r.dist >= 1000; }},
        {"rookie5k", u8"ルーキーの挑戦", u8"200Wの初心者パイロット+体力モデルで機体効率が試される",
         u8"5km以上飛行",
         [](SimParams& p, AircraftParams& s) { s.powerMax = 200; p.P = 200; p.stamina = true; p.summer = true; p.weather = 0; p.tod = 6.5; },
         [](const SimResult& r, double) { return r.dist >= 5000; }},
        {"hira", u8"比良おろしを生き延びろ", u8"荒天。西岸の吹き下ろしと突風に警戒せよ",
         u8"荒天で1km以上飛行",
         [](SimParams& p, AircraftParams&) { p.summer = true; p.weather = 3; p.tod = 10.0; },
         [](const SimResult& r, double) { return r.dist >= 1000; }},
        {"noon", u8"午後の魔物", u8"浜風が発達した15時発進。低空の乱流に注意",
         u8"15時発進で1km以上",
         [](SimParams& p, AircraftParams&) { p.summer = true; p.weather = 1; p.tod = 15.0; },
         [](const SimResult& r, double) { return r.dist >= 1000; }},
        {"soar", u8"サーマルライダー", u8"強い上昇気流の日。風の粒子(緑)を目印に上昇帯を継げ",
         u8"上昇気流を使って3km以上",
         [](SimParams& p, AircraftParams&) { p.summer = true; p.weather = 4; p.tod = 12.0; p.thermal = 1.0; },
         [](const SimResult& r, double) { return r.dist >= 3000 && r.inThermalUsed; }},
        {"lapTA", u8"北パイロンTA", u8"北パイロン(11km)を回ってPFへ。50分を切れ",
         u8"周回50分以内",
         [](SimParams& p, AircraftParams&) { p.summer = true; p.weather = 0; p.tod = 6.0; },
         [](const SimResult&, double lap) { return lap > 0 && lap <= 3000; }},
    };
    return M;
}

} // namespace bm
