#pragma once
// キャリアモード(年次大会・予算・評判) と チャレンジミッション定義
#include "core/Types.hpp"
#include <functional>

namespace bm {

// ---- キャリア ----
struct CareerYearRec {
    int year = 1;
    std::string wx;         // 大会当日の天候名
    double dist = 0;        // 記録 m
    double prize = 0;       // 賞金 万円
    bool sixdof = false;    // 大会時の物理モデル(旧保存は標準)
    bool assist = true;     // 大会時の拡張物理補助(旧保存はON)
};
struct CareerState {
    int year = 1;
    double money = 150;     // 資金 万円
    int rep = 0;            // 評判(実績で上がり、高級パーツが解禁される)
    std::vector<CareerYearRec> hist;
};

class Career {
public:
    void load();            // save/career.json
    void save() const;
    CareerState st;

    double budget() const;                       // 今年使える製作予算
    struct CostItem { std::string name; double cost; };
    // 機体の製作コスト内訳(万円)
    static std::vector<CostItem> costBreakdown(const AircraftParams& p, const Analysis& a);
    static double totalCost(const AircraftParams& p, const Analysis& a);
    // 評判ロック違反(空なら出場可)
    std::vector<std::string> lockViolations(const AircraftParams& p) const;
    // 大会結果を記録(賞金・評判・翌年へ)。rank>0ならライバル込み順位(1=優勝ボーナス)
    std::string recordContest(const std::string& wxName, double dist, double cost,
                              int rank = 0, int field = 0,
                              bool sixdof = false, bool assist = true);
};

// ---- チャレンジミッション ----
struct Mission {
    const char* id;
    const char* name;
    const char* desc;
    const char* goalText;
    // prm/機体への上書き(挑戦開始時)。機体は終了後に復元される
    std::function<void(SimParams&, AircraftParams&)> apply;
    // 達成判定(飛行終了時)
    std::function<bool(const SimResult&, double lapTime)> goal;
};
const std::vector<Mission>& missions();

} // namespace bm
