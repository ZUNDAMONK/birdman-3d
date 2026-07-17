#pragma once
// セーブ・ベスト記録・バッジ (JS loadBest/saveBest/loadBadges/awardBadge/checkMissions)
#include "core/Types.hpp"

namespace bm {

struct GhostPoint { double x, h, yl; };
struct BestRun {
    double dist = 0;
    std::string name;
    std::vector<GhostPoint> path;
    bool valid = false;
};

class SaveData {
public:
    void load();                                   // save/best.json + save/badges.json
    const BestRun& best() const { return best_; }
    void saveBest(const BestRun& b);
    // 未取得なら授与してラベルを返す。取得済みなら空文字
    std::string awardBadge(const std::string& id, const std::string& label);
    // 達成バッジ判定 (JS checkMissions)
    std::vector<std::string> checkMissions(const SimResult& r);
    const std::vector<std::pair<std::string, std::string>>& badges() const { return badges_; }

private:
    void saveBadges();
    BestRun best_;
    std::vector<std::pair<std::string, std::string>> badges_;   // id, label
};

} // namespace bm
