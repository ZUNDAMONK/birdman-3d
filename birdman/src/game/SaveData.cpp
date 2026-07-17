#include "game/SaveData.hpp"
#include "core/DesignIO.hpp"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <sstream>
#if defined(_WIN32)
#include <direct.h>
#define MKDIR(d) _mkdir(d)
#else
#include <sys/stat.h>
#define MKDIR(d) mkdir(d, 0755)
#endif

namespace bm {

static const double PI = 3.14159265358979323846;

void SaveData::load() {
    // best.json: {"dist":123.4,"name":"...","path":[[x,h,yl],...]}
    std::ifstream f(savePath("best.json"));
    if (f) {
        std::stringstream ss; ss << f.rdbuf();
        std::string s = ss.str();
        size_t p = s.find("\"dist\":");
        if (p != std::string::npos) best_.dist = std::atof(s.c_str() + p + 7);
        p = s.find("\"name\":\"");
        if (p != std::string::npos) {
            size_t e = s.find('"', p + 8);
            if (e != std::string::npos) best_.name = s.substr(p + 8, e - p - 8);
        }
        p = s.find("\"path\":[");
        if (p != std::string::npos) {
            const char* c = s.c_str() + p + 8;
            while (*c && *c != ']') {
                if (*c == '[') {
                    GhostPoint g{0, 0, 0};
                    if (std::sscanf(c, "[%lf,%lf,%lf]", &g.x, &g.h, &g.yl) == 3)
                        best_.path.push_back(g);
                    const char* close = std::strchr(c, ']');
                    if (!close) break;
                    c = close + 1;
                } else c++;
            }
        }
        best_.valid = best_.dist > 0;
    }
    // badges.json: [["id","label"],...]
    std::ifstream bf(savePath("badges.json"));
    if (bf) {
        std::stringstream ss; ss << bf.rdbuf();
        std::string s = ss.str();
        size_t p = 0;
        while ((p = s.find("[\"", p)) != std::string::npos) {
            size_t e1 = s.find('"', p + 2);
            if (e1 == std::string::npos) break;
            size_t q1 = s.find('"', e1 + 2);
            size_t q2 = q1 == std::string::npos ? std::string::npos : s.find('"', q1 + 1);
            if (q2 == std::string::npos) break;
            badges_.push_back({s.substr(p + 2, e1 - p - 2), s.substr(q1 + 1, q2 - q1 - 1)});
            p = q2 + 1;
        }
    }
}

void SaveData::saveBest(const BestRun& b) {
    best_ = b;
    best_.valid = true;
    std::ofstream f(savePath("best.json"));
    if (!f) return;
    f << "{\"dist\":" << b.dist << ",\"name\":\"" << b.name << "\",\"path\":[";
    for (size_t i = 0; i < b.path.size(); i++) {
        if (i) f << ",";
        char buf[80];
        std::snprintf(buf, sizeof(buf), "[%.1f,%.2f,%.1f]", b.path[i].x, b.path[i].h, b.path[i].yl);
        f << buf;
    }
    f << "]}";
}

void SaveData::saveBadges() {
    std::ofstream f(savePath("badges.json"));
    if (!f) return;
    f << "[";
    for (size_t i = 0; i < badges_.size(); i++) {
        if (i) f << ",";
        f << "[\"" << badges_[i].first << "\",\"" << badges_[i].second << "\"]";
    }
    f << "]";
}

std::string SaveData::awardBadge(const std::string& id, const std::string& label) {
    for (const auto& b : badges_) if (b.first == id) return "";
    badges_.push_back({id, label});
    saveBadges();
    return label;
}

std::vector<std::string> SaveData::checkMissions(const SimResult& r) {
    std::vector<std::string> got;
    auto add = [&](const char* id, const char* l) {
        std::string m = awardBadge(id, l);
        if (!m.empty()) got.push_back(m);
    };
    if (r.dist >= 1000) add("d1k", u8"★1km達成");
    if (r.dist >= 3000) add("d3k", u8"★3km達成");
    if (r.dist >= 10000) add("d10k", u8"★10km完走!");
    if (r.summer && r.tod >= 15 && r.dist >= 1000) add("afternoon", u8"★午後の浜風で1km(難)");
    if (r.auto_ && r.dist >= 5000) add("auto5k", u8"★オート性能5km");
    if (r.inThermalUsed && r.dist >= 2000) add("thermal", u8"★上昇気流を使って2km");
    if (r.maxBank >= 35 * PI / 180 && r.dist >= 1000) add("turn", u8"★35°バンク旋回で1km");
    return got;
}

} // namespace bm
