#pragma once
// 機体設計のJSON入出力 (save/designs.json)。
// ゲームの保存・比較UIと最適化CLIの両方から使う(SFML非依存)。
#include "core/Types.hpp"
#include "core/Airframe.hpp"

#include <optional>

namespace bm {

struct DesignEntry {
    std::string name;
    AircraftParams st;
    std::optional<AirframeGraph> layout;
    // 保存時点のメトリクススナップショット
    double b = 0, W = 0, SM = 0, Preq = 0, LD = 0;
    double dist = -1;               // 最後のシミュ距離(-1=なし)
};

struct AircraftJsonReport {
    int schemaVersion = 1;
    bool layoutPresent = false;
    bool layoutAccepted = false;
    std::vector<std::string> warnings;
};

std::string aircraftToJson(const AircraftParams& st);
AircraftJsonReport aircraftFromJson(const std::string& json, AircraftParams& st);
std::string aircraftToJson(const AircraftParams& st, const AirframeGraph* layout);
AircraftJsonReport aircraftFromJson(const std::string& json, AircraftParams& st, AirframeGraph* layout);

// exeのあるディレクトリ(作業ディレクトリに依存しないリソース解決用)
std::string exeDirPath();
// exe位置基準の保存パス。save/ディレクトリも作成する
std::string savePath(const std::string& filename);

std::vector<DesignEntry> loadDesigns(const std::string& path = "");
void saveDesigns(const std::vector<DesignEntry>& v, const std::string& path = "");

// メトリクスを analyze + computePolar から埋める
void fillMetrics(DesignEntry& e, const SimParams& prm);

} // namespace bm
