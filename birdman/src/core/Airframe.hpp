#pragma once

#include "core/MiniJson.hpp"

#include <glm/glm.hpp>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace bm {

struct AircraftParams;
struct Analysis;

struct Transform3 {
    glm::dvec3 pos{0.0};
    glm::dvec3 rotDeg{0.0};
};

enum class PartKind {
    Fuselage,
    Wing,
    HTail,
    VTail,
    BoomWing,
    Prop,
    Cockpit,
    Gear,
    Fairing,
    Pilot,
    Unknown
};

const char* partKindName(PartKind kind);
PartKind partKindFromString(const std::string& name);

enum class MirrorMode { None, Pair };
const char* mirrorModeName(MirrorMode mode);
MirrorMode mirrorModeFromString(const std::string& name);

struct Hardpoint {
    std::string id;
    Transform3 t;
    // Phase 1予約: 剛性、許容荷重、荷重経路分類。
};

struct Mount {
    std::string parentId;
    std::string hardpointId;
    Transform3 offset;
    MirrorMode mirror = MirrorMode::None;
};

struct MassNode {
    double kg = 0.0;
    glm::dvec3 cgLocal{0.0};
    glm::dmat3 I0{0.0};
    int analysisItem = -1;
};

struct SparDesign {
    int count = 1;
    double chordFrac = 0.30;
    double rootDiaMm = 110.0;
    double tipDiaMm = 60.0;
    int jointCount = 3;
    std::string section = "tube";       // tube / box / i-beam
};

struct FairingDesign {
    double lengthM = 2.1;
    double widthM = 0.75;
    double heightM = 1.35;
    double noseRatio = 0.25;
    double tailRatio = 0.55;
};

struct PilotStationDesign {
    double seatHeightM = 0.62;
    double pedalZM = -0.50;
    double pedalHeightM = 0.58;
    double crankZM = -0.80;
    double crankHeightM = 0.58;
};

struct TailSupportDesign {
    std::string mounting = "cantilever"; // cantilever / strut / wire
    int supportCount = 0;
    double supportDiaMm = 20.0;
};

struct PartDesign {
    std::optional<SparDesign> spar;
    std::optional<FairingDesign> fairing;
    std::optional<PilotStationDesign> pilot;
    std::optional<TailSupportDesign> tailSupport;
};

struct Part {
    std::string id;
    PartKind kind = PartKind::Unknown;
    Mount mount;
    std::vector<Hardpoint> hardpoints;
    // 1部品が構造・駆動・艤装など複数の解析質量項目を担える。
    std::vector<MassNode> massNodes;
    PartDesign design;
};

glm::dmat4 transformMatrix(const Transform3& transform);
glm::dmat4 mirrorMatrix(const glm::dmat4& transform);

// JSON配列 [x,y,z] を読む。失敗時はoutを変更しない。
bool vec3FromJson(const json::Value& value, glm::dvec3& out);

class AirframeGraph {
public:
    struct ValidationError {
        std::string partId;
        std::string message;
    };
    struct Placed {
        const Part* part = nullptr;
        glm::dmat4 world{1.0};
        bool mirrored = false;
    };

    // ファイル読込など、未検証の入力を検査する入口。重複IDもvalidate()で報告する。
    static AirframeGraph fromUntrusted(std::vector<Part> parts);

    bool addPart(Part part, std::string* error = nullptr);
    bool removeSubtree(const std::string& id, std::string* error = nullptr);
    // 編集はコピー上で全グラフを検証し、成功時だけ反映する。部品IDの変更は禁止。
    bool replacePart(const std::string& id, Part replacement, std::string* error = nullptr);
    bool setMount(const std::string& id, Mount mount, std::string* error = nullptr);
    bool setHardpointTransform(const std::string& partId, const std::string& hardpointId,
                               Transform3 transform, std::string* error = nullptr);

    const Part* find(const std::string& id) const;

    std::vector<ValidationError> validate() const;
    std::vector<Placed> resolve() const;

    const std::map<std::string, Part>& parts() const { return parts_; }
    const std::string& rootId() const { return rootId_; }
    std::size_t size() const { return parts_.size(); }

private:
    std::map<std::string, Part> parts_;
    std::string rootId_;
    std::vector<ValidationError> ingestErrors_;
};

struct MassBreakdown {
    double totalKg = 0.0;
    glm::dvec3 cg{0.0};
    glm::dmat3 I{0.0};
    struct Item {
        std::string partId;
        double kg = 0.0;
        glm::dvec3 cgWorld{0.0};
    };
    std::vector<Item> items;
};

// resolve()した全インスタンスを集約し、全機重心まわりの慣性を平行軸の定理で求める。
// Pair部品のMassNode::kgは片側分として2インスタンスへ自動計上される。
MassBreakdown aggregateMass(const AirframeGraph& graph);

struct AeroLayoutProperties {
    bool valid = false;
    double wingLEZ = 0.0;
    double wingIncidenceDeg = 0.0;
    double hTailZ = 0.0;
    double hAreaScale = 0.0;
    double vTailZ = 0.0;
    double vAreaScale = 0.0;
};

// 配置済み主翼・尾翼から空力に必要な前後位置、取付角、投影面積倍率を集約する。
// 水平尾翼は世界Y方向、垂直尾翼は世界X方向への法線投影を有効面積とする。
AeroLayoutProperties aggregateAeroLayout(const AirframeGraph& graph);

struct DesignPhysicsProperties {
    bool valid = false;
    SparDesign spar;
    double fairingCdReduction = 0.0;
    double supportDragAreaM2 = 0.0;
    bool compositionValid = false;
    bool hasPilot = true;
    bool hasGear = true;
    std::string gearType = "tandem";
    double pilotPowerFactor = 1.0;
    double pilotEnergyFactor = 1.0;
    double boomWingAreaM2 = 0.0;
    double boomWingZ = 0.0;
};

// 部品固有設計から梁解析用桁仕様と寄生抗力補正を集約する。
DesignPhysicsProperties aggregateDesignPhysics(const AirframeGraph& graph);
// 姿勢別の標準操縦席を基準に、部品構成と人間工学係数も集約する。
DesignPhysicsProperties aggregateDesignPhysics(const AirframeGraph& graph,
                                               const AircraftParams& st);

// 既存パラメータを真実の源として、決定的な標準機Layoutを生成する。
AirframeGraph buildDefaultLayout(const AircraftParams& st, const Analysis& an);

} // namespace bm
