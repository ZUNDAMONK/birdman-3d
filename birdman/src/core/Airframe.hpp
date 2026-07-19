#pragma once

#include "core/MiniJson.hpp"

#include <glm/glm.hpp>

#include <map>
#include <string>
#include <vector>

namespace bm {

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

struct Part {
    std::string id;
    PartKind kind = PartKind::Unknown;
    Mount mount;
    std::vector<Hardpoint> hardpoints;
    MassNode mass;
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

    const Part* find(const std::string& id) const;
    Part* find(const std::string& id);

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

} // namespace bm
