#include "core/Airframe.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <sstream>
#include <utility>

namespace bm {
namespace {

constexpr double kPositionLimitM = 1000.0;

bool finiteVec(const glm::dvec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

double normalizeAngle(double angle) {
    if (!std::isfinite(angle)) return angle;
    angle = std::fmod(angle + 180.0, 360.0);
    if (angle < 0.0) angle += 360.0;
    return angle - 180.0;
}

void normalize(Transform3& transform) {
    transform.rotDeg.x = normalizeAngle(transform.rotDeg.x);
    transform.rotDeg.y = normalizeAngle(transform.rotDeg.y);
    transform.rotDeg.z = normalizeAngle(transform.rotDeg.z);
}

void normalize(Part& part) {
    normalize(part.mount.offset);
    for (auto& hp : part.hardpoints) normalize(hp.t);
}

bool validTransform(const Transform3& transform) {
    if (!finiteVec(transform.pos) || !finiteVec(transform.rotDeg)) return false;
    return std::abs(transform.pos.x) <= kPositionLimitM
        && std::abs(transform.pos.y) <= kPositionLimitM
        && std::abs(transform.pos.z) <= kPositionLimitM;
}

const Hardpoint* findHardpoint(const Part& part, const std::string& id) {
    for (const auto& hp : part.hardpoints) {
        if (hp.id == id) return &hp;
    }
    return nullptr;
}

void setError(std::string* error, std::string message) {
    if (error) *error = std::move(message);
}

} // namespace

const char* partKindName(PartKind kind) {
    switch (kind) {
    case PartKind::Fuselage: return "fuselage";
    case PartKind::Wing: return "wing";
    case PartKind::HTail: return "htail";
    case PartKind::VTail: return "vtail";
    case PartKind::BoomWing: return "boomwing";
    case PartKind::Prop: return "prop";
    case PartKind::Cockpit: return "cockpit";
    case PartKind::Gear: return "gear";
    case PartKind::Fairing: return "fairing";
    case PartKind::Pilot: return "pilot";
    default: return "unknown";
    }
}

PartKind partKindFromString(const std::string& name) {
    if (name == "fuselage") return PartKind::Fuselage;
    if (name == "wing") return PartKind::Wing;
    if (name == "htail") return PartKind::HTail;
    if (name == "vtail") return PartKind::VTail;
    if (name == "boomwing") return PartKind::BoomWing;
    if (name == "prop") return PartKind::Prop;
    if (name == "cockpit") return PartKind::Cockpit;
    if (name == "gear") return PartKind::Gear;
    if (name == "fairing") return PartKind::Fairing;
    if (name == "pilot") return PartKind::Pilot;
    return PartKind::Unknown;
}

const char* mirrorModeName(MirrorMode mode) {
    return mode == MirrorMode::Pair ? "pair" : "none";
}

MirrorMode mirrorModeFromString(const std::string& name) {
    return name == "pair" ? MirrorMode::Pair : MirrorMode::None;
}

glm::dmat4 transformMatrix(const Transform3& transform) {
    glm::dmat4 result = glm::translate(glm::dmat4(1.0), transform.pos);
    // 機体座標: X右、Y上、Z尾。R = Ry(yaw) * Rx(pitch) * Rz(roll)。
    result = glm::rotate(result, glm::radians(transform.rotDeg.y), glm::dvec3(0.0, 1.0, 0.0));
    result = glm::rotate(result, glm::radians(transform.rotDeg.x), glm::dvec3(1.0, 0.0, 0.0));
    result = glm::rotate(result, glm::radians(transform.rotDeg.z), glm::dvec3(0.0, 0.0, 1.0));
    return result;
}

glm::dmat4 mirrorMatrix(const glm::dmat4& transform) {
    glm::dmat4 s(1.0);
    s[0][0] = -1.0;
    return s * transform * s;
}

bool vec3FromJson(const json::Value& value, glm::dvec3& out) {
    if (!value.isArray() || value.array.size() != 3) return false;
    for (const auto& component : value.array) {
        if (!component.isNumber() || !std::isfinite(component.number)) return false;
    }
    out = {value.array[0].number, value.array[1].number, value.array[2].number};
    return true;
}

AirframeGraph AirframeGraph::fromUntrusted(std::vector<Part> parts) {
    AirframeGraph graph;
    for (auto& part : parts) {
        normalize(part);
        if (part.id.empty()) {
            graph.ingestErrors_.push_back({"", "part id is empty"});
            continue;
        }
        const std::string id = part.id;
        if (!graph.parts_.emplace(id, std::move(part)).second) {
            graph.ingestErrors_.push_back({id, "duplicate part id"});
        }
    }
    for (const auto& entry : graph.parts_) {
        if (entry.second.mount.parentId.empty()) {
            if (graph.rootId_.empty()) graph.rootId_ = entry.first;
        }
    }
    return graph;
}

bool AirframeGraph::addPart(Part part, std::string* error) {
    normalize(part);
    if (part.id.empty()) { setError(error, "part id is empty"); return false; }
    if (parts_.count(part.id)) { setError(error, "duplicate part id: " + part.id); return false; }
    if (part.mount.parentId.empty()) {
        if (!rootId_.empty()) { setError(error, "graph already has a root"); return false; }
        if (part.kind != PartKind::Fuselage) { setError(error, "root must be a fuselage"); return false; }
    } else {
        if (part.kind == PartKind::Fuselage) { setError(error, "fuselage may only be the root"); return false; }
        const Part* parent = find(part.mount.parentId);
        if (!parent) { setError(error, "missing parent: " + part.mount.parentId); return false; }
        if (!findHardpoint(*parent, part.mount.hardpointId)) {
            setError(error, "missing hardpoint: " + part.mount.hardpointId);
            return false;
        }
    }
    if (!validTransform(part.mount.offset)) { setError(error, "invalid mount transform"); return false; }
    std::set<std::string> hardpointIds;
    for (const auto& hp : part.hardpoints) {
        if (hp.id.empty() || !hardpointIds.insert(hp.id).second || !validTransform(hp.t)) {
            setError(error, "invalid hardpoint");
            return false;
        }
    }
    const MassNode& mass = part.mass;
    bool finiteInertia = true;
    for (int c = 0; c < 3; ++c) for (int r = 0; r < 3; ++r)
        finiteInertia = finiteInertia && std::isfinite(mass.I0[c][r]);
    if (!std::isfinite(mass.kg) || mass.kg < 0.0 || !finiteVec(mass.cgLocal) || !finiteInertia) {
        setError(error, "invalid mass node");
        return false;
    }
    const bool isRoot = part.mount.parentId.empty();
    const std::string id = part.id;
    parts_.emplace(id, std::move(part));
    if (isRoot) rootId_ = id;
    return true;
}

bool AirframeGraph::removeSubtree(const std::string& id, std::string* error) {
    if (!parts_.count(id)) { setError(error, "part not found: " + id); return false; }
    if (id == rootId_) { setError(error, "root cannot be removed"); return false; }
    std::set<std::string> doomed{id};
    bool changed = true;
    while (changed) {
        changed = false;
        for (const auto& entry : parts_) {
            if (!doomed.count(entry.first) && doomed.count(entry.second.mount.parentId)) {
                doomed.insert(entry.first);
                changed = true;
            }
        }
    }
    for (const auto& partId : doomed) parts_.erase(partId);
    return true;
}

const Part* AirframeGraph::find(const std::string& id) const {
    const auto it = parts_.find(id);
    return it == parts_.end() ? nullptr : &it->second;
}

std::vector<AirframeGraph::ValidationError> AirframeGraph::validate() const {
    std::vector<ValidationError> errors = ingestErrors_;
    std::size_t roots = 0;
    for (const auto& entry : parts_) {
        const Part& part = entry.second;
        if (part.mount.parentId.empty()) {
            ++roots;
            if (part.kind != PartKind::Fuselage) errors.push_back({part.id, "root must be a fuselage"});
        } else {
            if (part.kind == PartKind::Fuselage) errors.push_back({part.id, "fuselage is not root"});
            const Part* parent = find(part.mount.parentId);
            if (!parent) errors.push_back({part.id, "missing parent: " + part.mount.parentId});
            else if (!findHardpoint(*parent, part.mount.hardpointId))
                errors.push_back({part.id, "missing hardpoint: " + part.mount.hardpointId});
        }
        if (!validTransform(part.mount.offset)) errors.push_back({part.id, "invalid mount transform"});
        std::set<std::string> hardpointIds;
        for (const auto& hp : part.hardpoints) {
            if (hp.id.empty()) errors.push_back({part.id, "empty hardpoint id"});
            else if (!hardpointIds.insert(hp.id).second) errors.push_back({part.id, "duplicate hardpoint id: " + hp.id});
            if (!validTransform(hp.t)) errors.push_back({part.id, "invalid hardpoint transform: " + hp.id});
        }
        const MassNode& mass = part.mass;
        bool finiteInertia = true;
        for (int c = 0; c < 3; ++c) for (int r = 0; r < 3; ++r)
            finiteInertia = finiteInertia && std::isfinite(mass.I0[c][r]);
        if (!std::isfinite(mass.kg) || mass.kg < 0.0 || !finiteVec(mass.cgLocal) || !finiteInertia)
            errors.push_back({part.id, "invalid mass node"});
    }
    if (roots != 1) errors.push_back({"", "graph must have exactly one root"});

    enum class Mark { Visiting, Done };
    std::map<std::string, Mark> marks;
    std::set<std::string> reported;
    std::vector<std::string> stack;
    std::function<void(const std::string&)> visit = [&](const std::string& id) {
        const auto mark = marks.find(id);
        if (mark != marks.end()) {
            if (mark->second == Mark::Visiting) {
                const auto begin = std::find(stack.begin(), stack.end(), id);
                for (auto it = begin; it != stack.end(); ++it) {
                    if (reported.insert(*it).second) errors.push_back({*it, "cycle includes part"});
                }
            }
            return;
        }
        marks[id] = Mark::Visiting;
        stack.push_back(id);
        const Part& part = parts_.at(id);
        if (!part.mount.parentId.empty() && parts_.count(part.mount.parentId)) visit(part.mount.parentId);
        stack.pop_back();
        marks[id] = Mark::Done;
    };
    for (const auto& entry : parts_) visit(entry.first);
    return errors;
}

std::vector<AirframeGraph::Placed> AirframeGraph::resolve() const {
    if (!validate().empty()) return {};
    std::map<std::string, std::vector<Placed>> placements;
    placements[rootId_] = {{&parts_.at(rootId_), glm::dmat4(1.0), false}};
    std::vector<std::string> topologicalOrder{rootId_};
    std::size_t resolved = 1;
    while (resolved < parts_.size()) {
        bool progressed = false;
        for (const auto& entry : parts_) {
            const Part& part = entry.second;
            if (placements.count(part.id)) continue;
            const auto parentIt = placements.find(part.mount.parentId);
            if (parentIt == placements.end()) continue;
            const Part& parent = parts_.at(part.mount.parentId);
            const Hardpoint* hp = findHardpoint(parent, part.mount.hardpointId);
            std::vector<Placed> current;
            for (const auto& parentPlacement : parentIt->second) {
                const glm::dmat4 local = transformMatrix(hp->t) * transformMatrix(part.mount.offset);
                const glm::dmat4 inheritedLocal = parentPlacement.mirrored ? mirrorMatrix(local) : local;
                const glm::dmat4 base = parentPlacement.world * inheritedLocal;
                current.push_back({&part, base, parentPlacement.mirrored});
                if (part.mount.mirror == MirrorMode::Pair) {
                    // Pairは合成済みワールド行列へS*M*Sを適用する（Euler角の符号操作は禁止）。
                    current.push_back({&part, mirrorMatrix(base), !parentPlacement.mirrored});
                }
            }
            placements.emplace(part.id, std::move(current));
            topologicalOrder.push_back(part.id);
            ++resolved;
            progressed = true;
        }
        if (!progressed) return {};
    }
    std::vector<Placed> result;
    for (const auto& id : topologicalOrder) {
        const auto& partPlacements = placements.at(id);
        result.insert(result.end(), partPlacements.begin(), partPlacements.end());
    }
    return result;
}

} // namespace bm
