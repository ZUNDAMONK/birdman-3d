#include "core/Airframe.hpp"
#include "core/Types.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cassert>
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

bool validChoice(const std::string& value, std::initializer_list<const char*> choices) {
    for (const char* choice : choices) if (value == choice) return true;
    return false;
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

std::string validationMessage(const AirframeGraph::ValidationError& error) {
    return error.partId.empty() ? error.message : error.partId + ": " + error.message;
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
        if (part.mount.mirror != MirrorMode::None) { setError(error, "root cannot use mirror pair"); return false; }
    } else {
        if (part.kind == PartKind::Fuselage) { setError(error, "fuselage may only be the root"); return false; }
        const Part* parent = find(part.mount.parentId);
        if (!parent) { setError(error, "missing parent: " + part.mount.parentId); return false; }
        if (!findHardpoint(*parent, part.mount.hardpointId)) {
            setError(error, "missing hardpoint: " + part.mount.hardpointId);
            return false;
        }
        if (part.mount.mirror == MirrorMode::Pair) {
            std::set<std::string> ancestors;
            const Part* ancestor = parent;
            while (ancestor && ancestors.insert(ancestor->id).second) {
                if (ancestor->mount.mirror == MirrorMode::Pair) {
                    setError(error, "nested mirror pair is not supported");
                    return false;
                }
                ancestor = ancestor->mount.parentId.empty() ? nullptr : find(ancestor->mount.parentId);
            }
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
    for (const MassNode& mass : part.massNodes) {
        bool finiteInertia = true;
        for (int c = 0; c < 3; ++c) for (int r = 0; r < 3; ++r)
            finiteInertia = finiteInertia && std::isfinite(mass.I0[c][r]);
        if (!std::isfinite(mass.kg) || mass.kg < 0.0 || !finiteVec(mass.cgLocal) || !finiteInertia) {
            setError(error, "invalid mass node");
            return false;
        }
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

bool AirframeGraph::replacePart(const std::string& id, Part replacement, std::string* error) {
    const auto existing = parts_.find(id);
    if (existing == parts_.end()) { setError(error, "part not found: " + id); return false; }
    normalize(replacement);
    if (replacement.id != id) { setError(error, "part id cannot be changed"); return false; }

    AirframeGraph candidate = *this;
    candidate.parts_[id] = std::move(replacement);
    const auto errors = candidate.validate();
    if (!errors.empty()) {
        setError(error, validationMessage(errors.front()));
        return false;
    }
    *this = std::move(candidate);
    return true;
}

bool AirframeGraph::setMount(const std::string& id, Mount mount, std::string* error) {
    const Part* current = find(id);
    if (!current) { setError(error, "part not found: " + id); return false; }
    if (id == rootId_) { setError(error, "root mount cannot be changed"); return false; }
    Part replacement = *current;
    replacement.mount = std::move(mount);
    return replacePart(id, std::move(replacement), error);
}

bool AirframeGraph::setHardpointTransform(const std::string& partId, const std::string& hardpointId,
                                          Transform3 transform, std::string* error) {
    const Part* current = find(partId);
    if (!current) { setError(error, "part not found: " + partId); return false; }
    Part replacement = *current;
    auto hardpoint = std::find_if(replacement.hardpoints.begin(), replacement.hardpoints.end(),
        [&](const Hardpoint& hp) { return hp.id == hardpointId; });
    if (hardpoint == replacement.hardpoints.end()) {
        setError(error, "hardpoint not found: " + hardpointId);
        return false;
    }
    hardpoint->t = std::move(transform);
    return replacePart(partId, std::move(replacement), error);
}

const Part* AirframeGraph::find(const std::string& id) const {
    const auto it = parts_.find(id);
    return it == parts_.end() ? nullptr : &it->second;
}

std::vector<AirframeGraph::ValidationError> AirframeGraph::validate() const {
    std::vector<ValidationError> errors = ingestErrors_;
    std::size_t roots = 0;
    std::set<int> analysisItems;
    for (const auto& entry : parts_) {
        const Part& part = entry.second;
        if (part.mount.parentId.empty()) {
            ++roots;
            if (part.kind != PartKind::Fuselage) errors.push_back({part.id, "root must be a fuselage"});
            if (part.mount.mirror != MirrorMode::None)
                errors.push_back({part.id, "root cannot use mirror pair"});
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
        for (const MassNode& mass : part.massNodes) {
            bool finiteInertia = true;
            for (int c = 0; c < 3; ++c) for (int r = 0; r < 3; ++r)
                finiteInertia = finiteInertia && std::isfinite(mass.I0[c][r]);
            if (!std::isfinite(mass.kg) || mass.kg < 0.0 || !finiteVec(mass.cgLocal) || !finiteInertia)
                errors.push_back({part.id, "invalid mass node"});
            if (mass.analysisItem >= 0 && !analysisItems.insert(mass.analysisItem).second)
                errors.push_back({part.id, "duplicate analysis mass item"});
        }
        if (part.design.spar) {
            const SparDesign& value = *part.design.spar;
            if (part.kind != PartKind::Wing || value.count < 1 || value.count > 4
                || !std::isfinite(value.chordFrac) || value.chordFrac < 0.1 || value.chordFrac > 0.8
                || !std::isfinite(value.rootDiaMm) || value.rootDiaMm < 20 || value.rootDiaMm > 300
                || !std::isfinite(value.tipDiaMm) || value.tipDiaMm < 10 || value.tipDiaMm > 300
                || value.jointCount < 0 || value.jointCount > 12
                || !validChoice(value.section, {"tube", "box", "i-beam"}))
                errors.push_back({part.id, "invalid spar design"});
        }
        if (part.design.fairing) {
            const FairingDesign& value = *part.design.fairing;
            if (part.kind != PartKind::Fairing || !std::isfinite(value.lengthM) || value.lengthM < 0.5 || value.lengthM > 6
                || !std::isfinite(value.widthM) || value.widthM < 0.25 || value.widthM > 2.5
                || !std::isfinite(value.heightM) || value.heightM < 0.25 || value.heightM > 2.5
                || !std::isfinite(value.noseRatio) || value.noseRatio < 0.05 || value.noseRatio > 0.45
                || !std::isfinite(value.tailRatio) || value.tailRatio < 0.2 || value.tailRatio > 0.9)
                errors.push_back({part.id, "invalid fairing design"});
        }
        if (part.design.pilot) {
            const PilotStationDesign& value = *part.design.pilot;
            if (part.kind != PartKind::Pilot || !std::isfinite(value.seatHeightM) || value.seatHeightM < 0.2 || value.seatHeightM > 1.8
                || !std::isfinite(value.pedalZM) || std::abs(value.pedalZM) > 3
                || !std::isfinite(value.pedalHeightM) || value.pedalHeightM < 0.1 || value.pedalHeightM > 1.8
                || !std::isfinite(value.crankZM) || std::abs(value.crankZM) > 3
                || !std::isfinite(value.crankHeightM) || value.crankHeightM < 0.1 || value.crankHeightM > 1.8)
                errors.push_back({part.id, "invalid pilot station design"});
        }
        if (part.design.tailSupport) {
            const TailSupportDesign& value = *part.design.tailSupport;
            if ((part.kind != PartKind::HTail && part.kind != PartKind::VTail)
                || !validChoice(value.mounting, {"cantilever", "strut", "wire"})
                || value.supportCount < 0 || value.supportCount > 8
                || !std::isfinite(value.supportDiaMm) || value.supportDiaMm < 2 || value.supportDiaMm > 80)
                errors.push_back({part.id, "invalid tail support design"});
        }
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

    // Pairは機体中心面に対する全体ミラー。Pair祖先の下で再度Pairにすると
    // 同一world変換が重複し、深い木では配置数が指数的に増えるため拒否する。
    for (const auto& entry : parts_) {
        const Part& part = entry.second;
        if (part.mount.mirror != MirrorMode::Pair) continue;
        std::set<std::string> ancestors;
        std::string parentId = part.mount.parentId;
        while (!parentId.empty() && ancestors.insert(parentId).second) {
            const Part* parent = find(parentId);
            if (!parent) break;
            if (parent->mount.mirror == MirrorMode::Pair) {
                errors.push_back({part.id, "nested mirror pair is not supported"});
                break;
            }
            parentId = parent->mount.parentId;
        }
    }
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

MassBreakdown aggregateMass(const AirframeGraph& graph) {
    MassBreakdown result;
    const auto placed = graph.resolve();
    if (placed.empty()) return result;

#ifndef NDEBUG
    std::set<int> analysisItems;
    for (const auto& entry : graph.parts()) for (const auto& node : entry.second.massNodes) {
        if (node.analysisItem >= 0) {
            const bool unique = analysisItems.insert(node.analysisItem).second;
            assert(unique && "Analysis::items index must occur in exactly one MassNode");
        }
    }
#endif

    struct Contribution {
        double kg;
        glm::dvec3 cg;
        glm::dmat3 inertiaAtCg;
    };
    std::vector<Contribution> contributions;
    glm::dmat3 mirror(1.0);
    mirror[0][0] = -1.0;
    for (const auto& instance : placed) {
        if (instance.part->kind == PartKind::Unknown) continue;
        const glm::dmat3 rotation(instance.world);
        for (const auto& node : instance.part->massNodes) {
            const glm::dvec3 localCg = instance.mirrored ? mirror * node.cgLocal : node.cgLocal;
            const glm::dmat3 localI = instance.mirrored ? mirror * node.I0 * mirror : node.I0;
            const glm::dvec3 worldCg(instance.world * glm::dvec4(localCg, 1.0));
            const glm::dmat3 worldI = rotation * localI * glm::transpose(rotation);
            contributions.push_back({node.kg, worldCg, worldI});
            result.items.push_back({instance.part->id, node.kg, worldCg});
            result.totalKg += node.kg;
            result.cg += node.kg * worldCg;
        }
    }
    if (result.totalKg <= 0.0) {
        result.cg = glm::dvec3(0.0);
        return result;
    }
    result.cg /= result.totalKg;
    const glm::dmat3 identity(1.0);
    for (const auto& contribution : contributions) {
        const glm::dvec3 d = contribution.cg - result.cg;
        result.I += contribution.inertiaAtCg
            + contribution.kg * (glm::dot(d, d) * identity - glm::outerProduct(d, d));
    }
    return result;
}

AeroLayoutProperties aggregateAeroLayout(const AirframeGraph& graph) {
    AeroLayoutProperties result;
    const auto placed = graph.resolve();
    if (placed.empty()) return result;

    bool haveWing = false;
    double hWeightedZ = 0.0, hFallbackZ = 0.0;
    double vWeightedZ = 0.0, vFallbackZ = 0.0;
    int hCount = 0, vCount = 0;
    for (const auto& instance : placed) {
        const glm::dvec3 origin(instance.world[3]);
        const glm::dmat3 rotation(instance.world);
        if (instance.part->id == "wing.main" && !instance.mirrored && !haveWing) {
            const glm::dvec3 chord = glm::normalize(rotation * glm::dvec3(0.0, 0.0, 1.0));
            result.wingLEZ = origin.z;
            result.wingIncidenceDeg = glm::degrees(std::atan2(-chord.y, chord.z));
            haveWing = finiteVec(origin) && finiteVec(chord)
                && std::isfinite(result.wingIncidenceDeg);
        } else if (instance.part->id == "tail.h") {
            const glm::dvec3 normal = glm::normalize(rotation * glm::dvec3(0.0, 1.0, 0.0));
            const double projection = std::abs(normal.y);
            if (!finiteVec(origin) || !finiteVec(normal) || !std::isfinite(projection)) return {};
            result.hAreaScale += projection;
            hWeightedZ += projection * origin.z;
            hFallbackZ += origin.z;
            ++hCount;
        } else if (instance.part->id == "tail.v") {
            const glm::dvec3 normal = glm::normalize(rotation * glm::dvec3(1.0, 0.0, 0.0));
            const double projection = std::abs(normal.x);
            if (!finiteVec(origin) || !finiteVec(normal) || !std::isfinite(projection)) return {};
            result.vAreaScale += projection;
            vWeightedZ += projection * origin.z;
            vFallbackZ += origin.z;
            ++vCount;
        }
    }
    if (!haveWing || hCount == 0 || vCount == 0) return result;
    result.hTailZ = result.hAreaScale > 1e-9 ? hWeightedZ / result.hAreaScale
                                             : hFallbackZ / hCount;
    result.vTailZ = result.vAreaScale > 1e-9 ? vWeightedZ / result.vAreaScale
                                             : vFallbackZ / vCount;
    result.valid = std::isfinite(result.wingLEZ) && std::isfinite(result.hTailZ)
        && std::isfinite(result.vTailZ) && std::isfinite(result.hAreaScale)
        && std::isfinite(result.vAreaScale);
    return result;
}

DesignPhysicsProperties aggregateDesignPhysics(const AirframeGraph& graph) {
    DesignPhysicsProperties result;
    const auto placed = graph.resolve();
    if (placed.empty()) return result;

    const Part* wing = graph.find("wing.main");
    if (!wing || !wing->design.spar) return result;
    result.spar = *wing->design.spar;

    if (const Part* fairing = graph.find("fairing")) {
        const FairingDesign value = fairing->design.fairing.value_or(FairingDesign{});
        const FairingDesign base;
        const double frontal = value.widthM * value.heightM;
        const double baseFrontal = base.widthM * base.heightM;
        const double fineness = value.lengthM / std::sqrt(frontal);
        const double baseFineness = base.lengthM / std::sqrt(baseFrontal);
        const double noseQuality = std::exp(-3.0 * std::abs(value.noseRatio - base.noseRatio));
        const double tailQuality = std::exp(-1.5 * std::abs(value.tailRatio - base.tailRatio));
        const double shape = std::pow(fineness / baseFineness, 0.45)
                           * std::pow(baseFrontal / frontal, 0.10)
                           * noseQuality * tailQuality;
        result.fairingCdReduction = 0.0012 * std::clamp(shape, 0.25, 1.50);
    }

    for (const auto& instance : placed) {
        if (instance.part->kind != PartKind::HTail && instance.part->kind != PartKind::VTail) continue;
        if (!instance.part->design.tailSupport) continue;
        const TailSupportDesign& support = *instance.part->design.tailSupport;
        if (support.mounting == "cantilever" || support.supportCount <= 0) continue;
        const double diameterM = support.supportDiaMm / 1000.0;
        const double lengthM = instance.part->kind == PartKind::HTail ? 0.80 : 0.65;
        const double cd = support.mounting == "wire" ? 1.10 : 0.65;
        result.supportDragAreaM2 += support.supportCount * diameterM * lengthM * cd;
    }
    result.valid = std::isfinite(result.fairingCdReduction)
        && std::isfinite(result.supportDragAreaM2)
        && result.fairingCdReduction >= 0.0 && result.supportDragAreaM2 >= 0.0;
    return result;
}

DesignPhysicsProperties aggregateDesignPhysics(const AirframeGraph& graph,
                                               const AircraftParams& st) {
    DesignPhysicsProperties result = aggregateDesignPhysics(graph);
    if (!result.valid) return result;

    result.compositionValid = true;
    const Part* pilot = graph.find("pilot");
    result.hasPilot = pilot != nullptr;
    result.pilotPowerFactor = result.hasPilot ? 1.0 : 0.0;
    result.pilotEnergyFactor = 1.0;
    if (pilot && pilot->design.pilot) {
        PilotStationDesign base;
        if (st.posture == "upright") base = {0.68, -0.30, 0.80, -0.28, 0.42};
        else if (st.posture == "semi") base = {0.62, -0.50, 0.85, -0.80, 0.58};
        else base = {0.60, -0.55, 0.92, -0.95, 0.68};
        const PilotStationDesign& value = *pilot->design.pilot;
        const auto reach = [](const PilotStationDesign& p, bool crank) {
            const double z = crank ? p.crankZM : p.pedalZM;
            const double y = crank ? p.crankHeightM : p.pedalHeightM;
            return std::hypot(z, y - p.seatHeightM);
        };
        const double pedalError = (reach(value, false) - reach(base, false))
                                / std::max(0.25, reach(base, false));
        const double crankError = (reach(value, true) - reach(base, true))
                                / std::max(0.25, reach(base, true));
        const double linkage = std::hypot(value.pedalZM - value.crankZM,
                                          value.pedalHeightM - value.crankHeightM);
        const double baseLinkage = std::hypot(base.pedalZM - base.crankZM,
                                              base.pedalHeightM - base.crankHeightM);
        const double linkageError = (linkage - baseLinkage) / std::max(0.20, baseLinkage);
        const double strain = 0.45 * pedalError * pedalError
                            + 0.35 * crankError * crankError
                            + 0.20 * linkageError * linkageError;
        result.pilotPowerFactor = std::clamp(std::exp(-0.55 * strain), 0.60, 1.0);
        result.pilotEnergyFactor = std::clamp(1.0 + 0.45 * strain, 1.0, 1.60);
    }

    int gearCount = 0;
    bool front = false, rear = false, main = false, tail = false;
    for (const auto& entry : graph.parts()) {
        if (entry.second.kind != PartKind::Gear) continue;
        ++gearCount;
        front = front || entry.first.find("front") != std::string::npos;
        rear = rear || entry.first.find("rear") != std::string::npos;
        main = main || entry.first.find("main") != std::string::npos;
        tail = tail || entry.first.find("tail") != std::string::npos;
    }
    result.hasGear = gearCount > 0;
    if (main && tail) result.gearType = "mono";
    else if (front && main) result.gearType = "tri";
    else result.gearType = "tandem";

    double boomWeightedZ = 0.0;
    for (const auto& instance : graph.resolve()) {
        if (instance.part->kind != PartKind::BoomWing) continue;
        const glm::dvec3 origin(instance.world[3]);
        const glm::dmat3 rotation(instance.world);
        const glm::dvec3 normal = glm::normalize(rotation * glm::dvec3(0.0, 1.0, 0.0));
        if (!finiteVec(origin) || !finiteVec(normal)) { result.compositionValid = false; return result; }
        // 各instanceは半翼。レンダラの0.65テーパと同じ平均翼弦率0.825を用いる。
        const double area = 0.5 * st.boomWingSpan * st.boomWingChord * 0.825
                          * std::abs(normal.y);
        result.boomWingAreaM2 += area;
        boomWeightedZ += area * origin.z;
    }
    result.boomWingZ = result.boomWingAreaM2 > 1e-12
        ? boomWeightedZ / result.boomWingAreaM2 : 0.0;
    return result;
}

AirframeGraph buildDefaultLayout(const AircraftParams& st, const Analysis& an) {
    AirframeGraph graph;
    const double hWing = st.posture == "upright" ? 2.4 : 2.0;
    const double hBoom = hWing - 0.15;
    const double propH = st.propConfig == "pylon" ? hBoom + 0.9 : hBoom;
    const double zBeamRear = st.seatX + 0.55;
    const double boomWingZ = zBeamRear + (an.fusLen - zBeamRear) * st.boomWingPos;
    const double boomWingKg = st.boomWing == "none" ? 0.0
        : (st.boomWing == "LR" ? 2.0 : 1.0) * st.boomWingSpan * st.boomWingChord * 0.55 + 0.12;

    auto massAt = [&](int item, const glm::dmat4& world) {
        MassNode node;
        node.kg = an.items.at((std::size_t)item).w;
        const glm::dvec3 origin(world[3]);
        const glm::dvec3 target{origin.x, origin.y, an.items.at((std::size_t)item).x};
        node.cgLocal = glm::dvec3(glm::inverse(world) * glm::dvec4(target, 1.0));
        node.analysisItem = item;
        return node;
    };
    auto hp = [](std::string id, glm::dvec3 pos, glm::dvec3 rot = glm::dvec3(0.0)) {
        return Hardpoint{std::move(id), {pos, rot}};
    };
    auto child = [](std::string id, PartKind kind, std::string hardpoint) {
        Part part;
        part.id = std::move(id);
        part.kind = kind;
        part.mount.parentId = "fuselage";
        part.mount.hardpointId = std::move(hardpoint);
        return part;
    };

    Part fuselage;
    fuselage.id = "fuselage";
    fuselage.kind = PartKind::Fuselage;
    fuselage.hardpoints = {
        hp("hp.wing", {0.0, hWing, st.wingX}, {st.incidence, 0.0, 0.0}),
        hp("hp.tail.h", {0.0, hBoom + 0.02, an.xHT}),
        hp("hp.tail.v", {0.0, hBoom, an.xVT}),
        hp("hp.prop", {0.0, propH, an.xProp}),
        hp("hp.cockpit", {0.0, 0.0, st.seatX}),
        hp("hp.frame.front.top", {0.0, hBoom, st.seatX - 0.55}),
        hp("hp.frame.front.bottom", {0.0, 0.36, st.seatX - 0.55}),
        hp("hp.frame.rear.top", {0.0, hBoom, zBeamRear}),
        hp("hp.frame.rear.bottom", {0.0, 0.36, zBeamRear}),
        hp("hp.tail.end", {0.0, hBoom, an.fusLen}),
        hp("hp.ui.tailbeam", {0.0, hBoom, (zBeamRear + an.fusLen) * 0.5}),
        hp("hp.gear.front", {0.0, st.gear == "tandem" ? 0.14 : 0.13, st.seatX - 0.55}),
        hp("hp.gear.rear", {0.0, 0.15, st.seatX + 0.55}),
        hp("hp.gear.main.tri", {0.55, 0.15, st.seatX + 0.45}),
        hp("hp.gear.main.mono", {0.0, 0.16, st.seatX + 0.05}),
        hp("hp.gear.tail", {0.0, 0.07, an.fusLen - 0.3}),
        hp("hp.boomwing", {0.0, hBoom + 0.02, boomWingZ})
    };
    const glm::dmat4 identity(1.0);
    for (int item : {3, 5, 9}) fuselage.massNodes.push_back(massAt(item, identity));
    if (st.gear == "none") fuselage.massNodes.push_back(massAt(7, identity));
    if (!st.fairing) fuselage.massNodes.push_back(massAt(8, identity));
    if (!graph.addPart(std::move(fuselage))) return {};
    const Part* root = graph.find("fuselage");
    if (!root) return {};

    auto addWithMass = [&](Part part, int item, const Transform3& hardpointTransform) {
        const glm::dmat4 world = transformMatrix(hardpointTransform) * transformMatrix(part.mount.offset);
        part.massNodes.push_back(massAt(item, world));
        return graph.addPart(std::move(part));
    };

    Part wing = child("wing.main", PartKind::Wing, "hp.wing");
    wing.design.spar = SparDesign{1, 0.30, st.rootDia, st.tipDia,
                                  std::max(0, st.segments - 1), "tube"};
    // Phase 0B: wing-mounted equipment (for example twin wingtip fins) uses a
    // stable hardpoint on the positive-X tip; Pair mirrors the complete child.
    wing.hardpoints.push_back(hp("hp.tip", glm::dvec3{
        st.span * 0.5,
        std::tan(st.dihedral * 3.14159265358979323846 / 180.0) * st.span * 0.5,
        st.tipChord * 0.4}));
    {
        const glm::dmat4 world = transformMatrix(root->hardpoints[0].t);
        MassNode wingMass = massAt(0, world);
        wingMass.kg = std::max(0.0, wingMass.kg - boomWingKg);
        wing.massNodes.push_back(wingMass);
        if (!graph.addPart(std::move(wing))) return {};
    }
    Part htail = child("tail.h", PartKind::HTail, "hp.tail.h");
    htail.design.tailSupport = TailSupportDesign{};
    htail.hardpoints.push_back(hp("hp.tip", {st.hSpan * 0.5, 0.0, st.hChord * 0.4}));
    if (!addWithMass(std::move(htail), 1, root->hardpoints[1].t)) return {};
    Part vtail = child("tail.v", PartKind::VTail, "hp.tail.v");
    vtail.design.tailSupport = TailSupportDesign{};
    if (!addWithMass(std::move(vtail), 2, root->hardpoints[2].t)) return {};
    Part prop = child("prop.main", PartKind::Prop, "hp.prop");
    if (!addWithMass(std::move(prop), 6, root->hardpoints[3].t)) return {};
    Part cockpit = child("cockpit", PartKind::Cockpit, "hp.cockpit");
    if (!addWithMass(std::move(cockpit), 4, root->hardpoints[4].t)) return {};

    Part pilot = child("pilot", PartKind::Pilot, "hp.cockpit");
    PilotStationDesign pilotStation;
    if (st.posture == "upright") pilotStation = {0.68, -0.30, 0.80, -0.28, 0.42};
    else if (st.posture == "semi") pilotStation = {0.62, -0.50, 0.85, -0.80, 0.58};
    else pilotStation = {0.60, -0.55, 0.92, -0.95, 0.68};
    pilot.design.pilot = pilotStation;
    pilot.mount.offset.pos.z = an.pilotCGx - st.seatX;
    if (!addWithMass(std::move(pilot), 10, root->hardpoints[4].t)) return {};

    if (st.fairing) {
        Part fairing = child("fairing", PartKind::Fairing, "hp.cockpit");
        fairing.design.fairing = FairingDesign{};
        if (!addWithMass(std::move(fairing), 8, root->hardpoints[4].t)) return {};
    }

    auto addGear = [&](std::string id, std::string hardpoint, MirrorMode mirror, bool carriesMass) {
        Part gear = child(std::move(id), PartKind::Gear, hardpoint);
        gear.mount.mirror = mirror;
        const Part* root = graph.find("fuselage");
        const auto it = std::find_if(root->hardpoints.begin(), root->hardpoints.end(),
            [&](const Hardpoint& value) { return value.id == hardpoint; });
        if (carriesMass) gear.massNodes.push_back(massAt(7, transformMatrix(it->t)));
        return graph.addPart(std::move(gear));
    };
    if (st.gear == "tri") {
        if (!addGear("gear.front", "hp.gear.front", MirrorMode::None, true)) return {};
        if (!addGear("gear.main", "hp.gear.main.tri", MirrorMode::Pair, false)) return {};
    } else if (st.gear == "tandem") {
        if (!addGear("gear.front", "hp.gear.front", MirrorMode::None, true)) return {};
        if (!addGear("gear.rear", "hp.gear.rear", MirrorMode::None, false)) return {};
    } else if (st.gear == "mono") {
        if (!addGear("gear.main", "hp.gear.main.mono", MirrorMode::None, true)) return {};
        if (!addGear("gear.tail", "hp.gear.tail", MirrorMode::None, false)) return {};
    }

    if (st.boomWing != "none") {
        Part boomWing = child("boomwing", PartKind::BoomWing, "hp.boomwing");
        boomWing.mount.mirror = st.boomWing == "LR" ? MirrorMode::Pair : MirrorMode::None;
        const auto hpIt = std::find_if(root->hardpoints.begin(), root->hardpoints.end(),
            [](const Hardpoint& value) { return value.id == "hp.boomwing"; });
        if (hpIt == root->hardpoints.end()) return {};
        const glm::dmat4 world = transformMatrix(hpIt->t);
        MassNode mass = massAt(0, world);
        mass.kg = boomWing.mount.mirror == MirrorMode::Pair ? boomWingKg * 0.5 : boomWingKg;
        mass.analysisItem = -1;
        boomWing.massNodes.push_back(mass);
        if (!graph.addPart(std::move(boomWing))) return {};
    }
    return graph;
}

} // namespace bm
