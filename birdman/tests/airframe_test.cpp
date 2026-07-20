#include "core/Airframe.hpp"
#include "core/Aircraft.hpp"
#include "core/DesignIO.hpp"
#include "core/MiniJson.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string& name) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << name << '\n';
    }
}

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }
bool near(const glm::dvec3& a, const glm::dvec3& b, double eps = 1e-9) {
    return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps);
}

bool near(const glm::dmat4& a, const glm::dmat4& b, double eps = 1e-9) {
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
            if (!near(a[column][row], b[column][row], eps)) return false;
    return true;
}

glm::dvec3 point(const glm::dmat4& m, glm::dvec3 p = {0.0, 0.0, 0.0}) {
    return glm::dvec3(m * glm::dvec4(p, 1.0));
}

bm::Part rootPart() {
    bm::Part root;
    root.id = "body";
    root.kind = bm::PartKind::Fuselage;
    root.hardpoints.push_back({"tail", {{0.0, 0.0, 2.0}, {0.0, 0.0, 0.0}}});
    root.hardpoints.push_back({"wing", {{1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}});
    return root;
}

bm::Part mounted(std::string id, bm::PartKind kind, std::string parent, std::string hardpoint) {
    bm::Part part;
    part.id = std::move(id);
    part.kind = kind;
    part.mount.parentId = std::move(parent);
    part.mount.hardpointId = std::move(hardpoint);
    return part;
}

void miniJsonTests() {
    bm::json::Value value;
    bm::json::ParseError error;
    check(bm::json::parse(R"({"z":true,"a":[1,-2.5e2,"\u65e5\ud83d\ude80",null]})", value, &error), "JSON valid document");
    check(value.isObject() && value.find("a") && value.find("a")->array.size() == 4, "JSON object/array shape");
    check(bm::json::stringify(value).rfind("{\"a\":", 0) == 0, "JSON deterministic object ordering");

    const std::vector<std::string> invalid = {
        "[1,", R"({"a":1,"a":2})", "01", "NaN", "1e9999", R"("\ud800")", "true false"
    };
    for (std::size_t i = 0; i < invalid.size(); ++i)
        check(!bm::json::parse(invalid[i], value, &error), "JSON invalid input " + std::to_string(i));

    glm::dvec3 v{9.0};
    bm::json::parse("[1,2,3]", value);
    check(bm::vec3FromJson(value, v) && near(v, {1.0, 2.0, 3.0}), "vec3 JSON valid");
    bm::json::parse("[1,2]", value);
    check(!bm::vec3FromJson(value, v) && near(v, {1.0, 2.0, 3.0}), "vec3 JSON failure leaves value unchanged");

    std::string tooDeep;
    for (int i = 0; i < 70; ++i) tooDeep.push_back('[');
    tooDeep += '0';
    for (int i = 0; i < 70; ++i) tooDeep.push_back(']');
    check(!bm::json::parse(tooDeep, value, &error), "JSON nesting depth is bounded");
}

void rotationTests() {
    bm::Transform3 yaw{{0.0, 0.0, 0.0}, {0.0, 90.0, 0.0}};
    bm::Transform3 pitch{{0.0, 0.0, 0.0}, {90.0, 0.0, 0.0}};
    bm::Transform3 roll{{0.0, 0.0, 0.0}, {0.0, 0.0, 90.0}};
    check(near(point(bm::transformMatrix(yaw), {0.0, 0.0, 1.0}), {1.0, 0.0, 0.0}), "yaw convention");
    check(near(point(bm::transformMatrix(pitch), {0.0, 1.0, 0.0}), {0.0, 0.0, 1.0}), "pitch convention");
    check(near(point(bm::transformMatrix(roll), {1.0, 0.0, 0.0}), {0.0, 1.0, 0.0}), "roll convention");

    bm::Transform3 combined{{3.0, -2.0, 5.0}, {20.0, 35.0, -15.0}};
    const glm::dvec3 source{0.7, -1.3, 2.1};
    glm::dvec3 expected = source;
    const double rz = glm::radians(combined.rotDeg.z);
    expected = {std::cos(rz) * expected.x - std::sin(rz) * expected.y,
                std::sin(rz) * expected.x + std::cos(rz) * expected.y, expected.z};
    const double rx = glm::radians(combined.rotDeg.x);
    expected = {expected.x, std::cos(rx) * expected.y - std::sin(rx) * expected.z,
                std::sin(rx) * expected.y + std::cos(rx) * expected.z};
    const double ry = glm::radians(combined.rotDeg.y);
    expected = {std::cos(ry) * expected.x + std::sin(ry) * expected.z, expected.y,
                -std::sin(ry) * expected.x + std::cos(ry) * expected.z};
    expected += combined.pos;
    check(near(point(bm::transformMatrix(combined), source), expected), "combined Ry Rx Rz order");
}

void graphResolveTests() {
    bm::AirframeGraph graph;
    check(graph.addPart(rootPart()), "add root");
    bm::Part boom = mounted("boom", bm::PartKind::BoomWing, "body", "tail");
    boom.mount.offset.pos = {0.0, 1.0, 0.5};
    boom.hardpoints.push_back({"tip", {{0.0, 0.0, 3.0}, {0.0, 0.0, 0.0}}});
    check(graph.addPart(boom), "add boom");
    bm::Part tail = mounted("tail", bm::PartKind::HTail, "boom", "tip");
    tail.mount.offset.pos = {0.0, 0.25, 0.0};
    check(graph.addPart(tail), "add nested tail");
    const auto resolved = graph.resolve();
    check(resolved.size() == 3, "resolve nested count");
    check(resolved.size() == 3 && resolved[0].part->id == "body"
        && resolved[1].part->id == "boom" && resolved[2].part->id == "tail",
        "resolve returns topological order");
    for (const auto& item : resolved) {
        if (item.part->id == "tail") check(near(point(item.world), {0.0, 1.25, 5.5}), "nested transform position");
    }

    bm::AirframeGraph mirrored;
    check(mirrored.addPart(rootPart()), "mirror add root");
    bm::Part wing = mounted("wing", bm::PartKind::Wing, "body", "wing");
    wing.mount.offset.pos = {2.0, 0.0, 0.0};
    wing.mount.offset.rotDeg = {0.0, 10.0, 20.0};
    wing.mount.mirror = bm::MirrorMode::Pair;
    wing.hardpoints.push_back({"tip", {{2.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}});
    check(mirrored.addPart(wing), "add paired wing");
    bm::Part pod = mounted("pod", bm::PartKind::Fairing, "wing", "tip");
    check(mirrored.addPart(pod), "add child of paired wing");
    const auto pairResolved = mirrored.resolve();
    int wingCount = 0, podCount = 0, mirroredCount = 0;
    std::vector<glm::dmat4> wingMatrices, podMatrices;
    for (const auto& item : pairResolved) {
        if (item.part->id == "wing") { ++wingCount; wingMatrices.push_back(item.world); }
        if (item.part->id == "pod") { ++podCount; podMatrices.push_back(item.world); }
        if (item.mirrored) ++mirroredCount;
    }
    check(wingCount == 2 && podCount == 2, "pair and child multiplicity");
    check(mirroredCount == 2, "mirror flag propagation");
    check(wingMatrices.size() == 2 && near(wingMatrices[1], bm::mirrorMatrix(wingMatrices[0])), "S M S mirror transform and axes");
    check(podMatrices.size() == 2 && near(podMatrices[1], bm::mirrorMatrix(podMatrices[0])), "mirrored child transform propagation");
}

void validationTests() {
    bm::Part root = rootPart();
    bm::Part duplicate = root;
    auto graph = bm::AirframeGraph::fromUntrusted({root, duplicate});
    check(!graph.validate().empty(), "duplicate part id validation");

    bm::Part missingParent = mounted("wing", bm::PartKind::Wing, "absent", "hp");
    graph = bm::AirframeGraph::fromUntrusted({root, missingParent});
    check(!graph.validate().empty() && graph.resolve().empty(), "missing parent validation");

    bm::Part missingHp = mounted("wing", bm::PartKind::Wing, "body", "absent");
    graph = bm::AirframeGraph::fromUntrusted({root, missingHp});
    check(!graph.validate().empty(), "missing hardpoint validation");

    bm::Part a = mounted("a", bm::PartKind::BoomWing, "c", "x"); a.hardpoints.push_back({"x", {}});
    bm::Part b = mounted("b", bm::PartKind::BoomWing, "a", "x"); b.hardpoints.push_back({"x", {}});
    bm::Part c = mounted("c", bm::PartKind::BoomWing, "b", "x"); c.hardpoints.push_back({"x", {}});
    graph = bm::AirframeGraph::fromUntrusted({root, a, b, c});
    const auto cycleErrors = graph.validate();
    int cycleIds = 0;
    for (const auto& error : cycleErrors) if (error.message == "cycle includes part") ++cycleIds;
    check(cycleIds == 3, "cycle validation reports every involved id");

    bm::Part self = mounted("self", bm::PartKind::BoomWing, "self", "x");
    self.hardpoints.push_back({"x", {}});
    graph = bm::AirframeGraph::fromUntrusted({root, self});
    bool selfReported = false;
    for (const auto& error : graph.validate())
        selfReported = selfReported || (error.partId == "self" && error.message == "cycle includes part");
    check(selfReported, "self reference validation");

    bm::Part secondRoot;
    secondRoot.id = "body2";
    secondRoot.kind = bm::PartKind::Fuselage;
    graph = bm::AirframeGraph::fromUntrusted({root, secondRoot});
    check(!graph.validate().empty(), "multiple root validation");

    bm::Part bad = mounted("bad", bm::PartKind::Wing, "body", "wing");
    bad.mount.offset.pos.x = std::numeric_limits<double>::infinity();
    graph = bm::AirframeGraph::fromUntrusted({root, bad});
    check(!graph.validate().empty(), "infinite coordinate validation");
    bad.mount.offset.pos.x = 1000.01;
    graph = bm::AirframeGraph::fromUntrusted({root, bad});
    check(!graph.validate().empty(), "coordinate range validation");
    bad.mount.offset.pos.x = std::numeric_limits<double>::quiet_NaN();
    graph = bm::AirframeGraph::fromUntrusted({root, bad});
    check(!graph.validate().empty(), "NaN coordinate validation");

    check(bm::partKindFromString("new-future-kind") == bm::PartKind::Unknown, "unknown part kind fallback");
    check(bm::mirrorModeFromString("new-future-mode") == bm::MirrorMode::None, "unknown mirror mode fallback");
}

void mutationTests() {
    bm::AirframeGraph graph;
    check(graph.addPart(rootPart()), "mutation root add");
    const std::size_t before = graph.size();
    bm::Part invalid = mounted("bad", bm::PartKind::Wing, "body", "missing");
    check(!graph.addPart(invalid) && graph.size() == before, "invalid add is atomic");

    bm::Part wing = mounted("wing", bm::PartKind::Wing, "body", "wing");
    wing.hardpoints.push_back({"child", {}});
    check(graph.addPart(wing), "mutation wing add");
    check(graph.addPart(mounted("pod", bm::PartKind::Fairing, "wing", "child")), "mutation pod add");
    check(graph.removeSubtree("wing") && graph.size() == 1 && !graph.find("pod"), "remove subtree");
    check(!graph.removeSubtree("body") && graph.size() == 1, "reject root removal");
}

void atomicEditingTests() {
    bm::Part root = rootPart();
    root.hardpoints.push_back({"alternate", {{4.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}});
    bm::Part wing = mounted("wing", bm::PartKind::Wing, "body", "wing");
    wing.hardpoints.push_back({"child", {{0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}}});
    bm::Part pod = mounted("pod", bm::PartKind::Fairing, "wing", "child");
    pod.hardpoints.push_back({"cycle", {}});
    bm::AirframeGraph graph = bm::AirframeGraph::fromUntrusted({root, wing, pod});
    check(graph.validate().empty(), "atomic edit fixture validates");

    bm::Mount moved = graph.find("wing")->mount;
    moved.offset.pos = {1.5, 0.25, -0.5};
    moved.offset.rotDeg = {370.0, -725.0, 90.0};
    moved.mirror = bm::MirrorMode::Pair;
    check(graph.setMount("wing", moved), "atomic mount transform and mirror edit");
    const bm::Part* movedWing = graph.find("wing");
    check(movedWing && near(movedWing->mount.offset.rotDeg, {10.0, -5.0, 90.0}),
          "edited mount angles are normalized");
    int wingInstances = 0, podInstances = 0;
    for (const auto& item : graph.resolve()) {
        if (item.part->id == "wing") ++wingInstances;
        if (item.part->id == "pod") ++podInstances;
    }
    check(wingInstances == 2 && podInstances == 2, "mount mirror edit propagates to children");

    const bm::Mount beforeFailure = graph.find("wing")->mount;
    std::string error;
    bm::Mount invalid = beforeFailure;
    invalid.parentId = "pod";
    invalid.hardpointId = "cycle";
    check(!graph.setMount("wing", invalid, &error) && !error.empty(), "cycle edit is rejected");
    check(graph.find("wing")->mount.parentId == beforeFailure.parentId
          && graph.find("wing")->mount.hardpointId == beforeFailure.hardpointId,
          "cycle rejection preserves original graph");

    bm::Transform3 alternate;
    alternate.pos = {5.0, 2.0, 1.0};
    check(graph.setHardpointTransform("body", "alternate", alternate), "hardpoint transform edit");
    bm::Mount reattached = graph.find("pod")->mount;
    reattached.parentId = "body";
    reattached.hardpointId = "alternate";
    check(graph.setMount("pod", reattached), "atomic reattach to another hardpoint");
    const auto reattachedPlaced = graph.resolve();
    const bm::AirframeGraph::Placed* podPlacement = nullptr;
    for (const auto& item : reattachedPlaced)
        if (!item.mirrored && item.part->id == "pod") podPlacement = &item;
    check(podPlacement && near(point(podPlacement->world), {5.0, 2.0, 1.0}),
          "reattached part follows edited hardpoint");

    invalid = beforeFailure;
    invalid.hardpointId = "missing";
    check(!graph.setMount("wing", invalid), "missing hardpoint edit is rejected");
    check(!graph.setMount("body", {}), "root mount edit is rejected");

    bm::Part renamed = *graph.find("wing");
    renamed.id = "renamed";
    check(!graph.replacePart("wing", renamed), "stable part id cannot be changed");

    bm::Part invalidDesign = *graph.find("wing");
    invalidDesign.design.spar = bm::SparDesign{};
    invalidDesign.design.spar->count = 0;
    check(!graph.replacePart("wing", invalidDesign), "invalid part-specific design is rejected");
    check(!graph.find("wing")->design.spar.has_value(),
          "invalid part-specific design preserves original graph");

    bm::Part brokenRoot = *graph.find("body");
    brokenRoot.hardpoints.erase(brokenRoot.hardpoints.begin() + 2);
    check(!graph.replacePart("body", brokenRoot), "hardpoint used by a child cannot be removed");
    check(graph.find("body")->hardpoints.size() == 3, "failed parent edit is rolled back");

    bm::Transform3 invalidTransform;
    invalidTransform.pos.x = std::numeric_limits<double>::quiet_NaN();
    check(!graph.setHardpointTransform("body", "alternate", invalidTransform),
          "non-finite hardpoint edit is rejected");
    const bm::Hardpoint* preservedAlternate = nullptr;
    for (const auto& hp : graph.find("body")->hardpoints)
        if (hp.id == "alternate") preservedAlternate = &hp;
    check(preservedAlternate && near(preservedAlternate->t.pos, {5.0, 2.0, 1.0}),
          "invalid hardpoint edit preserves original transform");
    check(!graph.setHardpointTransform("body", "missing", {}), "unknown hardpoint edit is rejected");
    check(!graph.replacePart("missing", {}), "unknown part replacement is rejected");

    bm::AirframeGraph nestedPair;
    bm::Part pairRoot = rootPart();
    check(nestedPair.addPart(pairRoot), "nested pair root");
    bm::Part pairParent = mounted("pair-parent", bm::PartKind::Wing, "body", "wing");
    pairParent.mount.mirror = bm::MirrorMode::Pair;
    pairParent.hardpoints.push_back({"pair-child", {}});
    check(nestedPair.addPart(pairParent), "nested pair parent");
    bm::Part pairChild = mounted("pair-child", bm::PartKind::Fairing, "pair-parent", "pair-child");
    pairChild.mount.mirror = bm::MirrorMode::Pair;
    check(!nestedPair.addPart(pairChild), "nested Pair insertion is rejected");
    check(nestedPair.validate().empty() && nestedPair.resolve().size() == 3,
          "nested Pair rejection preserves the valid graph");

    bm::AirframeGraph untrustedNested = bm::AirframeGraph::fromUntrusted({pairRoot, pairParent, pairChild});
    check(!untrustedNested.validate().empty() && untrustedNested.resolve().empty(),
          "untrusted nested Pair is rejected before expansion");

    bm::Mount nestedMount = graph.find("pod")->mount;
    nestedMount.parentId = "wing";
    nestedMount.hardpointId = "child";
    nestedMount.mirror = bm::MirrorMode::Pair;
    check(!graph.setMount("pod", nestedMount), "atomic edit rejects nested Pair");
}

const bm::AirframeGraph::Placed* normalPlacement(
    const std::vector<bm::AirframeGraph::Placed>& placed, const std::string& id) {
    for (const auto& item : placed)
        if (!item.mirrored && item.part->id == id) return &item;
    return nullptr;
}

void defaultLayoutTests() {
    const std::vector<std::string> postures{"upright", "semi", "recumbent"};
    const std::vector<std::string> props{"tractor", "pylon", "midboom", "pusher"};
    const std::vector<std::string> gears{"none", "tandem", "tri", "mono"};
    for (const auto& posture : postures) for (const auto& propConfig : props) for (const auto& gear : gears) {
        bm::AircraftParams st;
        st.posture = posture;
        st.propConfig = propConfig;
        st.gear = gear;
        st.fairing = gear == "tri";
        const bm::Analysis an = bm::analyze(st);
        const bm::AirframeGraph graph = bm::buildDefaultLayout(st, an);
        const auto errors = graph.validate();
        const auto placed = graph.resolve();
        const std::string variant = posture + "/" + propConfig + "/" + gear;
        check(errors.empty(), "default layout validates: " + variant);

        const auto* wing = normalPlacement(placed, "wing.main");
        const auto* htail = normalPlacement(placed, "tail.h");
        const auto* vtail = normalPlacement(placed, "tail.v");
        const auto* prop = normalPlacement(placed, "prop.main");
        const auto* cockpit = normalPlacement(placed, "cockpit");
        const double hWing = posture == "upright" ? 2.4 : 2.0;
        const double hBoom = hWing - 0.15;
        const double propH = propConfig == "pylon" ? hBoom + 0.9 : hBoom;
        check(wing && near(point(wing->world), {0.0, hWing, st.wingX}, 1e-12), "default wing origin: " + variant);
        check(htail && near(point(htail->world), {0.0, hBoom + 0.02, an.xHT}, 1e-12), "default htail origin: " + variant);
        check(vtail && near(point(vtail->world), {0.0, hBoom, an.xVT}, 1e-12), "default vtail origin: " + variant);
        check(prop && near(point(prop->world), {0.0, propH, an.xProp}, 1e-12), "default prop origin: " + variant);
        check(cockpit && near(point(cockpit->world), {0.0, 0.0, st.seatX}, 1e-12), "default cockpit origin: " + variant);

        // bodyAnchorsの部品別表示オフセットを含め、旧式の座標と一致する。
        check(wing && near(point(wing->world) + glm::dvec3(0.0, 0.0, an.MAC * 0.5),
                           {0.0, hWing, an.wingLE + an.MAC * 0.5}, 1e-12), "wing UI golden: " + variant);
        check(htail && near(point(htail->world) + glm::dvec3(0.0, -0.02, 0.0),
                            {0.0, hBoom, an.xHT}, 1e-12), "htail UI golden: " + variant);
        check(vtail && near(point(vtail->world) + glm::dvec3(0.0, st.vHeight * 0.5, 0.0),
                            {0.0, hBoom + st.vHeight * 0.5, an.xVT}, 1e-12), "vtail UI golden: " + variant);
        check(cockpit && near(point(cockpit->world) + glm::dvec3(0.0, 1.0, 0.0),
                              {0.0, 1.0, st.seatX}, 1e-12), "cockpit UI golden: " + variant);
        const bm::Part* root = graph.find("fuselage");
        const bm::Hardpoint* tailBeamAnchor = nullptr;
        const bm::Hardpoint* frameFrontTop = nullptr;
        const bm::Hardpoint* frameRearTop = nullptr;
        const bm::Hardpoint* tailEnd = nullptr;
        for (const auto& hp : root->hardpoints) if (hp.id == "hp.ui.tailbeam") tailBeamAnchor = &hp;
        for (const auto& hp : root->hardpoints) {
            if (hp.id == "hp.frame.front.top") frameFrontTop = &hp;
            else if (hp.id == "hp.frame.rear.top") frameRearTop = &hp;
            else if (hp.id == "hp.tail.end") tailEnd = &hp;
        }
        check(tailBeamAnchor && near(tailBeamAnchor->t.pos,
            {0.0, hBoom, (st.seatX + 0.55 + an.fusLen) * 0.5}, 1e-12), "tail beam UI golden: " + variant);
        check(frameFrontTop && near(frameFrontTop->t.pos, {0.0, hBoom, st.seatX - 0.55}, 1e-12),
              "frame front golden: " + variant);
        check(frameRearTop && near(frameRearTop->t.pos, {0.0, hBoom, st.seatX + 0.55}, 1e-12),
              "frame rear golden: " + variant);
        check(tailEnd && near(tailEnd->t.pos, {0.0, hBoom, an.fusLen}, 1e-12),
              "tail end golden: " + variant);

        std::vector<glm::dvec3> expectedGear;
        if (gear == "tandem") expectedGear = {{0.0, 0.14, st.seatX - 0.55}, {0.0, 0.15, st.seatX + 0.55}};
        else if (gear == "tri") expectedGear = {{0.0, 0.13, st.seatX - 0.55},
            {0.55, 0.15, st.seatX + 0.45}, {-0.55, 0.15, st.seatX + 0.45}};
        else if (gear == "mono") expectedGear = {{0.0, 0.16, st.seatX + 0.05},
            {0.0, 0.07, an.fusLen - 0.3}};
        std::vector<glm::dvec3> actualGear;
        for (const auto& item : placed)
            if (item.part->kind == bm::PartKind::Gear) actualGear.push_back(point(item.world));
        check(actualGear.size() == expectedGear.size(), "gear instance count golden: " + variant);
        for (const auto& expected : expectedGear) {
            bool found = false;
            for (const auto& actual : actualGear) found = found || near(actual, expected, 1e-12);
            check(found, "gear position golden: " + variant);
        }

        int analysisUse[11]{};
        double totalKg = 0.0, zMoment = 0.0;
        for (const auto& entry : graph.parts()) {
            const auto* placement = normalPlacement(placed, entry.first);
            for (const auto& mass : entry.second.massNodes) {
                if (mass.analysisItem >= 0 && mass.analysisItem < 11) ++analysisUse[mass.analysisItem];
                const glm::dvec3 cg = point(placement->world, mass.cgLocal);
                totalKg += mass.kg;
                zMoment += mass.kg * cg.z;
            }
        }
        for (int item = 0; item < 11; ++item)
            check(analysisUse[item] == 1, "analysis mass item used once " + std::to_string(item) + ": " + variant);
        check(near(totalKg, an.W, 1e-9), "layout mass total golden: " + variant);
        check(near(zMoment / totalKg, an.xCG, 1e-9), "layout mass cg golden: " + variant);
        const bm::MassBreakdown aggregate = bm::aggregateMass(graph);
        check(near(aggregate.totalKg, an.W, 1e-9), "aggregate mass total golden: " + variant);
        check(near(aggregate.cg.z, an.xCG, 1e-9), "aggregate mass cg golden: " + variant);
    }

    bm::AircraftParams st;
    st.gear = "none";
    for (const auto& mode : {std::string("L"), std::string("R"), std::string("LR")}) {
        st.boomWing = mode;
        const bm::Analysis an = bm::analyze(st);
        const bm::AirframeGraph graph = bm::buildDefaultLayout(st, an);
        int count = 0;
        for (const auto& item : graph.resolve()) if (item.part->id == "boomwing") ++count;
        check(count == (mode == "LR" ? 2 : 1), "boomwing instance count: " + mode);
        double totalKg = 0.0;
        for (const auto& entry : graph.parts())
            for (const auto& mass : entry.second.massNodes)
                totalKg += mass.kg * (entry.second.mount.mirror == bm::MirrorMode::Pair ? 2.0 : 1.0);
        check(near(totalKg, an.W, 1e-9), "boomwing layout mass total: " + mode);
        const bm::Part* boomPart = graph.find("boomwing");
        check(boomPart && !boomPart->massNodes.empty()
              && boomPart->massNodes.front().I0[2][2] > 0.0,
              "boomwing carries spanwise yaw/roll inertia: " + mode);
        const bm::MassBreakdown distributed = bm::aggregateMass(graph);
        std::vector<double> boomX;
        for (const auto& item : distributed.items)
            if (item.partId == "boomwing") boomX.push_back(item.cgWorld.x);
        if (mode == "L") check(boomX.size() == 1 && boomX[0] > 0.0,
                                "left boomwing has off-center mass CG");
        if (mode == "R") check(boomX.size() == 1 && boomX[0] < 0.0,
                                "right boomwing has off-center mass CG");
        if (mode == "LR") check(boomX.size() == 2 && near(boomX[0], -boomX[1], 1e-12)
                                 && near(distributed.cg.x, 0.0, 1e-12),
                                 "paired boomwing mass CG mirrors about centerline");
    }

    st.boomWing = "LR";
    const bm::Analysis legacyBoomAnalysis = bm::analyze(st);
    bm::AirframeGraph legacyBoom = bm::buildDefaultLayout(st, legacyBoomAnalysis);
    bm::Part legacyWing = *legacyBoom.find("wing.main");
    bm::Part legacyBoomPart = *legacyBoom.find("boomwing");
    double splitKg = 0.0;
    for (const auto& node : legacyBoomPart.massNodes) splitKg += node.kg * 2.0;
    for (auto& node : legacyWing.massNodes)
        if (node.analysisItem == 0) node.kg += splitKg;
    legacyBoomPart.massNodes.clear();
    check(legacyBoom.replacePart("wing.main", legacyWing)
          && legacyBoom.replacePart("boomwing", legacyBoomPart),
          "legacy boomwing fixture restores Phase 0B embedded mass");
    check(bm::migrateLegacyBoomWingMass(legacyBoom, st, legacyBoomAnalysis),
          "legacy boomwing mass migrates to independent component");
    const bm::MassBreakdown migratedBoom = bm::aggregateMass(legacyBoom);
    check(legacyBoom.find("boomwing") && !legacyBoom.find("boomwing")->massNodes.empty()
          && near(migratedBoom.totalKg, legacyBoomAnalysis.W, 1e-9)
          && near(migratedBoom.cg.z, legacyBoomAnalysis.xCG, 1e-9),
          "legacy boomwing migration preserves total mass and longitudinal CG");

    // Phase 0B-4 connection presets: the stable tip hardpoints allow a single
    // vertical-tail part to describe either twin-tail or wingtip-tail pairs.
    st = {};
    bm::AirframeGraph connected = bm::buildDefaultLayout(st, bm::analyze(st));
    bm::Mount fin = connected.find("tail.v")->mount;
    fin.parentId = "tail.h"; fin.hardpointId = "hp.tip"; fin.mirror = bm::MirrorMode::Pair;
    check(connected.setMount("tail.v", fin), "twin-tail preset mount validates");
    int finCount = 0;
    for (const auto& item : connected.resolve()) if (item.part->id == "tail.v") {
        ++finCount;
        check(near(std::abs(point(item.world).x), st.hSpan * 0.5, 1e-12), "twin-tail uses htail tip");
    }
    check(finCount == 2, "twin-tail resolves two fins");
    fin.parentId = "wing.main"; fin.hardpointId = "hp.tip";
    check(connected.setMount("tail.v", fin), "wingtip-tail preset mount validates");
    finCount = 0;
    for (const auto& item : connected.resolve()) if (item.part->id == "tail.v") {
        ++finCount;
        check(near(std::abs(point(item.world).x), st.span * 0.5, 1e-12), "wingtip-tail uses wing tip");
    }
    check(finCount == 2, "wingtip-tail resolves two fins");

    bm::AircraftParams invalid;
    invalid.wingX = 1001.0;
    const bm::AirframeGraph rejected = bm::buildDefaultLayout(invalid, bm::analyze(invalid));
    check(rejected.resolve().empty(), "out-of-range generated layout fails safely");
}

void aggregateMassTests() {
    bm::AirframeGraph pair;
    bm::Part root;
    root.id = "fuselage";
    root.kind = bm::PartKind::Fuselage;
    root.hardpoints.push_back({"hp.pair", {{2.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}});
    check(pair.addPart(root), "aggregate pair root");
    bm::Part pod = mounted("pod", bm::PartKind::Gear, "fuselage", "hp.pair");
    pod.mount.mirror = bm::MirrorMode::Pair;
    bm::MassNode pairMass;
    pairMass.kg = 3.0;
    pairMass.cgLocal = {0.5, 0.0, 0.0};
    pairMass.I0[0][0] = 1.0;
    pairMass.I0[1][1] = 2.0;
    pairMass.I0[2][2] = 3.0;
    pod.massNodes.push_back(pairMass);
    check(pair.addPart(pod), "aggregate paired pod");
    const bm::MassBreakdown paired = bm::aggregateMass(pair);
    check(near(paired.totalKg, 6.0) && near(paired.cg, {0.0, 0.0, 0.0}),
          "Pair mass and mirrored local CG are counted symmetrically");
    check(paired.items.size() == 2 && near(paired.items[0].cgWorld.x, 2.5)
          && near(paired.items[1].cgWorld.x, -2.5), "Pair item world CGs");
    check(near(paired.I[0][0], 2.0) && near(paired.I[1][1], 41.5)
          && near(paired.I[2][2], 43.5), "Pair parallel-axis inertia");

    bm::AirframeGraph rotated;
    bm::Part rotatedRoot;
    rotatedRoot.id = "fuselage";
    rotatedRoot.kind = bm::PartKind::Fuselage;
    rotatedRoot.hardpoints.push_back({"hp.rot", {{0.0, 0.0, 0.0}, {0.0, 0.0, 90.0}}});
    check(rotated.addPart(rotatedRoot), "aggregate rotated root");
    bm::Part rotor = mounted("rotor", bm::PartKind::Wing, "fuselage", "hp.rot");
    bm::MassNode rotatedMass;
    rotatedMass.kg = 1.0;
    rotatedMass.I0[0][0] = 1.0;
    rotatedMass.I0[1][1] = 2.0;
    rotatedMass.I0[2][2] = 3.0;
    rotor.massNodes.push_back(rotatedMass);
    check(rotated.addPart(rotor), "aggregate rotated part");
    const bm::MassBreakdown rotatedResult = bm::aggregateMass(rotated);
    check(near(rotatedResult.I[0][0], 2.0) && near(rotatedResult.I[1][1], 1.0)
          && near(rotatedResult.I[2][2], 3.0), "local inertia rotates into airframe axes");

    bm::Part invalidRoot;
    invalidRoot.id = "not-fuselage";
    invalidRoot.kind = bm::PartKind::Wing;
    const auto invalid = bm::AirframeGraph::fromUntrusted({invalidRoot});
    check(near(bm::aggregateMass(invalid).totalKg, 0.0), "invalid graph aggregate is empty");
}

void aeroLayoutTests() {
    bm::AircraftParams st;
    const bm::Analysis an = bm::analyze(st);
    const bm::AirframeGraph standard = bm::buildDefaultLayout(st, an);
    const bm::AeroLayoutProperties base = bm::aggregateAeroLayout(standard);
    check(base.valid, "default aero layout aggregates");
    check(near(base.wingLEZ, an.wingLE, 1e-12)
          && near(base.wingIncidenceDeg, st.incidence, 1e-12),
          "default wing placement preserves legacy aerodynamic geometry");
    check(near(base.hTailZ, an.xHT, 1e-12) && near(base.hAreaScale, 1.0, 1e-12)
          && near(base.vTailZ, an.xVT, 1e-12) && near(base.vAreaScale, 1.0, 1e-12),
          "default tail placement preserves legacy aerodynamic geometry");

    bm::AirframeGraph aftTail = standard;
    bm::Mount hMount = aftTail.find("tail.h")->mount;
    hMount.offset.pos.z = 0.8;
    check(aftTail.setMount("tail.h", hMount), "move horizontal tail for aero aggregation");
    const bm::AeroLayoutProperties aft = bm::aggregateAeroLayout(aftTail);
    check(aft.valid && aft.hTailZ > base.hTailZ + 0.7,
          "horizontal-tail world position feeds aero layout");

    bm::AirframeGraph twin = standard;
    bm::Mount fin = twin.find("tail.v")->mount;
    fin.parentId = "tail.h"; fin.hardpointId = "hp.tip"; fin.mirror = bm::MirrorMode::Pair;
    check(twin.setMount("tail.v", fin), "mount twin fins for aero aggregation");
    const bm::AeroLayoutProperties twinAero = bm::aggregateAeroLayout(twin);
    check(twinAero.valid && near(twinAero.vAreaScale, 2.0, 1e-12),
          "paired vertical tails double projected area");
    check(near(twinAero.vTailZ, an.xHT + st.hChord * 0.4, 1e-12),
          "paired vertical tails use resolved longitudinal position");

    bm::AirframeGraph tilted = standard;
    bm::Mount vMount = tilted.find("tail.v")->mount;
    vMount.offset.rotDeg.z = 90.0;
    check(tilted.setMount("tail.v", vMount), "tilt vertical tail for aero aggregation");
    const bm::AeroLayoutProperties tiltedAero = bm::aggregateAeroLayout(tilted);
    check(tiltedAero.valid && tiltedAero.vAreaScale < 1e-12,
          "horizontal fin has negligible vertical-tail projection");
}

void designPhysicsTests() {
    bm::AircraftParams st;
    const bm::Analysis an = bm::analyze(st);
    const bm::AirframeGraph standard = bm::buildDefaultLayout(st, an);
    const bm::DesignPhysicsProperties base = bm::aggregateDesignPhysics(standard);
    check(base.valid && base.spar.count == 1 && base.spar.section == "tube",
          "default part physics exposes legacy spar");
    check(near(base.fairingCdReduction, 0.0) && near(base.supportDragAreaM2, 0.0),
          "unfaired cantilever default adds no custom parasite correction");

    bm::AircraftParams fairedSt = st;
    fairedSt.fairing = true;
    bm::AirframeGraph faired = bm::buildDefaultLayout(fairedSt, bm::analyze(fairedSt));
    const bm::DesignPhysicsProperties fairedPhysics = bm::aggregateDesignPhysics(faired);
    check(fairedPhysics.valid && near(fairedPhysics.fairingCdReduction, 0.0012, 1e-12),
          "default fairing reproduces legacy CD0 reduction");

    bm::AirframeGraph supported = standard;
    bm::Part htail = *supported.find("tail.h");
    htail.design.tailSupport = bm::TailSupportDesign{"strut", 2, 20.0};
    check(supported.replacePart("tail.h", htail), "set support design for physics aggregation");
    const double singleArea = bm::aggregateDesignPhysics(supported).supportDragAreaM2;
    bm::Mount hMount = supported.find("tail.h")->mount;
    hMount.mirror = bm::MirrorMode::Pair;
    check(supported.setMount("tail.h", hMount), "pair supported tail for drag aggregation");
    const double pairArea = bm::aggregateDesignPhysics(supported).supportDragAreaM2;
    check(singleArea > 0.0 && near(pairArea, 2.0 * singleArea, 1e-12),
          "support drag area counts resolved Pair instances");
}

void designJsonV2Tests() {
    bm::AircraftParams saved;
    saved.span = 31.5;
    saved.posture = "recumbent";
    saved.propConfig = "pylon";
    saved.gear = "tri";
    saved.fairing = true;
    saved.sparMat = "m40j";
    const std::string encoded = bm::aircraftToJson(saved);
    check(encoded.rfind("{\"schemaVersion\":2,", 0) == 0, "v2 schemaVersion is the first key");
    check(encoded.find("\"layout\"") == std::string::npos, "Phase 0A does not save generated layout");
    bm::AircraftParams loaded;
    const bm::AircraftJsonReport roundTrip = bm::aircraftFromJson(encoded, loaded);
    check(roundTrip.schemaVersion == 2 && !roundTrip.layoutPresent, "v2 flat document report");
    check(bm::aircraftToJson(loaded) == encoded, "v2 flat JSON byte-stable round trip");

    bm::AirframeGraph custom = bm::buildDefaultLayout(saved, bm::analyze(saved));
    bm::Part customWing = *custom.find("wing.main");
    customWing.mount.offset.pos = {1.25, -0.2, 0.4};
    customWing.mount.offset.rotDeg = {12.0, -4.0, 7.0};
    customWing.massNodes[0].I0[0][0] = 3.5;
    customWing.massNodes[0].I0[1][2] = -0.25;
    customWing.design.spar = bm::SparDesign{2, 0.37, 128.0, 44.0, 5, "box"};
    check(custom.replacePart("wing.main", customWing), "v3 custom fixture edit");
    bm::Part customFairing = *custom.find("fairing");
    customFairing.design.fairing = bm::FairingDesign{2.65, 0.82, 1.18, 0.19, 0.71};
    check(custom.replacePart("fairing", customFairing), "v3 fairing design fixture");
    bm::Part customPilot = *custom.find("pilot");
    customPilot.design.pilot = bm::PilotStationDesign{0.55, -0.95, 0.48, -0.67, 0.62};
    check(custom.replacePart("pilot", customPilot), "v3 pilot station fixture");
    bm::Part customTail = *custom.find("tail.v");
    customTail.design.tailSupport = bm::TailSupportDesign{"strut", 2, 14.0};
    check(custom.replacePart("tail.v", customTail), "v3 tail support fixture");
    const std::string customJson = bm::aircraftToJson(saved, &custom);
    bm::AircraftParams customParams;
    bm::AirframeGraph customLoaded;
    const bm::AircraftJsonReport customReport = bm::aircraftFromJson(customJson, customParams, &customLoaded);
    check(customReport.schemaVersion == 3 && customReport.layoutPresent && customReport.layoutAccepted,
          "v3 custom layout is accepted");
    const bm::Part* loadedWing = customLoaded.find("wing.main");
    check(loadedWing && near(loadedWing->mount.offset.pos, customWing.mount.offset.pos)
          && near(loadedWing->mount.offset.rotDeg, customWing.mount.offset.rotDeg),
          "v3 mount transform round trip");
    check(loadedWing && loadedWing->massNodes.size() == customWing.massNodes.size()
          && near(loadedWing->massNodes[0].I0[0][0], 3.5)
          && near(loadedWing->massNodes[0].I0[1][2], -0.25), "v3 mass inertia round trip");
    check(loadedWing && loadedWing->design.spar && loadedWing->design.spar->count == 2
          && near(loadedWing->design.spar->chordFrac, 0.37)
          && loadedWing->design.spar->section == "box", "v3 spar design round trip");
    const bm::Part* loadedFairing = customLoaded.find("fairing");
    const bm::Part* loadedPilot = customLoaded.find("pilot");
    const bm::Part* loadedTail = customLoaded.find("tail.v");
    check(loadedFairing && loadedFairing->design.fairing
          && near(loadedFairing->design.fairing->lengthM, 2.65)
          && near(loadedFairing->design.fairing->tailRatio, 0.71), "v3 fairing design round trip");
    check(loadedPilot && loadedPilot->design.pilot
          && near(loadedPilot->design.pilot->pedalZM, -0.95)
          && near(loadedPilot->design.pilot->crankHeightM, 0.62), "v3 pilot design round trip");
    check(loadedTail && loadedTail->design.tailSupport
          && loadedTail->design.tailSupport->mounting == "strut"
          && loadedTail->design.tailSupport->supportCount == 2, "v3 tail support round trip");
    check(bm::aircraftToJson(customParams, &customLoaded) == customJson, "v3 JSON byte-stable round trip");

    const std::string validLayout = R"({
      "schemaVersion":2,"span":30,
      "layout":{"units":{"length":"m","angle":"deg","mass":"kg"},"parts":[
        {"id":"fuselage","kind":"fuselage","hardpoints":[
          {"id":"hp.wing","pos":[0,2,1.5],"rot":[3,0,0]}]},
        {"id":"wing.main","kind":"wing","mount":{"parent":"fuselage",
          "hardpoint":"hp.wing","offset":{"pos":[0,0,0],"rot":[0,0,0]},"mirror":"none"},
          "mass":{"kg":18.4,"cg":[0,0.05,0.21]}}
      ]}}
    )";
    bm::AircraftParams validParams;
    const bm::AircraftJsonReport valid = bm::aircraftFromJson(validLayout, validParams);
    check(valid.schemaVersion == 2 && valid.layoutPresent && valid.layoutAccepted,
          "valid v2 layout is parsed and accepted");
    check(near(validParams.span, 30.0), "valid v2 still reads flat parameters");
    check(bm::buildDefaultLayout(validParams, bm::analyze(validParams)).validate().empty(),
          "accepted layout is discarded in favor of deterministic Phase 0A layout");

    const std::string tolerantLayout = R"({"schemaVersion":2,"layout":{"parts":[
      {"id":"fuselage","kind":"fuselage","hardpoints":[{"id":"hp","pos":[0,0,0]}]},
      {"id":"future","kind":"future-kind","mount":{"parent":"fuselage","hardpoint":"hp",
       "offset":{"pos":[1,2],"rot":[0,0,0]},"mirror":"future-mode"},"futureField":123}
    ]}})";
    bm::AircraftParams tolerantParams;
    const bm::AircraftJsonReport tolerant = bm::aircraftFromJson(tolerantLayout, tolerantParams);
    check(tolerant.layoutAccepted && tolerant.warnings.size() >= 3,
          "unknown kind/mirror and short vec3 fall back with warnings");

    const std::vector<std::string> invalidLayouts = {
        R"({"schemaVersion":2,"layout":{"parts":[{"id":"fuselage","kind":"fuselage"},{"id":"x","kind":"wing","mount":{"parent":"missing","hardpoint":"hp"}}]}})",
        R"({"schemaVersion":2,"layout":{"parts":[{"id":"fuselage","kind":"fuselage"},{"id":"fuselage","kind":"fuselage"}]}})",
        R"({"schemaVersion":2,"layout":{"parts":[{"id":"fuselage","kind":"fuselage","hardpoints":[{"id":"hp","pos":[1001,0,0]}]}]}})",
        R"({"schemaVersion":3,"layout":{"parts":[{"id":"fuselage","kind":"fuselage","design":{"spar":{"count":99}}}]}})"
    };
    for (std::size_t i = 0; i < invalidLayouts.size(); ++i) {
        bm::AircraftParams params;
        const bm::AircraftJsonReport report = bm::aircraftFromJson(invalidLayouts[i], params);
        check(report.layoutPresent && !report.layoutAccepted && !report.warnings.empty(),
              "invalid layout falls back " + std::to_string(i));
    }

    std::string excessiveParts = R"({"schemaVersion":2,"layout":{"parts":[)";
    for (int i = 0; i < 513; ++i) {
        if (i) excessiveParts += ',';
        excessiveParts += R"({"id":"p)" + std::to_string(i) + R"(","kind":"unknown"})";
    }
    excessiveParts += "]}}";
    bm::AircraftParams excessiveParams;
    const bm::AircraftJsonReport excessive = bm::aircraftFromJson(excessiveParts, excessiveParams);
    check(excessive.layoutPresent && !excessive.layoutAccepted && !excessive.warnings.empty(),
          "excessive layout part count falls back");

    bm::AircraftParams brokenParams;
    const bm::AircraftJsonReport broken = bm::aircraftFromJson(
        "{\"schemaVersion\":2,\"span\":33,\"layout\":{\"parts\":[", brokenParams);
    check(broken.layoutPresent && !broken.layoutAccepted && near(brokenParams.span, 33.0),
          "truncated layout falls back without losing readable flat fields");

    bm::AircraftParams futureParams;
    const bm::AircraftJsonReport future = bm::aircraftFromJson(
        R"({"schemaVersion":99,"span":34,"layout":{"parts":[]}})", futureParams);
    check(future.schemaVersion == 99 && future.layoutPresent && !future.layoutAccepted
          && near(futureParams.span, 34.0), "future schema reads flat fields and ignores layout");

    const std::string legacy = R"({"span":28,"rootChord":1,"tipChord":0.55,"wingX":1.5,
      "posture":"semi","propConfig":"tractor","gear":"tandem","sparMat":"t700"})";
    bm::AircraftParams legacyLoaded, legacyExpected;
    legacyExpected.span = 28; legacyExpected.rootChord = 1; legacyExpected.tipChord = 0.55;
    legacyExpected.wingX = 1.5; legacyExpected.posture = "semi";
    legacyExpected.propConfig = "tractor"; legacyExpected.gear = "tandem"; legacyExpected.sparMat = "t700";
    const bm::AircraftJsonReport legacyReport = bm::aircraftFromJson(legacy, legacyLoaded);
    const bm::Analysis actual = bm::analyze(legacyLoaded);
    const bm::Analysis expected = bm::analyze(legacyExpected);
    check(legacyReport.schemaVersion == 1 && !legacyReport.layoutPresent, "legacy document is v1");
    check(near(actual.W, expected.W, 1e-12) && near(actual.xCG, expected.xCG, 1e-12)
          && near(actual.SM, expected.SM, 1e-12) && near(actual.Preq, expected.Preq, 1e-12),
          "v1 analysis W/xCG/SM/Preq remains unchanged");

    const std::string path = "airframe_v1_designs_test.json";
    {
        std::ofstream file(path);
        file << "{\"designs\":[{\"name\":\"legacy\",\"b\":28,\"W\":0,\"SM\":0,"
                "\"Preq\":0,\"LD\":0,\"dist\":-1,\"st\":" << legacy << "}]}";
    }
    const auto designs = bm::loadDesigns(path);
    std::remove(path.c_str());
    check(designs.size() == 1 && designs[0].name == "legacy", "legacy designs.json entry loads");
    if (designs.size() == 1) {
        const bm::Analysis fromFile = bm::analyze(designs[0].st);
        check(near(fromFile.W, expected.W, 1e-12) && near(fromFile.xCG, expected.xCG, 1e-12)
              && near(fromFile.SM, expected.SM, 1e-12) && near(fromFile.Preq, expected.Preq, 1e-12),
              "legacy designs.json W/xCG/SM/Preq remains unchanged");
    }

    const std::string customPath = "airframe_v3_designs_test.json";
    bm::DesignEntry customEntry;
    customEntry.name = "custom";
    customEntry.st = saved;
    customEntry.layout = custom;
    bm::saveDesigns({customEntry}, customPath);
    const auto customDesigns = bm::loadDesigns(customPath);
    std::remove(customPath.c_str());
    check(customDesigns.size() == 1 && customDesigns[0].layout.has_value()
          && customDesigns[0].layout->find("wing.main"), "design list preserves v3 custom layout");
}

} // namespace

int main() {
    miniJsonTests();
    rotationTests();
    graphResolveTests();
    validationTests();
    mutationTests();
    atomicEditingTests();
    defaultLayoutTests();
    aggregateMassTests();
    aeroLayoutTests();
    designPhysicsTests();
    designJsonV2Tests();
    if (failures == 0) std::cout << "airframe_test: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
