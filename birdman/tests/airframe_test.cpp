#include "core/Airframe.hpp"
#include "core/MiniJson.hpp"

#include <glm/glm.hpp>

#include <cmath>
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

} // namespace

int main() {
    miniJsonTests();
    rotationTests();
    graphResolveTests();
    validationTests();
    mutationTests();
    if (failures == 0) std::cout << "airframe_test: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
