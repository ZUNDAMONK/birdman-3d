#include "core/DesignIO.hpp"
#include "core/Airframe.hpp"
#include "core/Aircraft.hpp"
#include "core/Material.hpp"
#include "core/MiniJson.hpp"
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <limits>
#if defined(_WIN32)
#include <direct.h>
#include <windows.h>
#define MKDIR(d) _mkdir(d)
#else
#include <sys/stat.h>
#include <unistd.h>
#define MKDIR(d) mkdir(d, 0755)
#endif

namespace bm {

std::string exeDirPath() {
#if defined(_WIN32)
    char buf[MAX_PATH];
    const DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    std::string s(buf, n);
#else
    char buf[4096];
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    std::string s(buf, n > 0 ? (size_t)n : 0);
#endif
    const size_t p = s.find_last_of("\\/");
    return p == std::string::npos ? "." : s.substr(0, p);
}

std::string savePath(const std::string& filename) {
    const std::string dir = exeDirPath() + "/save";
    MKDIR(dir.c_str());
    return dir + "/" + filename;
}

// ---- ミニJSONヘルパー(自前フォーマット専用の緩いパーサ) ----
static double jNum(const std::string& s, const char* key, double def) {
    std::string k = std::string("\"") + key + "\":";
    size_t p = s.find(k);
    if (p == std::string::npos) return def;
    return std::atof(s.c_str() + p + k.size());
}
static std::string jStr(const std::string& s, const char* key, const std::string& def) {
    std::string k = std::string("\"") + key + "\":\"";
    size_t p = s.find(k);
    if (p == std::string::npos) return def;
    size_t e = s.find('"', p + k.size());
    if (e == std::string::npos) return def;
    return s.substr(p + k.size(), e - p - k.size());
}
static bool jBool(const std::string& s, const char* key, bool def) {
    std::string k = std::string("\"") + key + "\":";
    size_t p = s.find(k);
    if (p == std::string::npos) return def;
    return s.compare(p + k.size(), 4, "true") == 0;
}

std::string aircraftToJson(const AircraftParams& st) {
    std::ostringstream o;
    char b[64];
    auto num = [&](const char* k, double v, bool comma = true) {
        std::snprintf(b, sizeof(b), "\"%s\":%g", k, v);
        o << b << (comma ? "," : "");
    };
    auto str = [&](const char* k, const std::string& v) { o << "\"" << k << "\":\"" << v << "\","; };
    o << "{\"schemaVersion\":2,";
    str("planform", st.planform);
    num("span", st.span); num("rootChord", st.rootChord); num("tipChord", st.tipChord);
    num("wingX", st.wingX); num("dihedral", st.dihedral); num("washout", st.washout);
    num("incidence", st.incidence); num("sweep", st.sweep);
    str("airfoil", st.airfoil); str("jig", st.jig);
    num("ribPitch", st.ribPitch);
    num("plankTop", st.plankTop); num("plankBot", st.plankBot);
    num("plankRearTop", st.plankRearTop); num("plankRearBot", st.plankRearBot);
    str("ailMode", st.ailMode);
    num("ailSpanFrac", st.ailSpanFrac); num("ailChordFrac", st.ailChordFrac);
    str("propConfig", st.propConfig); str("propMat", st.propMat);
    num("propDia", st.propDia); num("propPitch", st.propPitch); num("propBlades", st.propBlades);
    o << "\"varPitch\":" << (st.varPitch ? "true" : "false") << ",";
    num("flapSpanFrac", st.flapSpanFrac);
    str("posture", st.posture);
    num("seatX", st.seatX); num("pilotW", st.pilotW); num("cd0Add", st.cd0Add); num("powerMax", st.powerMax);
    o << "\"fairing\":" << (st.fairing ? "true" : "false") << ",";
    str("drive", st.drive);
    num("driveEffPct", st.driveEffPct);
    str("gear", st.gear);
    num("segments", st.segments); num("rootDia", st.rootDia); num("tipDia", st.tipDia); str("sparMat", st.sparMat);
    str("hShape", st.hShape);
    num("hSpan", st.hSpan); num("hChord", st.hChord); num("tailArm", st.tailArm);
    str("vShape", st.vShape);
    num("vHeight", st.vHeight); num("vChord", st.vChord);
    num("elevRatio", st.elevRatio); num("rudRatio", st.rudRatio); num("boomDia", st.boomDia);
    str("boomWing", st.boomWing);
    num("boomWingSpan", st.boomWingSpan); num("boomWingChord", st.boomWingChord);
    num("boomWingPos", st.boomWingPos, false);
    o << "}";
    return o.str();
}

std::string aircraftToJson(const AircraftParams& st, const AirframeGraph* layout) {
    if (!layout) return aircraftToJson(st);
    const auto errors = layout->validate();
    if (!errors.empty()) return aircraftToJson(st);

    auto vec3 = [](const glm::dvec3& v) {
        return json::Value::arrayValue({json::Value::numberValue(v.x),
            json::Value::numberValue(v.y), json::Value::numberValue(v.z)});
    };
    json::Value layoutValue = json::Value::objectValue();
    layoutValue.object["units"] = json::Value::objectValue({
        {"angle", json::Value::stringValue("deg")}, {"length", json::Value::stringValue("m")},
        {"mass", json::Value::stringValue("kg")}});
    json::Value parts = json::Value::arrayValue();
    for (const auto& entry : layout->parts()) {
        const Part& part = entry.second;
        json::Value encoded = json::Value::objectValue();
        encoded.object["id"] = json::Value::stringValue(part.id);
        encoded.object["kind"] = json::Value::stringValue(partKindName(part.kind));
        if (!part.mount.parentId.empty()) {
            json::Value offset = json::Value::objectValue({{"pos", vec3(part.mount.offset.pos)},
                                                           {"rot", vec3(part.mount.offset.rotDeg)}});
            encoded.object["mount"] = json::Value::objectValue({
                {"hardpoint", json::Value::stringValue(part.mount.hardpointId)},
                {"mirror", json::Value::stringValue(mirrorModeName(part.mount.mirror))},
                {"offset", std::move(offset)},
                {"parent", json::Value::stringValue(part.mount.parentId)}});
        }
        json::Value hardpoints = json::Value::arrayValue();
        for (const auto& hp : part.hardpoints)
            hardpoints.array.push_back(json::Value::objectValue({{"id", json::Value::stringValue(hp.id)},
                {"pos", vec3(hp.t.pos)}, {"rot", vec3(hp.t.rotDeg)}}));
        if (!hardpoints.array.empty()) encoded.object["hardpoints"] = std::move(hardpoints);

        json::Value design = json::Value::objectValue();
        if (part.design.spar) {
            const SparDesign& value = *part.design.spar;
            design.object["spar"] = json::Value::objectValue({
                {"chordFrac", json::Value::numberValue(value.chordFrac)},
                {"count", json::Value::numberValue(value.count)},
                {"jointCount", json::Value::numberValue(value.jointCount)},
                {"rootDiaMm", json::Value::numberValue(value.rootDiaMm)},
                {"section", json::Value::stringValue(value.section)},
                {"tipDiaMm", json::Value::numberValue(value.tipDiaMm)}});
        }
        if (part.design.fairing) {
            const FairingDesign& value = *part.design.fairing;
            design.object["fairing"] = json::Value::objectValue({
                {"heightM", json::Value::numberValue(value.heightM)},
                {"lengthM", json::Value::numberValue(value.lengthM)},
                {"noseRatio", json::Value::numberValue(value.noseRatio)},
                {"tailRatio", json::Value::numberValue(value.tailRatio)},
                {"widthM", json::Value::numberValue(value.widthM)}});
        }
        if (part.design.pilot) {
            const PilotStationDesign& value = *part.design.pilot;
            design.object["pilot"] = json::Value::objectValue({
                {"crankHeightM", json::Value::numberValue(value.crankHeightM)},
                {"crankZM", json::Value::numberValue(value.crankZM)},
                {"pedalHeightM", json::Value::numberValue(value.pedalHeightM)},
                {"pedalZM", json::Value::numberValue(value.pedalZM)},
                {"seatHeightM", json::Value::numberValue(value.seatHeightM)}});
        }
        if (part.design.tailSupport) {
            const TailSupportDesign& value = *part.design.tailSupport;
            design.object["tailSupport"] = json::Value::objectValue({
                {"mounting", json::Value::stringValue(value.mounting)},
                {"supportCount", json::Value::numberValue(value.supportCount)},
                {"supportDiaMm", json::Value::numberValue(value.supportDiaMm)}});
        }
        if (!design.object.empty()) encoded.object["design"] = std::move(design);

        json::Value masses = json::Value::arrayValue();
        for (const auto& mass : part.massNodes) {
            json::Value inertia = json::Value::arrayValue();
            for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column)
                inertia.array.push_back(json::Value::numberValue(mass.I0[column][row]));
            masses.array.push_back(json::Value::objectValue({
                {"analysisItem", json::Value::numberValue(mass.analysisItem)},
                {"cg", vec3(mass.cgLocal)}, {"inertia", std::move(inertia)},
                {"kg", json::Value::numberValue(mass.kg)}}));
        }
        if (!masses.array.empty()) encoded.object["masses"] = std::move(masses);
        parts.array.push_back(std::move(encoded));
    }
    layoutValue.object["parts"] = std::move(parts);

    std::string flat = aircraftToJson(st);
    const std::string version2 = "\"schemaVersion\":2";
    const std::size_t versionAt = flat.find(version2);
    if (versionAt != std::string::npos) flat.replace(versionAt, version2.size(), "\"schemaVersion\":3");
    flat.pop_back();
    return flat + ",\"layout\":" + json::stringify(layoutValue) + "}";
}

namespace {

constexpr std::size_t kMaxLayoutParts = 512;
constexpr std::size_t kMaxPartHardpoints = 256;
constexpr std::size_t kMaxPartMassNodes = 256;

void addWarning(AircraftJsonReport& report, std::string message) {
    report.warnings.push_back(std::move(message));
}

static std::string jsonObjectAt(const std::string& s, size_t open) {
    if (open >= s.size() || s[open] != '{') return {};
    int depth = 0;
    bool inString = false, escaped = false;
    for (size_t i = open; i < s.size(); ++i) {
        const char c = s[i];
        if (inString) {
            if (escaped) escaped = false;
            else if (c == '\\') escaped = true;
            else if (c == '"') inString = false;
            continue;
        }
        if (c == '"') inString = true;
        else if (c == '{') ++depth;
        else if (c == '}' && --depth == 0) return s.substr(open, i - open + 1);
    }
    return {};
}

std::string stringField(const json::Value* object, const char* key, const std::string& fallback = "") {
    if (!object || !object->isObject()) return fallback;
    const json::Value* value = object->find(key);
    return value && value->isString() ? value->string : fallback;
}

double numberField(const json::Value* object, const char* key, double fallback,
                   AircraftJsonReport& report, const std::string& context) {
    if (!object || !object->isObject()) return fallback;
    const json::Value* value = object->find(key);
    if (!value) return fallback;
    if (!value->isNumber() || !std::isfinite(value->number)) {
        addWarning(report, context + "." + key + " must be a finite number; using default");
        return fallback;
    }
    return value->number;
}

int integerField(const json::Value* object, const char* key, int fallback,
                 AircraftJsonReport& report, const std::string& context) {
    const double value = numberField(object, key, fallback, report, context);
    if (std::floor(value) != value || value < (double)std::numeric_limits<int>::min()
        || value > (double)std::numeric_limits<int>::max()) {
        addWarning(report, context + "." + key + " must be an integer; using default");
        return fallback;
    }
    return (int)value;
}

glm::dvec3 vectorField(const json::Value* object, const char* key,
                       AircraftJsonReport& report, const std::string& context) {
    glm::dvec3 result(0.0);
    if (!object || !object->isObject()) return result;
    const json::Value* value = object->find(key);
    if (!value) return result;
    if (!vec3FromJson(*value, result))
        addWarning(report, context + "." + key + " must contain exactly three finite numbers; using zero");
    return result;
}

AirframeGraph parseLayout(const json::Value& layout, AircraftJsonReport& report) {
    std::vector<Part> parts;
    if (!layout.isObject()) {
        addWarning(report, "layout must be an object");
        return AirframeGraph::fromUntrusted(std::move(parts));
    }
    const json::Value* values = layout.find("parts");
    if (!values || !values->isArray()) {
        addWarning(report, "layout.parts must be an array");
        return AirframeGraph::fromUntrusted(std::move(parts));
    }
    if (values->array.size() > kMaxLayoutParts) {
        addWarning(report, "layout.parts exceeds the maximum of " + std::to_string(kMaxLayoutParts));
        return AirframeGraph::fromUntrusted(std::move(parts));
    }
    for (std::size_t index = 0; index < values->array.size(); ++index) {
        const json::Value& source = values->array[index];
        const std::string context = "layout.parts[" + std::to_string(index) + "]";
        Part part;
        if (!source.isObject()) {
            addWarning(report, context + " must be an object");
            parts.push_back(std::move(part));
            continue;
        }
        part.id = stringField(&source, "id");
        const std::string kind = stringField(&source, "kind", "unknown");
        part.kind = partKindFromString(kind);
        if (part.kind == PartKind::Unknown && kind != "unknown")
            addWarning(report, context + ".kind is unknown; using unknown");

        if (const json::Value* mount = source.find("mount")) {
            part.mount.parentId = stringField(mount, "parent");
            part.mount.hardpointId = stringField(mount, "hardpoint");
            const json::Value* offset = mount->isObject() ? mount->find("offset") : nullptr;
            part.mount.offset.pos = vectorField(offset, "pos", report, context + ".mount.offset");
            part.mount.offset.rotDeg = vectorField(offset, "rot", report, context + ".mount.offset");
            const std::string mirror = stringField(mount, "mirror", "none");
            part.mount.mirror = mirrorModeFromString(mirror);
            if (mirror != "none" && mirror != "pair")
                addWarning(report, context + ".mount.mirror is unknown; using none");
        }

        if (const json::Value* hardpoints = source.find("hardpoints")) {
            if (!hardpoints->isArray()) addWarning(report, context + ".hardpoints must be an array");
            else if (hardpoints->array.size() > kMaxPartHardpoints) {
                addWarning(report, context + ".hardpoints exceeds the maximum of "
                    + std::to_string(kMaxPartHardpoints));
                return AirframeGraph::fromUntrusted({});
            } else for (std::size_t hpIndex = 0; hpIndex < hardpoints->array.size(); ++hpIndex) {
                const json::Value& hpSource = hardpoints->array[hpIndex];
                const std::string hpContext = context + ".hardpoints[" + std::to_string(hpIndex) + "]";
                Hardpoint hp;
                hp.id = stringField(&hpSource, "id");
                hp.t.pos = vectorField(&hpSource, "pos", report, hpContext);
                hp.t.rotDeg = vectorField(&hpSource, "rot", report, hpContext);
                part.hardpoints.push_back(std::move(hp));
            }
        }

        if (const json::Value* design = source.find("design")) {
            if (!design->isObject()) addWarning(report, context + ".design must be an object");
            else {
                if (const json::Value* spec = design->find("spar")) {
                    SparDesign value;
                    const std::string where = context + ".design.spar";
                    value.count = integerField(spec, "count", value.count, report, where);
                    value.chordFrac = numberField(spec, "chordFrac", value.chordFrac, report, where);
                    value.rootDiaMm = numberField(spec, "rootDiaMm", value.rootDiaMm, report, where);
                    value.tipDiaMm = numberField(spec, "tipDiaMm", value.tipDiaMm, report, where);
                    value.jointCount = integerField(spec, "jointCount", value.jointCount, report, where);
                    value.section = stringField(spec, "section", value.section);
                    part.design.spar = std::move(value);
                }
                if (const json::Value* spec = design->find("fairing")) {
                    FairingDesign value;
                    const std::string where = context + ".design.fairing";
                    value.lengthM = numberField(spec, "lengthM", value.lengthM, report, where);
                    value.widthM = numberField(spec, "widthM", value.widthM, report, where);
                    value.heightM = numberField(spec, "heightM", value.heightM, report, where);
                    value.noseRatio = numberField(spec, "noseRatio", value.noseRatio, report, where);
                    value.tailRatio = numberField(spec, "tailRatio", value.tailRatio, report, where);
                    part.design.fairing = std::move(value);
                }
                if (const json::Value* spec = design->find("pilot")) {
                    PilotStationDesign value;
                    const std::string where = context + ".design.pilot";
                    value.seatHeightM = numberField(spec, "seatHeightM", value.seatHeightM, report, where);
                    value.pedalZM = numberField(spec, "pedalZM", value.pedalZM, report, where);
                    value.pedalHeightM = numberField(spec, "pedalHeightM", value.pedalHeightM, report, where);
                    value.crankZM = numberField(spec, "crankZM", value.crankZM, report, where);
                    value.crankHeightM = numberField(spec, "crankHeightM", value.crankHeightM, report, where);
                    part.design.pilot = std::move(value);
                }
                if (const json::Value* spec = design->find("tailSupport")) {
                    TailSupportDesign value;
                    const std::string where = context + ".design.tailSupport";
                    value.mounting = stringField(spec, "mounting", value.mounting);
                    value.supportCount = integerField(spec, "supportCount", value.supportCount, report, where);
                    value.supportDiaMm = numberField(spec, "supportDiaMm", value.supportDiaMm, report, where);
                    part.design.tailSupport = std::move(value);
                }
            }
        }

        auto readMass = [&](const json::Value& mass, const std::string& massContext) {
            if (!mass.isObject()) { addWarning(report, massContext + " must be an object"); return; }
            MassNode node;
            if (const json::Value* kg = mass.find("kg"); kg && kg->isNumber()) node.kg = kg->number;
            node.cgLocal = vectorField(&mass, "cg", report, massContext);
            if (const json::Value* item = mass.find("analysisItem"); item && item->isNumber()) {
                const double value = item->number;
                if (std::floor(value) == value && value >= -1.0
                    && value <= (double)std::numeric_limits<int>::max())
                    node.analysisItem = (int)value;
                else addWarning(report, massContext + ".analysisItem must be -1 or a non-negative integer");
            }
            if (const json::Value* inertia = mass.find("inertia")) {
                if (!inertia->isArray() || inertia->array.size() != 9) {
                    addWarning(report, massContext + ".inertia must contain nine finite numbers");
                } else {
                    bool valid = true;
                    for (const auto& value : inertia->array)
                        valid = valid && value.isNumber() && std::isfinite(value.number);
                    if (!valid) addWarning(report, massContext + ".inertia must contain nine finite numbers");
                    else for (int row = 0; row < 3; ++row) for (int column = 0; column < 3; ++column)
                        node.I0[column][row] = inertia->array[(std::size_t)(row * 3 + column)].number;
                }
            }
            if (part.kind != PartKind::Unknown) part.massNodes.push_back(node);
        };
        if (const json::Value* masses = source.find("masses")) {
            if (!masses->isArray()) addWarning(report, context + ".masses must be an array");
            else if (masses->array.size() > kMaxPartMassNodes) {
                addWarning(report, context + ".masses exceeds the maximum of "
                    + std::to_string(kMaxPartMassNodes));
                return AirframeGraph::fromUntrusted({});
            }
            else for (std::size_t massIndex = 0; massIndex < masses->array.size(); ++massIndex)
                readMass(masses->array[massIndex], context + ".masses[" + std::to_string(massIndex) + "]");
        } else if (const json::Value* mass = source.find("mass")) {
            readMass(*mass, context + ".mass");
        }
        parts.push_back(std::move(part));
    }
    return AirframeGraph::fromUntrusted(std::move(parts));
}

} // namespace

AircraftJsonReport aircraftFromJson(const std::string& s, AircraftParams& st, AirframeGraph* acceptedLayout) {
    AircraftJsonReport report;
    if (acceptedLayout) *acceptedLayout = {};
    json::Value document;
    json::ParseError parseError;
    const bool parsed = json::parse(s, document, &parseError);
    if (parsed && document.isObject()) {
        if (const json::Value* version = document.find("schemaVersion"); version && version->isNumber())
            report.schemaVersion = (int)std::lround(version->number);
        if (const json::Value* layout = document.find("layout")) {
            report.layoutPresent = true;
            if (report.schemaVersion > 3) {
                addWarning(report, "schemaVersion is newer than supported; layout ignored");
            } else if (report.schemaVersion >= 2) {
                AirframeGraph candidate = parseLayout(*layout, report);
                const auto errors = candidate.validate();
                for (const auto& error : errors)
                    addWarning(report, "layout " + (error.partId.empty() ? std::string() : error.partId + ": ") + error.message);
                report.layoutAccepted = errors.empty();
                if (report.layoutAccepted && acceptedLayout) *acceptedLayout = std::move(candidate);
            } else {
                addWarning(report, "layout ignored for legacy schema");
            }
        }
    } else if (s.find("\"layout\"") != std::string::npos) {
        report.layoutPresent = true;
        addWarning(report, "layout JSON parse failed at byte " + std::to_string(parseError.offset));
    }

    st.planform = jStr(s, "planform", st.planform);
    st.span = jNum(s, "span", st.span);
    st.rootChord = jNum(s, "rootChord", st.rootChord);
    st.tipChord = jNum(s, "tipChord", st.tipChord);
    st.wingX = jNum(s, "wingX", st.wingX);
    st.dihedral = jNum(s, "dihedral", st.dihedral);
    st.washout = jNum(s, "washout", st.washout);
    st.incidence = jNum(s, "incidence", st.incidence);
    st.sweep = jNum(s, "sweep", st.sweep);
    st.airfoil = jStr(s, "airfoil", st.airfoil);
    st.jig = jStr(s, "jig", st.jig);
    st.ribPitch = jNum(s, "ribPitch", st.ribPitch);
    st.plankTop = jNum(s, "plankTop", st.plankTop);
    st.plankBot = jNum(s, "plankBot", st.plankBot);
    st.plankRearTop = jNum(s, "plankRearTop", st.plankRearTop);
    st.plankRearBot = jNum(s, "plankRearBot", st.plankRearBot);
    st.ailMode = jStr(s, "ailMode", st.ailMode);
    st.ailSpanFrac = jNum(s, "ailSpanFrac", st.ailSpanFrac);
    st.ailChordFrac = jNum(s, "ailChordFrac", st.ailChordFrac);
    st.propConfig = jStr(s, "propConfig", st.propConfig);
    st.propMat = jStr(s, "propMat", st.propMat);
    st.propDia = jNum(s, "propDia", st.propDia);
    st.propPitch = jNum(s, "propPitch", st.propPitch);
    st.propBlades = (int)std::lround(jNum(s, "propBlades", st.propBlades));
    st.varPitch = jBool(s, "varPitch", st.varPitch);
    st.flapSpanFrac = jNum(s, "flapSpanFrac", st.flapSpanFrac);
    st.posture = jStr(s, "posture", st.posture);
    st.seatX = jNum(s, "seatX", st.seatX);
    st.pilotW = jNum(s, "pilotW", st.pilotW);
    st.cd0Add = jNum(s, "cd0Add", st.cd0Add);
    st.powerMax = jNum(s, "powerMax", st.powerMax);
    st.fairing = jBool(s, "fairing", st.fairing);
    st.drive = jStr(s, "drive", st.drive);
    st.driveEffPct = jNum(s, "driveEffPct", st.driveEffPct);
    st.gear = jStr(s, "gear", st.gear);
    st.segments = (int)std::lround(jNum(s, "segments", st.segments));
    st.rootDia = jNum(s, "rootDia", st.rootDia);
    st.tipDia = jNum(s, "tipDia", st.tipDia);
    const bool hasLegacySparMod = s.find("\"sparMod\":") != std::string::npos;
    const double legacySparMod = jNum(s, "sparMod", 230.0);
    const bool hasSparMat = s.find("\"sparMat\":\"") != std::string::npos;
    st.sparMat = hasSparMat ? jStr(s, "sparMat", st.sparMat)
                            : hasLegacySparMod ? materialFromLegacyMod(legacySparMod)
                                               : st.sparMat;
    st.sparMat = materialOf(st.sparMat).id; // unknown IDs safely fall back to T700
    st.hShape = jStr(s, "hShape", st.hShape);
    st.hSpan = jNum(s, "hSpan", st.hSpan);
    st.hChord = jNum(s, "hChord", st.hChord);
    st.tailArm = jNum(s, "tailArm", st.tailArm);
    st.vShape = jStr(s, "vShape", st.vShape);
    st.vHeight = jNum(s, "vHeight", st.vHeight);
    st.vChord = jNum(s, "vChord", st.vChord);
    st.elevRatio = jNum(s, "elevRatio", st.elevRatio);
    st.rudRatio = jNum(s, "rudRatio", st.rudRatio);
    st.boomDia = jNum(s, "boomDia", st.boomDia);
    st.boomWing = jStr(s, "boomWing", st.boomWing);
    st.boomWingSpan = jNum(s, "boomWingSpan", st.boomWingSpan);
    st.boomWingChord = jNum(s, "boomWingChord", st.boomWingChord);
    st.boomWingPos = jNum(s, "boomWingPos", st.boomWingPos);
    for (const auto& warning : report.warnings)
        std::fprintf(stderr, "warning: aircraft JSON: %s\n", warning.c_str());
    return report;
}

AircraftJsonReport aircraftFromJson(const std::string& s, AircraftParams& st) {
    return aircraftFromJson(s, st, nullptr);
}

std::vector<DesignEntry> loadDesigns(const std::string& path) {
    std::vector<DesignEntry> v;
    std::ifstream f(path.empty() ? savePath("designs.json") : path);
    if (!f) return v;
    std::stringstream ss; ss << f.rdbuf();
    const std::string s = ss.str();
    // トップレベル配列の要素({...})をブレース深度で切り出す
    size_t arr = s.find('[');
    if (arr == std::string::npos) return v;
    int depth = 0;
    size_t start = 0;
    for (size_t i = arr; i < s.size(); i++) {
        if (s[i] == '{') { if (depth == 0) start = i; depth++; }
        else if (s[i] == '}') {
            depth--;
            if (depth == 0) {
                const std::string obj = s.substr(start, i - start + 1);
                DesignEntry e;
                e.name = jStr(obj, "name", u8"(無名)");
                e.b = jNum(obj, "b", 0); e.W = jNum(obj, "W", 0);
                e.SM = jNum(obj, "SM", 0); e.Preq = jNum(obj, "Preq", 0);
                e.LD = jNum(obj, "LD", 0); e.dist = jNum(obj, "dist", -1);
                size_t stp = obj.find("\"st\":{");
                if (stp != std::string::npos) {
                    const std::string aircraft = jsonObjectAt(obj, stp + 5);
                    if (!aircraft.empty()) {
                        AirframeGraph layout;
                        const AircraftJsonReport report = aircraftFromJson(aircraft, e.st, &layout);
                        if (report.layoutAccepted) e.layout = std::move(layout);
                    }
                }
                v.push_back(std::move(e));
            }
        }
    }
    return v;
}

void saveDesigns(const std::vector<DesignEntry>& v, const std::string& path) {
    std::ofstream f(path.empty() ? savePath("designs.json") : path);
    if (!f) return;
    f << "{\"designs\":[\n";
    for (size_t i = 0; i < v.size(); i++) {
        const DesignEntry& e = v[i];
        char m[200];
        std::snprintf(m, sizeof(m), "\"b\":%.2f,\"W\":%.1f,\"SM\":%.1f,\"Preq\":%.0f,\"LD\":%.1f,\"dist\":%.0f",
                      e.b, e.W, e.SM, e.Preq, e.LD, e.dist);
        f << "{\"name\":\"" << e.name << "\"," << m << ",\"st\":"
          << aircraftToJson(e.st, e.layout ? &*e.layout : nullptr) << "}";
        f << (i + 1 < v.size() ? ",\n" : "\n");
    }
    f << "]}\n";
}

void fillMetrics(DesignEntry& e, const SimParams& prm) {
    Analysis a = analyze(e.st);
    PolarResult pol = computePolar(e.st, a, prm);
    e.b = e.st.span; e.W = a.W; e.SM = a.SM; e.Preq = a.Preq; e.LD = pol.bgY;
}

} // namespace bm
