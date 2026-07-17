#pragma once
// Phase 3 CFRP material grades. Internal units are SI; UI labels use GPa/MPa.
#include <string>
#include <vector>

namespace bm {

struct MaterialGrade {
    const char* id;
    const char* name;
    double density;       // kg/m^3
    double young;         // Pa
    double tensile;       // Pa
    double compressive;   // Pa
    double shear;         // Pa
    double costFactor;
};

inline const std::vector<MaterialGrade>& materialDB() {
    static const std::vector<MaterialGrade> db = {
        {"t700", u8"T700 標準高強度", 1550, 115e9, 1350e6, 650e6, 70e6, 1.0},
        {"t800", u8"T800 中間",       1550, 150e9, 1500e6, 700e6, 70e6, 1.4},
        {"m40j", u8"M40J 高弾性",     1600, 210e9, 1100e6, 450e6, 60e6, 1.9},
    };
    return db;
}

inline const MaterialGrade& materialOf(const std::string& id) {
    for (const auto& m : materialDB()) if (id == m.id) return m;
    return materialDB()[0];
}

inline const char* materialFromLegacyMod(double sparMod) {
    return sparMod <= 200 ? "t700" : sparMod <= 260 ? "t800" : "m40j";
}

constexpr double CFRP_DESIGN_K = 0.55;
constexpr double CFRP_G_OVER_E = 1.0 / 24.0; // gameplay-calibrated effective laminate shear modulus
constexpr double SPAR_MASS_CAL = 0.78;

} // namespace bm
