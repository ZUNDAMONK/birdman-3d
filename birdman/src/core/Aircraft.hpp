#pragma once
// 機体解析 (JSの analyze / aeroConsts / computeLoads / computePolar / gustCalc)
#include "core/Types.hpp"

namespace bm {

double lerp(double a, double b, double t);
double clamp(double v, double lo, double hi);

// 翼弦長 (JS chordAt)
double chordAt(const AircraftParams& p, double t);

// 翼型データベース
const AirfoilData& airfoilOf(const AircraftParams& p);
const std::vector<AirfoilData>& airfoilDB();
// Re別極曲線の補間(log-Re線形)
double afCd0AtRe(const AirfoilData& af, double Re);
double afClmaxAtRe(const AirfoilData& af, double Re);
double reTableInterp(const double reT[4], const double vT[4], double Re);

// 翼形状 (JS wingGeom): S, MAC, AR
struct WingGeom { double S, MAC, AR; };
WingGeom wingGeom(const AircraftParams& p);

// 総合解析 (JS analyze)
Analysis analyze(const AircraftParams& st);

// 空力係数 (JS aeroConsts)
AeroConsts aeroConsts(const AircraftParams& st);

// 揚力線理論(Prandtl/Glauert モノプレーン方程式・奇数調和)。
// 平面形とねじり下げからスパン効率e・翼端失速CL・失速開始位置を解く。
// 後退角(sweep)は古典LLTの範囲外のため無視。
struct LLTResult {
    double e = 0.88;         // スパン効率(非粘性)。CLtargetでの値
    double CLmax3D = 1.3;    // どこかの翼素が2D clmaxに達する全機CL
    double tStall = 0.5;     // 失速開始スパン位置(0=翼根, 1=翼端)
    std::vector<double> ts, clAtStall;   // 失速CL時の局所cl分布(表示用)
};
LLTResult computeLLT(const AircraftParams& st, double CLtarget);

// Schrenk近似の荷重・たわみ (JS computeLoads)
LoadsResult computeLoads(const AircraftParams& st, const Analysis& a, double n);

// 揚力1Gと自重1gそれぞれ単独のたわみ曲線(線形重ね合わせ用, JS computeFlexBasis)
struct FlexBasis { std::vector<double> ys, dL, dW; };
FlexBasis computeFlexBasis(const AircraftParams& st, const Analysis& a);
double interpA(const std::vector<double>& xs, const std::vector<double>& ys, double x);

// ---- プロペラBEMT(ブレード要素+運動量理論) ----
// 直径・枚数・ピッチ・材質から代表的HPAブレードを生成し、
// 前進率J=V/(nD)に対する推力/パワー係数テーブルを作る(結果はキャッシュ)
struct PropTables { std::vector<double> J, Ct, Cp; };
PropTables computeBEMT(const AircraftParams& st);

// 軸出力Pshaft[W]と速度Vから回転数nps[rev/s]を二分法で解き、推力[N]を返す
double propThrust(const std::vector<double>& J, const std::vector<double>& Ct,
                  const std::vector<double>& Cp, double D, double rho,
                  double V, double Pshaft, double& nps);

// 速度Vで抗力Dragと釣り合う軸出力[W]を解く(設計解析用)
double propPowerFor(const std::vector<double>& J, const std::vector<double>& Ct,
                    const std::vector<double>& Cp, double D, double rho,
                    double V, double drag, double inst);

// 速度/L-Dポーラー (JS computePolar)
PolarResult computePolar(const AircraftParams& st, const Analysis& a, const SimParams& prm);

// 突風静解析 (JS gustCalc)
GustResult gustCalc(const AircraftParams& st, const Analysis& a, const SimParams& prm);

} // namespace bm
