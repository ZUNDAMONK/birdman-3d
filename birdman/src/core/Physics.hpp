#pragma once
// 飛行物理 (JSの groundEffect / aeroPack / stepSim)
#include "core/Types.hpp"

namespace bm {

// 乱数 [0,1) — JSのMath.random()相当
double frand();

// 地面効果(Wieselberger近似・修正版): CDi *= groundEffect(h, span)
double groundEffect(double h, double span);

// 発進台デッキ上面高(x<=0で後方ほど高い。x=0が前縁10.0m、x=-10が後端10.6m)
double deckHeightAt(double x);

// 空力定数パック (JS aeroPack)
AircraftConstants aeroPack(const AircraftParams& st, const Analysis& a, const SimParams& prm);

// お遊びモード: 小型プロペラ機(超軽量動力機)の定数パック。
// HPA設計解析を通さず、典型的な軽飛行機の値を直接構成する
AircraftConstants funPlaneConstants(const SimParams& prm);
// お遊び機のエンジン最大出力 W(スライダーP 0..700 をスロットル0..100%に読み替え)
static const double FUN_ENGINE_W = 35000.0;

// 乱流(有色ノイズ)・サーマルの更新 (stepSim/stepSim6共通)
void stepTurbulence(FlightState& L, const SimParams& prm, double dt);
void stepThermal(FlightState& L, const SimParams& prm);

// サーマル場の即時上昇流(平滑化なし)。風の可視化・地図表示用
double thermalCellVz(const SimParams& prm, double x, double yl);

// 1物理ステップ dt秒 (JS stepSim と同一ロジック。標準=3自由度+バンク)
void stepSim(FlightState& L, const AircraftConstants& c, const SimParams& prm, double dt);

// 6自由度モデル(実験): ピッチ/ロール/ヨーの回転動力学を持つ。
// 静安定余裕SM・尾翼容積・上反角が実際の動的挙動として現れる。
void stepSim6(FlightState& L, const AircraftConstants& c, const SimParams& prm, double dt);

// 発進直前の初期状態を作る (JS startSim の L 初期化部)
FlightState makeInitialState(const AircraftConstants& c, const SimParams& prm, double z0);

SimSample simSample(const FlightState& L);

} // namespace bm
