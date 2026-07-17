#pragma once
// 琵琶湖天候システム (JSの BIWA_WEATHER / rollBiwaWeather / applySummer / applyWindVec / updateEnv)
#include "core/Types.hpp"

namespace bm {

const std::vector<WeatherPreset>& biwaWeather();

// その日の天候を確率で抽選(戻り値はbiwaWeather()のindex)
int rollBiwaWeather();

// 富士川飛行場(蒲原)の天候プリセット。駿河湾の海風/陸風+冬季の富士川おろしを想定。
const std::vector<WeatherPreset>& fujikawaWeather();

// サイト対応版: site=="fujikawa"ならfujikawaWeather()、それ以外はbiwaWeather()
const std::vector<WeatherPreset>& siteWeather(const std::string& site);
// サイト対応の天候抽選(戻り値はsiteWeather(site)のindex)
int rollWeather(const std::string& site);

// 風向・風速 → コース成分(wind=追い風+, xwind=右流され+)へ分解
void applyWindVec(SimParams& prm);

// 夏の琵琶湖: 時刻→気温・浜風・乱流・サーマルを自動設定
void applySummer(SimParams& prm);

// 発進時の気象状態を初期化する。seedが同じSimParams同士は同じ気象履歴になる。
// lockAtmosphere=trueでは温度・密度など発進時に構築した空力定数を飛行中固定する。
void resetWeatherState(SimParams& prm, unsigned seed, bool lockAtmosphere = true);

// シミュレーション時間dtに基づく環境更新。描画fpsや再生速度には依存しない。
void updateWeatherJitter(SimParams& prm, bool flying, double dt);

// 時刻→空の色(空・霧の色。早朝=淡金、日中=青、夕=橙)。0-255 RGB
void skyColor(double tod, int& r, int& g, int& b);

std::string todLabel(const SimParams& prm);
// psi=機首方位(飛行中はlive_.psiを渡すと向かい/追い風が機体基準で更新される)
std::string windLabel(const SimParams& prm, double psi = 0);

// ---- 琵琶湖の地形 ----
// 湖岸の閉多角形(コース座標系: x=前方(北向きコース), yl=右(東))。PFは(0,0)の東岸。
// 描画(Renderer)・着水/接地判定・風場で共有する。
const std::vector<std::pair<double, double>>& biwaShore();
bool insideLake(double x, double yl);

// ---- 富士川滑空場の地形 ----
// 富士川滑空場の水域(駿河湾+富士川)。コース絶対座標(x=前方(南), yl=右(西))
const std::vector<std::pair<double, double>>& fujikawaWater();
// サイト対応の水域判定(biwa=insideLake / fujikawa=fujikawaWater)
bool insideWaterSite(const SimParams& prm, double x, double yl);

// ---- 動的サーマル場(リアルサーマル) ----
// 湧いて消える円形セルの合成上昇流。位置・時刻から決定的に計算(状態レス)。
// x は発進点基準の絶対コース座標(= L.x - z0)
double thermalFieldVz(const SimParams& prm, double x, double yl, double t);
// 可視化用: (x,yl)近傍のアクティブセルを列挙して個数を返す
int thermalSitesNear(const SimParams& prm, double x, double yl, double t,
                     int maxN, double* sx, double* syl, double* sstr, double* srad);

// 位置依存の局所風(地形風)。基本風への加算分を返す。
//  - 比良おろし: 西岸(比良山地)の風下で下降流+東向き成分(天候「比良おろし」で激化)
//  - 岸サーマル: 東岸の陸地対流による上昇流帯(サーマル有効時)
// x は発進点基準の絶対コース座標(= L.x - z0)
void localWind(const SimParams& prm, double x, double yl, double h,
               double& addWind, double& addXwind, double& addVz);

} // namespace bm
