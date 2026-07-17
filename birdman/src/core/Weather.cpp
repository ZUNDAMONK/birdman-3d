#include "core/Weather.hpp"
#include "core/Physics.hpp"
#include "core/Aircraft.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace bm {

static const double PI = 3.14159265358979323846;

// ==========================================================================
// 琵琶湖 気象モデル(第4弾: 文献ベース再設計)
// 出典: 枝川尚資(1986)「琵琶湖上の気候特性について」地理学評論 Ser.A 59巻10号
//       p.589-605, DOI: 10.4157/grj1984a.59.10_589。
//       沖の白石(湖上の島)と湖上ボーリング塔での1年間(1982.7〜1983.7)の観測。
// この観測から得られた知見(本モデルの設計根拠):
//   (1) 湖陸の気温差は冬季夜間と春季昼間に最大(夜間は湖上が陸より暖かい=水の熱容量)
//   (2) 湖陸の湿度差は春季に最大(湖上が湿潤)
//   (3) 風速は湖上>陸上で、特に夜間の差が顕著(陸は夜凪になるが湖上は風が残る)
//   (4) 琵琶湖の湖陸風は「北西岸・北東岸・南東岸」の3系統の循環からなる
//   (5) 強風時の風は地形に規定される(湖盆軸NNE-SSW・周辺山地に沿って流れる)
// (1)(4)(5)はlocalWind()/applySummer()、(2)は早朝の靄(Renderer3D)、
// (3)はlocalWind()の夜間湖上増幅で反映。
// ==========================================================================

const std::vector<WeatherPreset>& biwaWeather() {
    static const std::vector<WeatherPreset> W = {
        // 確率は合計1.0。id文字列で分岐(index依存の大会/ミッションはCareer.cppのweather=0/1/3/4を維持)
        {"calm",    u8"朝凪・湖風待ち",       u8"湖風の立ち上がりが遅く弱い。じっくり基本を確認できる",        0.6, 0.5, 0.5, 0.3, 15, 0.28},
        {"normal",  u8"湖風日和",             u8"標準的な夏の日。南東・北東・北西の3系統の湖陸風が明瞭",      1.0, 0.9, 1.0, 0.4, 25, 0.32},
        {"swkaze",  u8"南西の強風(湖盆軸)",  u8"湖盆軸(南南西〜北北東)沿いの総観規模の強風。地形チャネリング大",2.6, 1.5, 1.4, 0.3, 20, 0.14},
        {"hira",    u8"比良おろし",           u8"比良山地からの吹き下ろし。突風率大、北西へチャネリングされやすい",2.0, 1.9, 1.6, 0.2, 40, 0.10},
        {"humid",   u8"不安定・対流活発",     u8"気温高く対流活発。岸サーマル強く、湖上は下降流が強め",         1.1, 1.4, 1.6, 0.6, 35, 0.16},
    };
    return W;
}

// ---- 富士川飛行場(蒲原)の天候プリセット ----
// 根拠: 富士川滑空場は富士川河口の右岸河川敷、滑走路方位は磁方位18/36
// (ほぼ南北)= 実測値(Wikipedia「富士川滑空場」等)。ゲーム内コースの
// 前方=南(真方位≈190°)とほぼ一致するため3D形状(滑走路・川)は変更していない。
// 風の実態はピンポイントのアメダス実況までは確証が取れなかったため、駿河湾沿岸の
// 一般的な海陸風パターン+局地風「富士川おろし」(冬季・山越えの吹き下ろし、
// 局地風として文献に記載あり)を基にフォールバック値を採用:
//  - 夏の日中: 駿河湾からの海風 南〜南西 3〜6 m/s(9〜10時立ち上がり、13〜15時ピーク)
//  - 夜間〜早朝: 陸風 北〜北北東 1〜2 m/s
//  - 冬: 富士川おろし 北寄り 5〜10 m/s、突風率高
//  - 谷筋(南北)に沿って風向が振れ、真横風は少ない
const std::vector<WeatherPreset>& fujikawaWeather() {
    static const std::vector<WeatherPreset> W = {
        {"clear_sea", u8"快晴・海風日和",     u8"駿河湾からの海風発達。昼は向かい風、朝夕は弱い陸風",       1.5, 1.0, 1.0, 0.7, 30, 0.32},
        {"calm_morn", u8"朝凪・微風",         u8"早朝は海陸風の入れ替わりで凪ぎ、穏やか",                   0.8, 0.6, 0.5, 0.4, 20, 0.22},
        {"oroshi",    u8"富士川おろし(北風強)", u8"山からの吹き下ろしで北寄りの強風・追い風気味。突風に厳重注意", 3.0, 2.4, 1.7, 0.3, 15, 0.12},
        {"cloudy",    u8"曇り・弱風",         u8"雲が広がり対流弱く、風も穏やか",                           1.0, 0.8, 0.7, 0.35, 25, 0.20},
        {"unstable",  u8"不安定・サーマル豊富", u8"大気不安定でサーマルが活発。風向が振れやすい",             1.6, 1.5, 1.3, 1.1, 50, 0.14},
    };
    return W;
}

static int rollFrom(const std::vector<WeatherPreset>& W) {
    const double r = frand();
    double acc = 0;
    for (size_t i = 0; i < W.size(); i++) {
        acc += W[i].prob;
        if (r <= acc) return (int)i;
    }
    return std::min<int>(1, (int)W.size() - 1);
}

int rollBiwaWeather() { return rollFrom(biwaWeather()); }

const std::vector<WeatherPreset>& siteWeather(const std::string& site) {
    return site == "fujikawa" ? fujikawaWeather() : biwaWeather();
}

int rollWeather(const std::string& site) { return rollFrom(siteWeather(site)); }

void applyWindVec(SimParams& prm) {
    const double th = prm.wdir * PI / 180;
    prm.wind = prm.wspd * -std::cos(th);   // 追い風+
    prm.xwind = prm.wspd * -std::sin(th);  // 右流され+(左から吹くと+)
}

void applySummer(SimParams& prm) {
    if (!prm.summer) return;
    const double t = prm.tod; // 5〜18時
    const bool fuji = prm.site == "fujikawa";
    if (prm.weather < 0) prm.weather = rollWeather(prm.site);
    const auto& list = siteWeather(prm.site);
    const WeatherPreset& wx = list[clamp(prm.weather, 0, (int)list.size() - 1)];

    // 気温: 琵琶湖=早朝22℃→午後34℃ピーク / 富士川(蒲原)=朝24℃→午後32℃ピーク
    // (どちらも sin((t-6)/12*PI) 型の概形。富士川おろしは冬季想定のためやや低めに補正)
    const double tempBase = fuji ? 24 + 8 * std::max(0.0, std::sin((t - 6) / 12 * PI))
                                  : 22 + 12 * std::max(0.0, std::sin((t - 6) / 12 * PI));
    const double tempOfs = fuji ? (wx.id == "oroshi" ? -8 : 0) : (wx.id == "humid" ? 2 : 0);
    prm.temp = std::round(tempBase + tempOfs + prm.tempJit);

    // 海風/湖風の発達度(0=夜間・陸風 / 1=ピーク)。
    //  琵琶湖: 彦根アメダスに校正(7-8月平年値。9時頃から発達→夕方減衰)
    //  富士川(蒲原): 駿河湾の海風は9時頃立ち上がり13〜15時ピーク、17時から減衰(フォールバック値)
    const double seaBreeze = fuji
        ? clamp((t - 9) / 5, 0.0, 1.0) * clamp(1 - (t - 17) / 3, 0.0, 1.0)
        : clamp((t - 9) / 4, 0.0, 1.0) * clamp(1 - (t - 16) / 3, 0.0, 1.0);
    // アメダスの数値は10分平均。実際の風は平均の周りをガスト/ラルが行き来する。
    // wspdMean=平均、wspd=瞬間値。wspdJit=遅い揺らぎ(~30秒)、gustJit=速い突風
    // (~2-3秒, updateWeatherJitterで更新)
    // 琵琶湖: 知見(4)に基づき、岸近傍の湖風増強はlocalWind()の3系統モデルが分担する設計に
    // 変更した(第4弾)。このためここの全域(場所非依存)係数は旧来値(0.9+2.2*seaBreeze)より
    // 縮小し、岸近傍ではlocalWind()の加算分と合わせて彦根アメダス校正レンジ(日中2.3-4.1m/s)
    // に収まるよう校正している(数値はphysics_testの[amedas]ブロックで検証)。
    const double peakCoef = fuji ? 2.2 : 0.9;
    prm.wspdMean = std::max(0.0, (0.9 + peakCoef * seaBreeze) * wx.wind);
    const double gustFac = clamp(1 + prm.wspdJit + prm.gustJit, 0.35, 1.95);
    prm.wspd = prm.wspdMean * gustFac;

    // 風向: 陸風⇔湖風/海風を発達度で補間。
    //  琵琶湖: 陸風(東南東115°) ⇔ 湖風(北西315°)。ただし「南西の強風」swkazeは
    //          海陸風サイクルと無関係な総観規模の風のため湖盆軸沿いのSSW(205°)で固定、
    //          「比良おろし」hiraは時刻に依存しない山越え吹き下ろしのためNW(315°)で固定。
    //  富士川: コース前方=南(真方位≈190°)。wdir 0°=向かい風=真方位190°から吹く風、
    //          180°=追い風=真方位10°(北)から吹く風、という変換になる。
    //          陸風(北・追い風185°) ⇔ 海風(南・向かい風25°、谷筋沿いで振れやすい)
    //          富士川おろし(北風)は季節風で海陸風サイクルと無関係に北寄り(≈180、追い風)で吹く
    double wdirBase;
    if (fuji) wdirBase = (wx.id == "oroshi") ? 180.0 : (185 - 160 * seaBreeze);
    else if (wx.id == "swkaze") wdirBase = 205.0;
    else if (wx.id == "hira") wdirBase = 315.0;
    else wdirBase = 115 - 160 * seaBreeze;   // 1→-45≡315°
    double wdirRaw = std::fmod(wdirBase + prm.dirJit + 360.0, 360.0);

    // 知見(5): 強風時の風は地形に規定される。wspdMeanが3.5m/sを超えたら、湖盆軸
    // (NNE=25°/SSW=205°)または比良おろしの吹き下ろし軸(NW=315°)のうち最寄りへ
    // 風向をブレンドする(6.5m/sで完全にチャネリング)。比良おろし天候時は常にNW優先。
    if (!fuji && prm.wspdMean > 3.5) {
        const double w = clamp((prm.wspdMean - 3.5) / 3.0, 0.0, 1.0);
        double target = 315.0;
        if (wx.id != "hira") {
            static const double axes[3] = {25.0, 205.0, 315.0};
            double best = 1e9;
            for (double ax : axes) {
                double diff = std::abs(std::fmod(wdirRaw - ax + 540.0, 360.0) - 180.0);
                if (diff < best) { best = diff; target = ax; }
            }
        }
        const double diff = std::fmod(target - wdirRaw + 540.0, 360.0) - 180.0;
        wdirRaw = std::fmod(wdirRaw + diff * w + 360.0, 360.0);
    }
    prm.wdir = std::round(wdirRaw);

    // 乱流・サーマル: 時刻 × 天候係数
    prm.turb = std::max(0.0, (0.2 + 0.7 * std::max(0.0, std::sin((t - 6) / 12 * PI))) * wx.turb);
    prm.thermal = std::max(0.0, (0.6 * std::max(0.0, std::sin((t - 7) / 11 * PI))) * wx.therm);
    prm.gust = wx.gust * (0.5 + seaBreeze);
    applyWindVec(prm);
}

void updateWeatherJitter(SimParams& prm, bool flying) {
    if (prm.summer && flying) {
        prm.tod = std::min(18.0, prm.tod + 0.00008 * prm.speed);
        if (prm.weather >= 0) {
            const auto& list = siteWeather(prm.site);
            const WeatherPreset& wx = list[clamp(prm.weather, 0, (int)list.size() - 1)];
            const double dj = wx.dirJit;
            prm.dirJit = prm.dirJit * 0.98 + (frand() - 0.5) * dj * 0.06;
            prm.tempJit = prm.tempJit * 0.99 + (frand() - 0.5) * 0.3;
            // 遅い揺らぎ(τ≈30秒, σ≈0.2): 数十秒単位の吹き寄せ/凪
            prm.wspdJit = clamp(prm.wspdJit * 0.9995 + (frand() - 0.5) * 0.024, -0.45, 0.45);
            // 速い突風(τ≈2秒): 突風の立ち上がり。天候のgust係数で強弱
            prm.gustJit = clamp(prm.gustJit * 0.992 + (frand() - 0.5) * 0.079 * (0.4 + 0.6 * wx.gust),
                                -0.7, 0.7);
        }
        applySummer(prm);
    }
}

void skyColor(double tod, int& r, int& g, int& b) {
    const double t = tod;
    const double day = clamp(std::sin((t - 5) / 13 * PI), 0.0, 1.0);
    const double morn = std::max(0.0, 1 - std::abs(t - 6) / 2);
    const double eve = std::max(0.0, 1 - std::abs(t - 17.5) / 2.5);
    r = (int)std::round(150 + 60 * day + 70 * eve);
    g = (int)std::round(180 + 30 * day + 10 * morn - 20 * eve);
    b = (int)std::round(210 + 20 * day - 40 * eve);
    r = std::min(r, 255); g = std::min(g, 255); b = std::min(b, 255);
}

std::string todLabel(const SimParams& prm) {
    const int h = (int)std::floor(prm.tod);
    const int m = (int)std::round((prm.tod - h) * 60);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d:%02d", h, m);
    return buf;
}

// ---- 動的サーマル場 ----
static unsigned thHash(int i, int j, unsigned k) {
    unsigned h = (unsigned)(i * 374761393) ^ (unsigned)(j * 668265263) ^ (k * 2246822519u);
    h ^= h >> 13; h *= 1274126177u; h ^= h >> 16;
    return h;
}
static double thRand(int i, int j, unsigned k) {
    return (thHash(i, j, k) & 0xffffff) / (double)0x1000000;
}
// グリッドサイト(i,j)のセル: 中心・半径・ライフサイクル包絡(0=非アクティブ)
static bool thCell(int i, int j, double t, double& cx, double& cyl, double& R, double& env) {
    const double G = 700.0;
    cx = i * G + (thRand(i, j, 1) - 0.5) * 500;
    cyl = j * G + (thRand(i, j, 2) - 0.5) * 500;
    R = 170 + thRand(i, j, 3) * 190;
    const double period = 420 + thRand(i, j, 4) * 240;   // 寿命サイクル7〜11分
    const double duty = 0.55;
    const double u = std::fmod(t / period + thRand(i, j, 5), 1.0);
    if (u >= duty) { env = 0; return false; }
    env = std::sin(PI * u / duty);                       // 湧く→ピーク→消える
    return true;
}

double thermalFieldVz(const SimParams& prm, double x, double yl, double t) {
    if (prm.thermal <= 0) return 0;
    const double G = 700.0;
    const int ci = (int)std::floor(x / G + 0.5), cj = (int)std::floor(yl / G + 0.5);
    double vz = -0.03 * prm.thermal;                     // セル間は弱い沈下域
    for (int i = ci - 1; i <= ci + 1; i++)
        for (int j = cj - 1; j <= cj + 1; j++) {
            double cx, cyl, R, env;
            if (!thCell(i, j, t, cx, cyl, R, env)) continue;
            const double d2 = (x - cx) * (x - cx) + (yl - cyl) * (yl - cyl);
            const double s = R * 0.62;
            vz += prm.thermal * (0.5 + 0.9 * thRand(i, j, 6)) * env * std::exp(-d2 / (2 * s * s));
        }
    return vz;
}

int thermalSitesNear(const SimParams& prm, double x, double yl, double t,
                     int maxN, double* sx, double* syl, double* sstr, double* srad) {
    if (prm.thermal <= 0) return 0;
    const double G = 700.0;
    const int ci = (int)std::floor(x / G + 0.5), cj = (int)std::floor(yl / G + 0.5);
    int n = 0;
    for (int i = ci - 4; i <= ci + 4 && n < maxN; i++)
        for (int j = cj - 4; j <= cj + 4 && n < maxN; j++) {
            double cx, cyl, R, env;
            if (!thCell(i, j, t, cx, cyl, R, env)) continue;
            const double str = prm.thermal * (0.5 + 0.9 * thRand(i, j, 6)) * env;
            if (str < 0.12) continue;
            sx[n] = cx; syl[n] = cyl; sstr[n] = str; srad[n] = R;
            n++;
        }
    return n;
}

const std::vector<std::pair<double, double>>& biwaShore() {
    // 琵琶湖近似形状(北湖が広く南東岸に発進地)。コース: PF(0,0)→北パイロン(11000,0)は湖上
    // 会場付近の岸線はx=-50(PFの50m後方)まで後退。発進プラットフォーム(0,0)は
    // 湖上に立つ構造物になり、PF周辺への墜落は着水判定になる(実際の鳥コンと同じ)。
    static const std::vector<std::pair<double, double>> P = {
        {-50, 900},      // PF右後方の岸(ここから北へ東岸)
        {2000, 2600},    // 東岸
        {7000, 3200},
        {12000, 3600},
        {17000, 2800},
        {21000, 500},    // 北東端
        {23000, -4000},  // 北端
        {20000, -9000},
        {15000, -12500}, // 北西
        {9000, -13500},  // 西岸(比良山地の麓)
        {3000, -12500},
        {-2000, -10000}, // 南西端
        {-4500, -6000},  // 南岸
        {-3500, -2500},
        {-1200, -900},   // 南東岸
        {-50, -250},     // PF左後方の岸
    };
    return P;
}

static bool pointInPoly(const std::vector<std::pair<double, double>>& P, double x, double yl) {
    bool in = false;
    for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++) {
        const double xi = P[i].first, yi = P[i].second;
        const double xj = P[j].first, yj = P[j].second;
        if ((yi > yl) != (yj > yl) && x < (xj - xi) * (yl - yi) / (yj - yi) + xi)
            in = !in;
    }
    return in;
}

bool insideLake(double x, double yl) {
    return pointInPoly(biwaShore(), x, yl);
}

const std::vector<std::pair<double, double>>& fujikawaWater() {
    // 第8弾: 静岡県航空協会 富士川滑空場の衛星写真に基づき再配置。
    // 滑走路(x=-860..-10, 850×30m再設計)の東側すぐ(近岸yl=-50一定、約38m)を富士川本流が並走し、
    // 河口(x≈-200〜1300、南=海側)に近づくほど東岸(遠岸)側が大きく広がって
    // 網目状の砂州(三角州)を形成する(-160→-700)。前方x>+1400が駿河湾。
    // Renderer3D::buildEnvFujikawaのriverBanks()ラムダ(視覚描画)と同一の区分線形にして
    // 見た目と当たり判定を一致させている
    static const std::vector<std::pair<double, double>> P = {
        {1500, -6000},                        // 海岸線・東端
        {1420, -500}, {1380, -50},            // 海岸を河口(近岸側)へ
        {-8500, -50},                          // 富士川 近岸(滑走路側=東岸)。河口〜上流端まで一定(-50)で並走
        {-8500, -160},                          // 上流端(対岸へ渡る)
        {-200, -160}, {1300, -700},            // 富士川 遠岸(東側): x<-200は一定(-160)、河口へ広がる(-700)
        {1450, 1600}, {1500, 6000},            // 河口西側から海岸線・西端へ
        {12000, 6000}, {12000, -6000},         // 沖(南)の外周
    };
    return P;
}

bool insideWaterSite(const SimParams& prm, double x, double yl) {
    if (prm.site == "fujikawa") return pointInPoly(fujikawaWater(), x, yl);
    return insideLake(x, yl);
}

// ---- 琵琶湖 局所風モデル(第4弾)のヘルパー ----

// 点(x,yl)から線分(ax,ayl)-(bx,byl)への最短距離(有限線分。両端でクランプ)
static double distToSegment(double x, double yl, double ax, double ayl, double bx, double byl) {
    const double dx = bx - ax, dyl = byl - ayl;
    const double len2 = dx * dx + dyl * dyl;
    double t = len2 > 1e-9 ? ((x - ax) * dx + (yl - ayl) * dyl) / len2 : 0.0;
    t = clamp(t, 0.0, 1.0);
    const double px = ax + t * dx, pyl = ayl + t * dyl;
    return std::sqrt((x - px) * (x - px) + (yl - pyl) * (yl - pyl));
}

static double distToPolyline(const std::vector<std::pair<double, double>>& pts, double x, double yl) {
    double best = 1e18;
    for (size_t i = 0; i + 1 < pts.size(); i++)
        best = std::min(best, distToSegment(x, yl, pts[i].first, pts[i].second,
                                             pts[i + 1].first, pts[i + 1].second));
    return best;
}

// 湖岸全周(biwaShore())への最短距離。知見(3)の夜間湖上風速増幅で「最寄り岸からの距離」に使う
static double distToShoreGeneric(double x, double yl) {
    const auto& P = biwaShore();
    double best = 1e18;
    for (size_t i = 0, j = P.size() - 1; i < P.size(); j = i++)
        best = std::min(best, distToSegment(x, yl, P[j].first, P[j].second, P[i].first, P[i].second));
    return best;
}

// 知見(4)の3系統湖陸風circulationの発達度カーブ。
// 昼(湖風,+): 9時(calmは10時)立ち上がり→13-15時ピーク(=1)→17時に0へ減衰
// 夜(陸風,-0.4倍で符号反転): 19時〜翌7時。7-9時/17-19時は遷移区間
static double lakeBreezeDev(double t, bool calmWx) {
    const double onset = calmWx ? 10.0 : 9.0;
    const double mul = calmWx ? 0.6 : 1.0;   // 「朝凪・湖風待ち」は発達度そのものが弱い
    double day;
    if (t < onset) day = 0.0;
    else if (t < 13.0) day = clamp((t - onset) / (13.0 - onset), 0.0, 1.0);
    else if (t < 15.0) day = 1.0;
    else if (t < 17.0) day = 1.0 - (t - 15.0) / 2.0;
    else day = 0.0;
    double night;
    if (t >= 19.0 || t < 7.0) night = -0.4;
    else if (t < 9.0) night = -0.4 * (1.0 - (t - 7.0) / 2.0);
    else if (t >= 17.0 && t < 19.0) night = -0.4 * ((t - 17.0) / 2.0);
    else night = 0.0;
    return mul * day + night;
}

// 知見(3)夜間の湖上風速増幅: 陸は夜凪(風が弱まる)が湖上は風が残るため、夜ほど・沖ほど強まる
static double nightnessOf(double t) {
    if (t < 7.0 || t > 18.0) return 1.0;
    if (t < 9.0) return 1.0 - (t - 7.0) / 2.0;
    if (t > 16.0) return (t - 16.0) / 2.0;
    return 0.0;
}

void localWind(const SimParams& prm, double x, double yl, double h,
               double& addWind, double& addXwind, double& addVz) {
    addWind = addXwind = addVz = 0;
    if (prm.site == "fujikawa") {
        // 谷風の収束: 川筋(yl≈200..1000)沿いは風が谷に沿って加速
        const double valley = clamp(1 - std::abs(yl - 550) / 900, 0.0, 1.0);
        if (h < 200) addWind += -0.25 * prm.wspd * valley;   // 海風時は向かい風が強まる向き
        // 西岸(yl>1500)の山地: 斜面下降流
        const double tw2 = clamp((yl - 1500) / 2500, 0.0, 1.0);
        addVz += -0.6 * (0.3 + prm.wspd / 5.0) * tw2;
        // 河原サーマル: 砂利の河川敷(滑走路周辺の陸地)は対流が強い。
        // 陸上は安定層が高く、湖上(60m)より高い150mスケールで減衰
        if (prm.thermal > 0 && !insideWaterSite(prm, x, yl)) {
            addVz += 0.8 * prm.thermal * clamp(h / 8.0, 0.3, 1.0)
                   * (0.35 + 0.65 * std::exp(-h / 150.0));
        }
        return;
    }
    // 天候id(indexではなく文字列で判定。プリセットの並び替えに対して頑健にするため)
    const auto& wlist = biwaWeather();
    const std::string wid = (prm.weather >= 0 && prm.weather < (int)wlist.size())
                                 ? wlist[prm.weather].id : std::string("normal");
    const bool calmWx = wid == "calm";
    const bool hiraWx = wid == "hira";

    // ---- 知見(4): 3系統の湖陸風循環(南東岸=彦根側/北東岸=長浜・湖北側/北西岸=高島・比良側) ----
    // 各系統は「代表岸線(biwaShoreの該当区間)」+「湖から岸へ吹き付ける向き(origin)」で定義。
    // 寄与 = 岸からの距離による減衰 exp(-d/2500) × 発達度(lakeBreezeDev)。
    // 3系統のベクトル和をaddWind/addXwindへ加算(湖心部では3系統がほぼ0に減衰し、基本風が支配的になる)。
    {
        static const std::vector<std::pair<double, double>> seShore = {
            {-4500, -6000}, {-3500, -2500}, {-1200, -900}, {-50, -250},
            {-50, 900}, {2000, 2600}, {7000, 3200}, {9500, 3400},
        };
        static const std::vector<std::pair<double, double>> neShore = {
            {9500, 3400}, {12000, 3600}, {17000, 2800}, {21000, 500}, {23000, -4000},
        };
        static const std::vector<std::pair<double, double>> nwShore = {
            {20000, -9000}, {15000, -12500}, {9000, -13500}, {3000, -12500},
        };
        struct Sys { const std::vector<std::pair<double, double>>* shore; double originDeg; };
        static const Sys systems[3] = {
            {&seShore, 315.0},   // 南東岸系(会場・彦根側): 湖風は北西から
            {&neShore, 225.0},   // 北東岸系(長浜・湖北側): 湖風は南西から
            {&nwShore, 90.0},    // 北西岸系(高島・比良側): 湖風は東から
        };
        const double AMP = 2.3;   // 各系統の岸際・ピーク時の強さ(m/s)。PF付近13時で局所寄与≈2〜3m/s、
                                   // 夜間陸風≈1m/sに校正(applySummer側の全域係数縮小と分担)
        const double dev = lakeBreezeDev(prm.tod, calmWx);
        for (const auto& s : systems) {
            const double d = distToPolyline(*s.shore, x, yl);
            const double decay = std::exp(-d / 2500.0);
            const double m = dev * decay * AMP;
            if (std::abs(m) < 1e-6) continue;
            const double th = s.originDeg * PI / 180.0;
            addWind += m * -std::cos(th);
            addXwind += m * -std::sin(th);
        }
    }

    // ---- 知見(3): 夜間の湖上風速増幅(陸は夜凪になるが湖上は風が残る) ----
    // 基本風(prm.wind/xwind)に対する倍率増幅として加算分のみをここへ足す
    // (基本風自体はapplySummer側の値のまま。ここは位置依存の追加分)
    if (insideLake(x, yl)) {
        const double offshore = clamp(distToShoreGeneric(x, yl) / 2000.0, 0.0, 1.0);
        const double nightness = nightnessOf(prm.tod);
        const double amp = 1.0 + 0.35 * offshore * (0.4 + 0.6 * nightness);
        addWind += (amp - 1.0) * prm.wind;
        addXwind += (amp - 1.0) * prm.xwind;
    }

    // 比良おろし: 西岸側(yl < -5000)で山越えの吹き下ろし。
    // 天候「比良おろし」で顕著、他天候でも弱く存在
    const double hira = hiraWx ? 1.0 : 0.2;
    const double tw = clamp((-5000 - yl) / 6000, 0.0, 1.0);
    if (tw > 0) {
        const double str = hira * (0.4 + prm.wspd / 5.0);
        addVz += -1.1 * str * tw;                 // 下降流
        addXwind += 0.9 * str * tw;               // 東(コース側)へ押し出す
        addWind += 0.2 * str * tw;
    }
    // 岸サーマル: 東岸の陸地(yl > +1200)に近づくほど対流上昇流が強い
    // (湖上サーマルより強い陸上対流の再現。低高度では地面効果域で減衰、
    //  高高度は安定層で減衰=際限ない上昇を防ぐ)
    if (prm.thermal > 0) {
        const double te = clamp((yl - 1200) / 1800, 0.0, 1.0);
        addVz += 0.55 * prm.thermal * te * clamp(h / 8.0, 0.3, 1.0)
               * (0.35 + 0.65 * std::exp(-h / 70.0));
    }

    // ---- 知見(1): 気温・サーマルの湖陸差(湖面上のみ。insideLake()で判定) ----
    // 湖上気温は夜+1.5℃/昼-2℃程度(枝川1986)。気温表示自体は彦根(陸)基準のまま据え置き、
    // ここでは対応する弱い鉛直流のみを反映する(乱流は増やさない)。
    if (insideLake(x, yl)) {
        const double tod = prm.tod;
        if (tod >= 5.0 && tod <= 7.5) {
            // 早朝: 湖面が相対的に暖 → 水面上に弱い上昇(5-7.5時内でピークは中央付近)
            const double f = clamp(1.0 - std::abs(tod - 6.25) / 1.25, 0.0, 1.0);
            addVz += 0.10 + 0.10 * f;             // 0.10〜0.20 m/s
        } else if (tod > 7.5 && tod < 19.0) {
            // 日中: 湖面は冷源 → 水面上に弱い下降(不安定・対流活発な日はより顕著)
            const double f = clamp(std::sin((tod - 7.5) / (19.0 - 7.5) * PI), 0.0, 1.0);
            const double humidMul = wid == "humid" ? 1.5 : 1.0;
            addVz -= (0.05 + 0.05 * f) * humidMul;   // 0.05〜0.10(humidは〜0.15) m/s
        }
    }
}

std::string windLabel(const SimParams& prm, double psi) {
    // 風は世界固定(コース軸のwind/xwind)。機首方位psiに対する相対の「風が吹いてくる向き」を
    // 求める(機体が向きを変えれば向かい風⇔追い風も変わる)。風向計drawWindArrowと同じ計算
    const double fwd = prm.wind * std::cos(psi) + prm.xwind * std::sin(psi);
    const double rgt = -prm.wind * std::sin(psi) + prm.xwind * std::cos(psi);
    const double fromDeg = std::atan2(-rgt, -fwd) * 180.0 / PI;   // 0=機首正面から
    const double d = std::fmod(fromDeg + 360.0, 360.0);
    static const char* names[8] = {
        u8"向かい風", u8"右前から", u8"右横から", u8"右後ろから",
        u8"追い風", u8"左後ろから", u8"左横から", u8"左前から"};
    char buf[80];
    // 夏モードは「平均(瞬間)」表示。アメダスの数値=10分平均に合わせた表記
    if (prm.summer && prm.wspdMean > 0.05)
        std::snprintf(buf, sizeof(buf), u8"%s 平均%.1f(瞬間%.1f) m/s",
                      names[(int)std::round(d / 45) % 8], prm.wspdMean, prm.wspd);
    else
        std::snprintf(buf, sizeof(buf), "%s %.1f m/s", names[(int)std::round(d / 45) % 8], prm.wspd);
    return buf;
}

} // namespace bm
