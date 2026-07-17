#pragma once
// OpenGL 3Dレンダラ (Three.jsシーンの移植)
// 座標系: X=右翼方向, Y=上, Z=機尾方向(コースは-Z)。JS版と同一。
#include "core/Types.hpp"
#include "core/Aircraft.hpp"
#include "render/Camera.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <map>

namespace bm {

// 画像スプライト(第7弾): PNGをGLテクスチャ化し、アルファ有効範囲(bbox)のUVと
// クロップ後のアスペクト比を保持する
struct SpriteTex {
    unsigned id = 0;
    double u0 = 0, v0 = 0, u1 = 1, v1 = 1;   // アルファbboxクロップ後のUV範囲
    double aspect = 1;                       // クロップ後の 幅/高さ 比
    bool ok = false;
};

struct CrashParticle {
    glm::dvec3 pos, vel, spinAxis;
    double size, life, spin;
    bool water, debris;
    float r, g, b;
};
struct CrashRing { double delay, max, t = 0; glm::dvec3 pos; bool water; };

// 変形可能メッシュ(主翼のたわみ用)。頂点毎に基準位置+たわみ基底係数を持つ
struct FlexMesh {
    std::vector<float> base;   // 基準xyz (interleaved)
    std::vector<float> pos;    // 変形後xyz
    std::vector<float> nrm;    // 法線xyz
    std::vector<float> col;    // RGBA
    std::vector<float> dL;     // 頂点毎: 揚力1Gたわみ量
    std::vector<float> dW;     // 頂点毎: 自重1gたわみ量
    void clear() { base.clear(); pos.clear(); nrm.clear(); col.clear(); dL.clear(); dW.clear(); }
    size_t count() const { return base.size() / 3; }
};

class Renderer3D {
public:
    void init();                                  // GLステート初期化(コンテキスト作成後に1回)
    void resize(int w, int h);
    void buildAircraft(const AircraftParams& st, const Analysis& an);   // 機体メッシュ再構築

    // 1フレーム描画
    // mode: "design" or "flight"。機体変換はJSのaircraft.position/rotationに対応。
    void drawFrame(const std::string& mode, const SimParams& prm, const Camera& cam,
                   const glm::dvec3& acPos, const glm::dvec3& acRot, bool acVisible,
                   double propAngle,
                   const std::vector<glm::dvec3>& trail,
                   bool ghostVisible, const glm::dvec3& ghostPos, double ghostSpan);

    // クラッシュエフェクト (JS spawnCrashFx / tickFx / clearFx)
    void spawnCrashFx(const glm::dvec3& pos, bool water);
    void tickFx(double dt);
    void clearFx();
    bool fxActive() const { return !parts_.empty() || !rings_.empty() || colLife_ > 0; }

    // 翼たわみ (JS applyFlex): fl=揚力倍率, fi=自重倍率。駐機=(0,1)で翼端が垂れる
    void applyFlex(double fl, double fi);

    // 風の可視化: 実際の風場(基本風+地形風+サーマル)で移流する粒子。
    // drawFrameの後(GL行列が生きている間)に呼ぶ。center=注視域(機体位置), simT=サーマル場の時刻
    void drawWind(const SimParams& prm, const glm::dvec3& center, double z0, double simT);

    // 地面効果の可視化: 機体の影(低高度ほど濃い)+水面のさざ波リング
    void drawGroundFx(const glm::dvec3& acPos, double span, bool overWater);

    // 簡易機体(ライバル機・色付き半透明)
    void drawSimplePlane(const glm::dvec3& pos, double span, unsigned color);

    // 発進方向矢印(富士川の自由発進UI): 機体足元の地面に平たい半透明青の矢印。
    // drawFrameの直後(GL行列が生きている間)に呼ぶ。hdgRad=機首方位(コース前方基準)
    void drawStartArrow(const glm::dvec3& pos, double hdgRad);

    // 設計モード: 主翼のフラップ区間(内翼flapSpanFrac範囲の後縁)を緑半透明でハイライト。
    // drawFrameの直後(GL行列が生きている間)に呼ぶ
    void drawFlapHighlight(const AircraftParams& st, const Analysis& an);

    // サイト切替(biwa/fujikawa): 環境ジオメトリを再構築
    void setSite(const std::string& site);

private:
    void buildEnvironment();
    void buildEnvFujikawa();
    // ---- 画像素材の読み込み(第7弾) ----
    // スプライト: assets/textures/<name>.png を読み、アルファ透過化(白色キーは
    // アルファ無し画像のみ)+bboxクロップ+cutLeftFrac(左端切り捨て)を適用してGL化。
    // キャッシュされ2回目以降は再読込しない
    const SpriteTex& loadSprite(const std::string& name, double cutLeftFrac = 0);
    // タイル: GL_REPEAT+ミップマップ(水面・アスファルト用)。失敗時0
    unsigned loadTileTexture(const std::string& name);
    // 十字クワッド(直交2枚)/1枚クワッド(yaw指定)/水平置き1枚 の描画
    void spriteCross(const SpriteTex& t, const glm::dvec3& pos, double h);
    void spriteQuad(const SpriteTex& t, const glm::dvec3& pos, double h, double yawDeg);
    void spriteFlat(const SpriteTex& t, const glm::dvec3& pos, double size);
    std::map<std::string, SpriteTex> sprites_;   // 名前→テクスチャのキャッシュ
    unsigned biwaWaterTex_ = 0, asphaltTex_ = 0; // タイルテクスチャ(0=未読込/失敗)
    void drawLakeAndGrid(const glm::dvec3& camPos);
    void drawFx();
    void buildFunPlane();
    void drawGulls();                  // 水面上を旋回するカモメ(tickFxで時刻を進める)
    std::string site_ = "biwa";
    unsigned envList_ = 0, designList_ = 0;
    unsigned acOpaque_ = 0, acFilm_ = 0, propList_ = 0;
    unsigned funBody_ = 0, funProp_ = 0;   // お遊びモードの小型プロペラ機
    FlexMesh wingOpq_, wingFilm_;      // 主翼(たわみ変形対象)
    FlexBasis flexBasis_;
    long flexKey_ = -1;
    unsigned waterTex_ = 0;
    double propH_ = 2.0, propZ_ = 0.35;           // プロペラ取付位置(機体ローカル)
    int vpW_ = 1280, vpH_ = 720;
    std::vector<CrashParticle> parts_;
    std::vector<CrashRing> rings_;
    std::vector<glm::dvec3> windPts_;   // 風可視化粒子(ワールド座標)
    std::vector<CrashRing> ripples_;    // 地面効果さざ波
    double rippleTimer_ = 0;
    double gullT_ = 0;                  // カモメ周回の経過時間
    glm::dvec3 colPos_{0,0,0};
    double colLife_ = 0, colT_ = 0;
};

} // namespace bm
