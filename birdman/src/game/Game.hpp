#pragma once
// ゲームループ・状態管理 (JSのモード切替/simTick/startSim/stopSim等)
#include <SFML/Graphics.hpp>
#include <array>
#include "core/Types.hpp"
#include "render/Renderer3D.hpp"
#include "render/Camera.hpp"
#include "render/HUD.hpp"
#include "ui/DesignPanel.hpp"
#include "ui/DesignTools.hpp"
#include "ui/SettingsPanel.hpp"
#include "game/SaveData.hpp"
#include "game/Audio.hpp"
#include "game/Career.hpp"

namespace bm {

class Game {
public:
    int run();

private:
    static constexpr double RWY_START = 2000;
    void rebuildAircraft();
    void setMode(const std::string& mode);
    void placeForMode();
    void startSim();
    void stopSim();
    void simTick();
    void finishSim();
    void handleEvent(const sf::Event& ev);
    void setCtrl(int ax, double v);        // 0=e 1=ail 2=rud
    void adjustPower(double d);
    void setView(int which);               // 0=iso 1=front 2=side 3=top
    void setFPV(bool on);
    void drawUI();
    std::string buildFinMsg() const;

    // ---- 富士川の自由発進(第5弾): 発進前の位置クリック移動+Q/E方位回転 ----
    bool startPlaceActive() const;         // 配置UIが有効か(富士川・飛行モード・非シミュ中)
    void rotateStartHdg(double d);         // 機首方位を±15°刻みで回転
    // クリック位置→地面(y=0)交点→滑走路内なら発進位置を移動。成功でtrue
    bool placeStartFromClick(float mx, float my, float W, float H);

    // ---- 部位クリックでの設計パネル(左ドック+カメラズーム) ----
    // 機体ローカル座標(design_モード時はワールド座標と一致)での各部位の代表点
    std::array<glm::dvec3, (size_t)BodyPart::Count> bodyAnchors() const;
    // localをスクリーン座標へ射影。戻り値=カメラ後方などで無効ならfalse
    bool projectToScreen(const glm::dvec3& local, float W, float H, sf::Vector2f& out) const;
    // mから最近傍の部位を探す(半径内)。見つからなければ-1
    int hitTestPart(sf::Vector2f m, float W, float H, sf::Vector2f& anchorOut) const;

    // ---- カメラアニメ(部位ズーム/標準ビュー復帰) ----
    struct CamGoal { glm::dvec3 target; double theta, phi, dist; };
    CamGoal camGoalForPart(BodyPart part) const;   // 部位ごとのズーム目標(左ドック分の補正込み)
    CamGoal designViewGoal() const;                // 設計モード標準ビュー
    void animCamTo(glm::dvec3 target, double theta, double phi, double dist);
    void tickCamAnim();                            // frameDt_ぶん進める(毎フレーム呼ぶ)
    void selectPart(BodyPart part, sf::Vector2f anchor, float W, float H);

    // 部位選択ラベル(引き出し線+ピル)。drawUIで毎フレーム更新し、
    // handleEventのクリック判定は前フレームの矩形を使う(1フレーム遅れは実害なし)
    sf::FloatRect partLabelRects_[(size_t)BodyPart::Count]{};
    bool partLabelValid_[(size_t)BodyPart::Count]{};

    // フラップ操作トースト(Gキー時に1.5秒表示)
    std::string flapToast_;
    double flapToastT_ = 0;

    bool camAnimActive_ = false;
    double camAnimT_ = 0;
    static constexpr double CAM_ANIM_DUR = 0.55;
    double camFromTheta_ = 0, camFromPhi_ = 0, camFromDist_ = 0;
    glm::dvec3 camFromTarget_{};
    double camToTheta_ = 0, camToPhi_ = 0, camToDist_ = 0;
    glm::dvec3 camToTarget_{};
    bool designPanelWasOpen_ = false;   // 前フレームでdesign_が開いていたか(閉じた瞬間の検出用)

    sf::RenderWindow window_;
    Renderer3D r3d_;
    Camera cam_;
    HUD hud_;
    SaveData save_;
    Audio audio_;
    bool wasSparBroken_ = false;
    int prevTouchdowns_ = 0;
    DesignPanel design_;
    DesignTools tools_;
    SettingsPanel settings_;
    // 荷重たわみプレビュー(DesignToolsから設定、レンダラへ)
    double flexPrevN_ = 1.0;
    bool flexPrevApply_ = false;
    double flexFl_ = 0, flexFi_ = 1;   // 表示たわみの平滑値(構造の一次遅れ)

    AircraftParams st_;
    Analysis an_;
    SimParams prm_;
    SimResult simRes_;

    std::string appMode_ = "design";
    bool hudEnabled_ = false;

    // シミュ実行状態 (JS simPlay)
    bool simActive_ = false;
    double simWait_ = 0;
    FlightState live_;
    AircraftConstants liveC_;
    std::vector<glm::dvec3> trail_;
    double lastX_ = -99;
    double z0_ = 0;
    size_t ghostIdx_ = 0;
    bool ghostShow_ = false;

    // キャリア(年次大会)・チャレンジミッション
    void startMission(int i);
    void startContest(double cost);
    Career career_;
    bool inStartSim_ = false;     // startSim内部のstopSimでミッション/大会を畳まないため
    int missionActive_ = -1;
    bool contestActive_ = false;
    double contestCost_ = 0;
    // 大会ライバル機(オート飛行で同時競技)
    struct RivalPlane {
        std::string name;
        unsigned color;
        double span;
        AircraftConstants c;
        FlightState L;
        SimParams prm;
    };
    std::vector<RivalPlane> rivals_;
    void buildRivals();
    void stepRivals();
    AircraftParams stBackup_;
    SimParams prmBackup_;
    double appT_ = 0;                 // アプリ経過時間(非飛行時のサーマル場用)

    // フライトデブリーフ
    bool debriefOpen_ = false;
    Button bDebrief_, bDebriefClose_;

    // リプレイカメラ(直前フライトのシネマティック再生)
    void startReplay();
    void stopReplay();
    void replayTick();
    void adjustReplaySpeed(int dir);   // 再生速度段階変更(-1=遅く/+1=速く, 負値=逆再生)
    bool replayActive_ = false;
    double replayT_ = 0, replayDur_ = 0, replayScale_ = 1;
    double replaySpeedMult_ = 1.0;   // 再生速度倍率(ボタン/キーで調整)
    SimSample replayS_{};
    Button bReplay_, bReplayExit_, bReplaySlow_, bReplayFast_;

    // 実時間ベースのタイミング(現実1秒=ゲーム内1秒を保証。フレームレート非依存)
    sf::Clock frameClock_;
    double frameDt_ = 1.0 / 60;

    // パイロン周回コース(プラットフォーム発進時のみ)
    int courseLeg_ = -1;          // -1=無効 0=北パイロンへ 1=PFへ帰還 2=周回達成
    double lapTime_ = -1;
    std::string courseMsg_;
    double courseMsgTimer_ = 0;

    // 機体の表示状態
    glm::dvec3 acPos_{0, 0, 0}, acRot_{0, 0, 0};
    bool acVisible_ = true;
    double propAngle_ = 0;

    // ヘッダ/ビュー/飛行バーの永続ボタン
    Button bModeDesign_, bModeFlight_;
    Button bViews_[4], bFpv_, bHud_;
    Button bSettingsIcon_, bFlightExit_;
    Button bStart_, bReset_, bStop_;

    // カメラドラッグ
    bool dragging_ = false, dragPan_ = false;
    sf::Vector2i lastMouse_;
    sf::Vector2i pressPos_{-9999, -9999};   // 左ボタン押下位置(クリック判定: ドラッグ量<5px)
};

} // namespace bm
