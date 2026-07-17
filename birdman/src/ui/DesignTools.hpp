#pragma once
// 設計モード下部ツールバー+ツールパネル
// (JSのtoolbar/toolpanel: 荷重・たわみ / ポーラー / 保存・比較 / リブ型紙)
#include "core/Types.hpp"
#include "core/DesignIO.hpp"
#include "game/Career.hpp"
#include "ui/Widgets.hpp"
#include "render/HUD.hpp"
#include <functional>

namespace bm {

class DesignTools {
public:
    struct Callbacks {
        std::function<void(const AircraftParams&)> onLoadDesign;   // 設計読込→再構築
        std::function<void(double n, bool apply)> onFlexPreview;   // たわみ3D表示
        std::function<double()> lastDist;                          // 直近シミュ距離
        // ミッション/キャリア
        Career* career = nullptr;
        std::function<void(int)> onStartMission;
        std::function<void(double cost)> onStartContest;
        std::function<bool(const std::string& id)> missionCleared;
    };
    void build(AircraftParams* st, SimParams* prm,
               std::function<const Analysis*()> getAn, Callbacks cb);
    bool handleEvent(const sf::Event& ev, sf::Vector2f m, float W, float H);
    void draw(sf::RenderTarget& rt, HUD& hud, float W, float H);
    // 荷重たわみプレビュー状態(Phase4の3D変形が参照)
    double loadN() const { return loadN_; }
    bool loadApply() const { return loadApply_; }

    static constexpr float BAR_H = 34, PANEL_H = 236;
    // 左ドック設計パネル(幅360)が開いているかをGameから毎フレーム伝える。
    // 開いている間はツールバー/ツールパネルをドックの右側(x>=360)だけに描画し、重なりを防ぐ。
    void setDockOpen(bool open) { dockOpen_ = open; }
    float DesignPanelLeft() const;
    bool panelOpen() const { return tool_ >= 0; }

private:
    void reloadDesigns();
    void syncDesignButtons();
    void exportRibsSVG();
    AircraftParams* st_ = nullptr;
    SimParams* prm_ = nullptr;
    std::function<const Analysis*()> getAn_;
    Callbacks cb_;
    int tool_ = -1;   // -1=閉, 0=荷重, 1=ポーラー, 2=保存比較, 3=リブ, 4=ミッション, 5=キャリア
    bool dockOpen_ = false;
    Button tb_[6];
    // ミッション
    std::vector<Button> msnBtns_;
    // キャリア
    Button bContest_;
    // 荷重・たわみ
    Slider ldN_;
    Button ldApply_;
    double loadN_ = 1.0;
    bool loadApply_ = false;
    // 保存・比較
    Button svSave_, svExportUrl_;
    std::string svMsg_;
    void exportJsUrl();
    std::vector<DesignEntry> designs_;
    std::vector<Button> svLoad_, svDel_;
    bool designsLoaded_ = false;
    float svScroll_ = 0;
    // リブ型紙
    Button rbExport_;
    std::string rbMsg_;
};

} // namespace bm
