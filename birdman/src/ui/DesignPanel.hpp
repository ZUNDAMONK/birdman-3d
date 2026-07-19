#pragma once
// 機体設計パネル: 部位クリックで開く左ドックパネル(幅360、画面左端固定)。
// テンプレート選択+スライダーは従来のsections_をそのまま流用し、
// 部位ごとに関連セクションだけを表示する。カメラは別途Game側でズームする。
#include "core/Types.hpp"
#include "core/Airframe.hpp"
#include "ui/Widgets.hpp"
#include <array>
#include <functional>
#include <map>

namespace bm {

// クリック可能な部位(Game側のヒットテストで使用)
enum class BodyPart { Wing = 0, Prop, Cockpit, HTail, VTail, TailBeam, Count };

class DesignPanel {
public:
    struct MountEditorCallbacks {
        std::function<bool(BodyPart, Mount&)> get;
        std::function<bool(BodyPart, const Mount&, std::string&)> apply;
        std::function<bool(BodyPart, std::string&)> reset;
        std::function<bool()> undo;
        std::function<bool()> redo;
        std::function<bool()> canUndo;
        std::function<bool()> canRedo;
    };

    // onChange: パラメータ変更時(機体再解析・再構築)
    void build(AircraftParams* st, std::function<void()> onChange,
               std::function<const Analysis*()> getAnalysis,
               MountEditorCallbacks mountCallbacks = {});

    // 左ドックパネルを開く/閉じる。anchorScreenは旧ポップアップAPIの名残(未使用)
    void openFor(BodyPart part, sf::Vector2f anchorScreen, float W, float H);
    void close();
    bool isOpen() const { return open_; }
    BodyPart currentPart() const { return part_; }

    // ドック矩形内のイベントのみ消費する。戻り値=消費したか
    bool handleEvent(const sf::Event& ev, sf::Vector2f mouse, float W, float H);
    void draw(sf::RenderTarget& rt, const sf::Font& font, float W, float H, float dt);

private:
    struct TplItem {
        std::string id, name, desc;
        std::function<void(AircraftParams&)> apply;
        Button btn;
    };
    struct TplGroup { std::string key; std::vector<TplItem> items; };
    struct Section {
        std::string title;
        std::vector<int> groups;           // groups_のindex
        std::vector<Slider> sliders;
        std::function<std::string()> note;
    };
    void relayout();
    Slider mkSlider(const std::string& label, double* field, double mn, double mx,
                    double stp, const std::string& unit, double mul = 1,
                    std::function<bool()> vis = nullptr);
    std::vector<int> sectionsForPart(BodyPart part) const;
    void updateDockRect(float W, float H);
    void loadMountEditor();

    AircraftParams* st_ = nullptr;
    std::function<void()> onChange_;
    std::function<const Analysis*()> getAn_;
    MountEditorCallbacks mountCb_;
    std::vector<TplGroup> groups_;
    std::vector<Section> sections_;
    std::map<std::string, std::string> tplSel_;
    float scroll_ = 0, contentH_ = 1000;

    // ポップアップ状態
    bool open_ = false;
    BodyPart part_ = BodyPart::Wing;
    std::vector<int> activeSections_;      // 表示中のsections_ index
    sf::FloatRect rect_{};                 // ポップアップ矩形(現在位置)
    float animT_ = 1.f;                    // 0=開き始め 1=完全表示(0.18秒でease-out)
    Button closeBtn_;
    Mount pendingMount_;
    std::array<Slider, 6> mountSliders_;
    Button mountMirror_, mountApply_, mountCancel_, mountReset_, mountUndo_, mountRedo_;
    bool mountEditable_ = false;
    std::string mountMessage_;
};

} // namespace bm
