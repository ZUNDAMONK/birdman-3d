#pragma once
// サウンド: すべて手続き生成(アセット不要)。
// 風切り音=対気速度連動 / プロペラ音=rpm連動 / 着水・破壊・接地はワンショット。
#include <SFML/Audio.hpp>

namespace bm {

class Audio {
public:
    void init();
    // 毎フレーム: 飛行状態に応じてループ音の音量・ピッチを更新
    void update(bool flying, double V, double rpm, int propBlades, bool ground);
    void onSplash();
    void onCrack();      // 桁折損
    void onTouchdown();
    void setEnabled(bool on);
    bool enabled() const { return enabled_; }

private:
    bool ok_ = false, enabled_ = true;
    sf::SoundBuffer windBuf_, propBuf_, splashBuf_, crackBuf_, thudBuf_;
    sf::Sound wind_, prop_, splash_, crack_, thud_;
    static constexpr double PROP_F0 = 30.0;   // プロペラ音の基準周波数(ピッチ換算基準)
};

} // namespace bm
