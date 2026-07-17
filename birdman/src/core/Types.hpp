#pragma once
// Birdman3D C++ 移植 — 共通データ型
// リファレンス: Birdman3Dβ.html (Three.js版)。物理はJSと同一ロジック。
// JSのNumberはdouble精度のため、物理コアはdoubleで実装(数値一致性を優先)。
#include <string>
#include <vector>
#include "core/SiteConst.hpp"

namespace bm {

// ---- 機体パラメータ (JSの st 相当) ----
struct AircraftParams {
    // 主翼
    std::string planform = "taper";     // rect / taper / ellipse / crescent
    double span = 28, rootChord = 1.0, tipChord = 0.55;
    double wingX = 1.5, dihedral = 1.5, washout = 2.0, incidence = 3.0, sweep = 0;
    std::string airfoil = "dae31";
    std::string jig = "dihedral";       // dihedral / flat
    double ribPitch = 0.25;
    double plankTop = 30, plankBot = 30, plankRearTop = 0, plankRearBot = 0;
    // エルロン (JSでは st.mode)
    std::string ailMode = "none";       // none / small / large / allmove
    double ailSpanFrac = 0, ailChordFrac = 0;
    // プロペラ (JSでは st.config)
    std::string propConfig = "tractor"; // tractor / pylon / midboom / pusher
    std::string propMat = "balsa";      // balsa / carbon
    double propDia = 2.8;
    double propPitch = 3.6;             // 幾何ピッチ m/rev (BEMT用)
    int propBlades = 2;
    bool varPitch = false;              // 可変ピッチ機構(飛行中Z/Xでピッチ変更、+0.6kg)
    double flapSpanFrac = 0;            // フラップスパン(半翼比)。0=なし
    // パイロット
    std::string posture = "semi";       // upright / semi / recumbent
    double seatX = 1.85, pilotW = 60, cd0Add = 0.002, powerMax = 270;
    bool fairing = false;
    // 駆動
    std::string drive = "chain";        // chain / shaft
    double driveEffPct = 97;
    // 降着
    std::string gear = "tandem";        // none / tandem / tri / mono
    // 桁
    int segments = 4;
    double rootDia = 110, tipDia = 60;
    std::string sparMat = "t700";       // t700 / t800 / m40j
    // 水平尾翼
    std::string hShape = "taper";       // rect / taper / ellipse / swept / delta
    double hSpan = 3.4, hChord = 0.6, tailArm = 5.0;
    // 垂直尾翼
    std::string vShape = "swept";       // rect / swept / ellipse / delta / dorsal
    double vHeight = 1.3, vChord = 0.6;
    double elevRatio = 1.0, rudRatio = 1.0;
    double boomDia = 80;                 // CFRPテールブーム外径 mm（肉厚=d/80）
    // テールビーム翼
    std::string boomWing = "none";      // none / LR / L / R
    double boomWingSpan = 1.0, boomWingChord = 0.3, boomWingPos = 0.5;
};

// ---- 琵琶湖天候プリセット ----
struct WeatherPreset {
    std::string id, name, desc;
    double wind, gust, turb, therm, dirJit, prob;
};

// ---- シミュレーション設定 (JSの simPrm 相当) ----
struct SimParams {
    double P = 270, V0 = 7.0, wind = 0, xwind = 0;
    double pushV = 5.0;                  // プッシャー発進速度(滑走路モードの初速・対地)
    double wdir = 0, wspd = 0;
    double gust = 1.0, CLmax = 1.4, hTgt = 2.0;
    double verticalGust = 0;             // 実行時の瞬間鉛直突風 m/s（上向き+）。gustは最大振幅
    int    speed = 1;
    std::string mode = "platform";      // platform / runway
    std::string site = "biwa";          // biwa / fujikawa
    bool   auto_ = false;
    double wcap = 20, sens = 1.0;
    bool   hold = false;
    double temp = 20, turb = 0.3, tod = 7.0;
    bool   summer = true, ghost = true, pjit = false;
    double thermal = 0.0;
    bool   sixdof = false;              // 6自由度物理モデル(実験)
    bool   terrainWind = true;          // 地形風(比良おろし・岸サーマル)
    bool   windVis = true;              // 風の可視化(粒子表示)
    bool   stamina = false;             // 体力モデルを手動出力にも適用
    bool   realThermal = true;          // サーマル: 動的セル(OFF=JS互換の固定関数)
    bool   funPlane = false;            // お遊びモード: 小型プロペラ機(滑走路専用)
    // 夏モードの内部ゆらぎ (JSの simPrm._dirJit 等)
    double dirJit = 0, tempJit = 0, wspdJit = 0;
    double gustJit = 0;                 // 水平風速の速い変動(2-3秒スケール、無次元)
    double wspdMean = 0;                // 10分平均相当の風速(表示用。wspdは瞬間値)
    int    weather = -1;                // BIWA_WEATHERのindex。-1=未抽選
    // dt基準の決定的な気象更新。コピーしたSimParamsは同じシード・同じ経過時間なら
    // 同じ履歴を生成するため、プレイヤーと大会ライバルで環境系列を共有できる。
    unsigned weatherSeed = 0xB17D5EEDu;
    unsigned weatherRng = 0xB17D5EEDu;
    double weatherElapsed = 0;           // 発進後のシミュレーション経過秒
    double launchTemp = 20;              // 発進時の大気温度（飛行中の空力定数と表示を固定）
    bool atmosphereLocked = false;
    // 自由発進(富士川・滑走路モードのみ。琵琶湖プラットフォームは無視)
    // 滑走路再設計(850×30m, 南エンド=RWY36/海側 z=10, 北エンド=RWY18/内陸側 z=860):
    // デフォルトは南エンド(startPos=830)から北向き(startHdg=180)=
    // 「南端から滑走路全長を北へ向いて使う」配置
    double startPos = site::FUJI_START_POS_DEFAULT; // 滑走路に沿った発進位置 0〜840m(0=北端, 830=南エンド)
    double startHdg = 180;              // 発進機首方位(コース前方基準の度数。0=南向き, 180=北向き, ±180)
};

// ---- 解析結果 (JSの an = analyze(st) 相当) ----
struct MassItem { std::string n; double w, x; };
struct Analysis {
    double S = 0, MAC = 0, AR = 0, Sh = 0, Sv = 0;
    double wEmpty = 0, W = 0;           // W は kg(JSと同じ: W/g を格納)
    std::vector<MassItem> items;
    double xCG = 0, xAC = 0, xNP = 0, SM = 0, Vh = 0, Vv = 0;
    double V = 0, Preq = 0, margin = 0;
    int    nRibs = 0;
    double wSpar = 0, wJoints = 0, wBoom = 0, wGear = 0, wFairing = 0, wShaft = 0;
    double sparSF = 0, failStation = 0;  // failStation: 半翼スパン比 0..1
    std::string failMode;
    double baseDih = 0, bendDih = 0;
    double fusLen = 0, xHT = 0, xVT = 0, xProp = 0, wingLE = 0;
    double pilotCGx = 0, lh = 0, driveDist = 0;
};

// ---- 翼型データ ----
struct AirfoilData {
    const char* id; const char* name; const char* desc;
    double cl0, cd0, clmax, cm, reSens, thick;
    // Re別極曲線(XFOIL系の代表値): reT[k]における最小抗力cd0と最大揚力clmax。
    // log(Re)線形補間で使用。cd0/clmax(上)は設計代表点の値のまま残す
    double reT[4], cdT[4], clT[4];
};

// ---- 空力係数 (JSの aeroConsts() 相当) ----
struct AeroConsts {
    double CD0, e, propEff, CLmaxWing, cl0, cm, reSens, thick;
    double tipStall = 0.5;   // LLTの失速開始スパン位置(0=根, 1=端)
};

// ---- 空力定数パック (JSの aeroPack() 相当) ----
struct AircraftConstants {
    Analysis a;
    double rho = 1.225, g = 9.81, W = 0, m = 0, S = 0;
    double CD0 = 0, e = 0.88, AR = 0, propEff = 0.82, driveEff = 0.97;
    double CLmax = 1.4, Vs = 0, Vt = 0;
    double CLg0 = 0.8;                   // 地上滑走時の揚力係数(主翼取付角=迎角から算出)
    double A = 0;                        // プロペラ円盤面積
    double nFail = 2, VNE = 16;
    double nFailNeg = -0.5;              // 負荷重の折損限界(HPAはフィルム翼で-0.5G)
    double ailRate = 0, rudYaw = 0, rudRoll = 0, elevAuth = 0.65;
    double failStation = 0;
    std::string failMode;
    double CP = 270, Vdesign = 8, chordRef = 0.8, Vmp = 8, Pmin = 250;
    bool   hasGear = true;
    // stepSimが必要とする機体形状(JSはグローバルstを参照していた)
    double span = 28, propDia = 2.8;
    double wheelBase = 2.0;              // 地上ステア用の前後輪間隔(第6弾。aeroPackで胴体長から概算)
    // BEMTプロペラテーブル(J=V/nD → Ct, Cp)と取付損失係数
    std::vector<double> pJ, pCt, pCp;
    double propInst = 0.96;
    // 可変ピッチ: ピッチオフセット別テーブル(pPitOfs[k]に対するpCtT[k]/pCpT[k])
    bool varPitch = false;
    double propPitch0 = 3.6;             // 設計基準ピッチ m/rev
    std::vector<double> pPitOfs;
    std::vector<std::vector<double>> pCtT, pCpT;
    // フラップ(薄翼理論の増分。L.flap 0..1 を乗じて使用)
    bool hasFlap = false;
    double flapDCL = 0, flapDCLmax = 0, flapDCD = 0, flapCm = 0;
    // 翼型のRe別極曲線(断面cd0)と設計Reでの基準値。飛行中のRe効果に使用
    double afReT[4] = {1e5, 2e5, 4e5, 7e5};
    double afCdT[4] = {0.011, 0.011, 0.011, 0.011};
    double cdProfD = 0.0095;

    // ---- 6DOFモデル用(aeroPackで推定。標準モデルは使わない) ----
    double Ixx = 3000, Iyy = 300, Izz = 3300;    // 慣性モーメント kg·m²
    double CLa = 5.5;                            // 揚力傾斜 /rad
    double CLcruise = 1.0;                       // 巡航CL(トリム基準)
    double astall = 0.08;                        // トリム基準の失速迎角 rad
    double Cma = -0.8, Cmq = -30, Cmde = 0.1;    // ピッチ: 静安定・減衰・舵
    double Clp = -0.7, Clda = 0, Clb = 0.03, Cldr = 0.005;   // ロール
    double Cnb = 0.05, Cnr = -0.08, Cndr = 0.02;             // ヨー
    double CYs = 0.1;                            // 横滑り側力(β減衰)
    double tipStall = 0.5;                       // 失速開始スパン位置(LLT, 1=翼端)
    double dihBase = 1.5, dihBend = 1.0;         // 上反角: 治具角 + 1gたわみ分(deg)
    double heatFac = 1.0;                        // 暑熱によるCP低下係数
};

// ---- 飛行状態 (JSの L 相当) ----
struct FlightState {
    double t = 0, x = 0, h = 0, V = 0, gam = 0, psi = 0, phi = 0, yl = 0;
    double pathAir = 0;                  // 対気水平経路長(解析用。公式記録には使わない)
    double officialDist = 0;             // 発進点から現在地点までの対地水平直線距離
    double officialX0 = 0, officialYl0 = 0;
    bool   officialInvalid = false;      // 発進不成立(nogear/離陸前overrun)は記録0
    double e = 0, ail = 0, rud = 0;
    double eTgt = 0, ailTgt = 0, rudTgt = 0;
    double brake = 0;                    // 地上ブレーキ(0..1, キャッチャー相当)
    double pitchOfs = 0;                 // 可変ピッチのオフセット m/rev (Z/Xキー)
    double flap = 0, flapTgt = 0;        // フラップ展開量 0..1 (Gキーで0/½/1切替)
    double n = 1;
    double tz = 0, ty = 0;               // 乱流(有色ノイズ)
    unsigned turbRng = 0x13579BDFu;       // weatherSeed由来の乱流専用RNG（呼出順に非依存）
    double thrm = 0, thrmVz = 0;
    bool   inThermal = false, inThermalUsed = false;
    double wbal = 0;                     // 無酸素容量残 J(W': CP以下で回復する速攻バッテリー)
    double gly = 1.0;                    // 持久力(グリコーゲン残 0..1): 出力に比例して減り飛行中は回復しない
    double softHTgt = -1;                // -1 = 未初期化(JSのnull)
    bool   auto_ = false, done = false, ground = false;
    bool   sparBroken = false;           // 主桁・構造破壊(空中破壊運動/破断音の対象)
    bool   gearBroken = false;           // 着陸装置破損(高摩擦・再離陸禁止のみ)
    bool   crashed = false;              // 地面/水面への致命的衝突
    std::string failureMsg;
    double failStation = 0;              // 実際に破断した半翼位置 0..1
    std::string failMode;
    int    touchdowns = 0;
    double tdT = -1e9;                   // 直近の接地時刻(再離陸の猶予判定用)
    double maxBank = 0;
    double rpm = 0, nps = 0, Pnow = 0, etaP = 0, Re = 0;
    double pj = 0;                       // 出力ゆらぎノイズ(JSの L._pj)
    double ctlJit = 0;                   // 疲労による操舵ゆらぎ(体力モデルON時)
    // ---- 6DOFモデルの追加状態 ----
    double theta = 0;                    // ピッチ姿勢角(機体軸)
    double alpha = 0;                    // 迎角(トリム基準からの偏差)
    double beta = 0;                     // 機首の対進行方向オフセット(右+)
    double pRate = 0, qRate = 0, rRate = 0;   // 角速度
    bool init6 = false;                  // 6DOF初期トリム適用済みか
    double fatigue = 0;                  // 疲労蓄積(フラッター/繰返し荷重, 1.0で破断)
    double overNT = 0;                   // 限界荷重(nFail)超過の連続時間s(0.15s持続で折損。瞬間スパイクでは折れない)
    double autoVtgt = 0;
    // 走行・終了状態
    double groundRoll = -1;              // -1 = 未離陸(JSのnull)
    double rollDist = 0;                 // 地上滑走の累積距離m(自由方位発進のoverrun判定用)
    double vLat = 0;                     // クラブ着陸の残留横対地速度(接地時保存, τ≈0.6sで減衰。第6弾)
    double liftoffT = -1e9;              // 離陸瞬間の時刻(直後0.5秒のgam滑らか立ち上げ用。第6弾)
    bool   hardLanding = false;          // ハードランディング発生(UIトースト用。Game側で表示後クリア)
    bool   landed = false;               // 再着陸して滑走路上で停止(成功終了)
    bool   overrun = false, splash = false, offcourse = false, nogear = false;
    std::string impact;                  // "crash" / "splash"
    // シミュ管理
    double acc = 0, lastSample = 0;
    long   frame = 0;
    double z0 = 0;                       // 発進位置(ワールドz)。JSはsimPlay.z0
};

// 時系列サンプル (JSの simSample)
struct SimSample {
    double t, x, h, V, gam, n, yl, phi, officialDist, pathAir, psi;
};

// シミュ結果 (JSの simRes)
struct SimResult {
    std::vector<SimSample> out;
    double Vs = 10;
    bool   runway = false;
    double dist = 0, pathAir = 0, time = 0;
    bool   splash = false, overrun = false, offcourse = false, nogear = false;
    bool   landed = false;
    bool   sparBroken = false, gearBroken = false, crashed = false;
    double failStation = 0;
    std::string failMode;
    int    touchdowns = 0;
    double groundRoll = -1;
    double rollDist = 0;                 // 地上滑走の累積距離(overrunメッセージ用)
    std::string brkMsg;
    bool   auto_ = false;
    bool   summer = false;
    double tod = 7;
    double maxBank = 0;
    bool   inThermalUsed = false;
    bool   newBest = false;
    std::vector<std::string> badges;
};

// 荷重解析 (JSの computeLoads)
struct LoadsResult {
    std::vector<double> ys, Lp, LpE, M, defl;
    double tip = 0, Mroot = 0, Mallow = 0, SF = 0;
    double failStation = 0;
    std::string failMode;
    double baseDih = 0, bendDih = 0, dihEff = 0;
};

// ポーラー (JSの computePolar)
struct PolarResult {
    double Vs = 0;
    std::vector<std::pair<double,double>> pP, pLD;
    double mpX = 0, mpY = 0;   // 最小パワー点
    double bgX = 0, bgY = 0;   // 最良滑空(L/D最大)点
    double Dp = 0, Di = 0;
};

// 突風静解析 (JSの gustCalc)
struct GustResult {
    double Vs, V, margin, launchVa;
    bool launchOK;
    double dA, n, sfGust;
};

} // namespace bm
