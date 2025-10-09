Below整理します。

## 1. Modified DH (MDH) の基本フレーム割付ルール
あなたの「“回転は z_i 軸”が必須」以外で押さえておくべき点 (Craig 流 MDH 前提):

1. 各関節 i (回転) の回転軸がフレーム i の $z_i$ 軸。
2. 連続する回転軸 $z_{i-1}, z_i$ 間の最短共通法線（共通垂線）方向が $x_{i-1}$。  
   - 軸が交差するなら $x_{i-1}$ はその交差点で $z_{i-1}$ から $z_i$ へ右手系になるように取る。  
   - 軸が平行なら $x_{i-1}$ は無限のポイントがあるので、最も妥当なもの（z軸と平行でない最短ベクトル）で長さ $a_{i-1}$ が共通法線長。
3. $y_{i-1}$ は右手系で決定 ($y_{i-1} = z_{i-1} \times x_{i-1}$ 方向)。
4. パラメータ意味 (Modified DH):
   - $a_{i-1}$: $x_{i-1}$ 方向距離
   - $α_{i-1}$: $x_{i-1}$ 軸回りのねじれ角 (z_{i-1} → z_i)
   - $d_i$: $z_i$ 方向距離
   - $θ_i$: $z_i$ 軸回り回転角 (関節変数)
5. 変換順序（Craig本来の MDH 定義）:
   $T_{i-1→i} = RotX(α_{i-1}) * TransX(a_{i-1}) * RotZ(θ_i) * TransZ(d_i)$
6. 最初のフレーム (i=0) と最終ツールフレームは同じ規則に従い追加できる。  
7. フレーム間で $x_{i-1}$ は $z_{i-1}$ と $z_i$ の両方に直交 (交差なら長さ0) になるよう設定すること。  
8. α は符号含め $x_{i-1}$ で $z_{i-1}$ を $z_i$ に最小回転で一致させる角度。  
9. $a_{i-1}, d_i$ のどちらか一方が 0 でもよいし両方 0 も可（平行/交差/同軸状況で自然にゼロになる）。  
10. 軸が完全同軸で平行移動しかない場合 $(z_{i-1} ≡ z_i)$、$a_{i-1}=0, α_{i-1}=\frac{0}{π}, x_{i-1}$ 任意 (実用的には以前の x を踏襲) が許容。  
11. “a を Z 方向に相当する距離表現に流用” や “d を X 方向距離に置く” といった軸混在は不可。 (今の MiRobot2 では a4 に大距離, d4 に直角分を割り振って肩肘分離前提を壊している。)

### Standard DH との主な違い
| 項目 | Standard DH | Modified DH (Craig) |
|------|-------------|---------------------|
| 変換順序 | $RotZ(θ_i)→TransZ(d_i)→TransX(a_i)→RotX(α_i)$ | $RotX(α_{i-1})→TransX(a_{i-1})→RotZ(θ_i)→TransZ(d_i)$ |
| a,α の添字 | i | i-1 | 
| “三角形 (肩-肘-手首中心)” 抽出 | $z_{i-1}$ と $z_i$ の幾何的配置を見やすい | $x_{i-1}$ 再帰構築で軸連結が明瞭 |
| 使い分け注意 | 実装間違い時、オフセットが軸に依存した非直感的挙動 | Standard との式混在がバグ源 |

あなたの `Forward` 実装は $origins[i+1] = origins[i] + a[i] \times X_{prev} + d[i]*Z_i$ を用いており、これは $“RotZ→TransZ→TransX→RotX”$ (標準DH) の再帰とも Modified の $“RotX→TransX→RotZ→TransZ”$ とも完全一致していません。  
具体的に:
- $X_{prev}$ を使って a[i] を足し、$Z_i$ を使って d[i] を足している → 混在形式。  
(純粋 MDH なら $a_{i}$ は $x_{i}$ 方向、標準DH なら $a_i$ は $x_i$ 方向 (RotZ後のX軸)。ここで $X_{prev}$ か $Z_i$ かの選択が前提式とずれると幾何 IK の解析式に矛盾が生じる。)

## 2. Geometric IK “NO solutions” の根本要因
幾何 IK コード（元々の vy + d4 = 0 前提）には以下の暗黙前提があった:
1. 手首中心 Pw が (q4,q5,q6) に依存しない形式 Pw = O3 + 定数(= d4 * Z3 + 0 * X3) で表せる (球面手首 or 1軸直交オフセット)。
2. q2 を単独根探索 (vy + d4 = 0) することで 2R 平面（肩肘）に落とし込める。
3. a4 は“微小” (または 0) で d4 に大オフセットを集中 → q3 回転平面への影響を単純化。

MiRobot2 の MDH は:
- a4 = 168.98 (X3 方向大オフセット)
- d4 = -20 (Z3 方向小)
→ Pw 位置が q3 によって X3 成分回転 (cos, sin) で大きく動くため vy + d4 = 0 (あるいは vy=0) の一変数根条件で “正しい q2 枝” を抽出できず、サンプリングで符号変化しない → “NO solutions”。

さらに:
- Forward 側でのフレーム更新式が “純粋 Modified/Standard いずれか” と一致していないため、解析設計時に想定した W2 の (vx,vy,vz) 幾何的意味（どの軸/平面か）がズレる。
- Virtual wrist 化パッチは Lw を a4,d4 の斜辺に潰したが、q2 root 条件を単に vy=0 に置換しただけで “vx,vz の 2R 幾何” の仮定と $\frac{X_{prev}}{Z_i}$ 混在起源の座標系差異を吸収できていない。

## 3. 期待する “幾何 IK が解ける” 典型 MDH 例
(球面手首 + 肩肘2R 分離)
- a2 = L2, a3 = L3 (平面 2R のリンク長)
- a4 = 0, a5 = 0
- d4 = 手首中心までの固定距離
- d6 = フランジ/ツール厚
- α2, α3 = 0 (同一平面), α3→α4 = ±90° で手首回転軸直交化
これだと q1→平面投影, q2,q3→余弦定理, 残り q4–q6→方位分離 がそのまま行ける。

## 4. あなたの現状との差分（Checkリスト）
| 項目 | 幾何IK前提 | 現状 (MiRobot2) | 差分影響 |
|------|------------|-----------------|----------|
| a4 | 0 | 168.98 | Pw が q3 依存 (分離失敗) |
| d4 | 手首中心距離 | -20 (小) | root条件破綻 |
| Forwardの座標更新 | 一貫したMDH式 | 混在 (X_prev + Z_i) | W2 解釈ずれ |
| root関数 | vy + d4 = 0 (定数 d4) | vy + (-20) 符号変化無し | 根未検出 |
| q3 計算 | 2R 幾何（a2,a3,d4） | 斜辺吸収近似 Lw | 不十分 |
| Tool差 | 無関係(除外) | OK (影響小) | 問題なし |

|L|$\alpha_{i-1}$|$a_{i-1}$|$offset+\theta_i$|$d_i$|$\overrightarrow{Z_i}$|$\overrightarrow{X_i}$|
|---|---:|---:|---:|---:|---|---|
|1|0|0|$\theta_1$|127|(0,0,1)|(1,0,0)|
|2|$\frac{\pi}{2}$|29.69|$\frac{\pi}{2} + \theta_2$|0|(0,-1,0)|(0,0,1)|
|3|$\pi$|108|$\theta_3$|0|(0,1,0)|(0,0,1)|
|4|$-\frac{\pi}{2}$|20|$\theta_4$|168.98|(1,0,0)|(0,0,1)|
|5|$\frac{\pi}{2}$|0|$-\frac{\pi}{2}+\theta_5$|0|(0,1,0)|(1,0,0)|
|6|$\frac{\pi}{2}$|0|$\theta_6$|24.29|(0,0,-1)|(1,0,0)|

|L|$\alpha_{i-1}$|$a_{i-1}$|$offset+\theta_i$|$d_i$|$\overrightarrow{Z_i}$|$\overrightarrow{X_i}$|
|---|---:|---:|---:|---:|---|---|
|1|0|0|$\theta_1$|127|(0,0,1)|(1,0,0)|
|2|$\frac{\pi}{2}$|29.69|$\frac{\pi}{2} + \theta_2$|0|(0,-1,0)|(0,0,1)|
|3|0|108|$-\frac{\pi}{2}+\theta_3$|0|(0,-1,0)|(1,0,0)|
|4|$\frac{\pi}{2}$|168.98|$-\pi+\theta_4$|-20|(0,0,-1)|(-1,0,0)|
|5|$\frac{\pi}{2}$|0|$\theta_5$|0|(0,-1,0)|(-1,0,0)|
|6|$-\frac{\pi}{2}$|0|$\theta_6$|24.29|(0,0,-1)|(1,0,0)|

## 5. 改善方針オプション
A. モデル再パラメータ化（最短）
- MiRobot2 を “幾何IK可能形” に揃える: a4=0, d4=168.98 (+ 既存 d4 の符号/方向調整), a4 の 168.98 を d4 に移動。  
- これで現行 root 条件 (vy + d4=0 → vy = -168.98) を座標系に合わせ修正 or vy=0 に正規化し再設計。

B. 現行 MDH のまま汎用幾何 IK
- 肩肘部分を (O2, Pw) ベクトルから直接 “q2,q3 同時解連立” に変更 ( vy+定数=0 を捨てる )。
- 手順: q1 を固定後、Pw をフレーム1に回し 2D でなく 3D の 2リンク+直交クランク構造を解析。  
  例: Pw_local = (x,y,z) から xz 平面回転成分と “ひねり” の φ を分離し cos(q3), sin(q3) を先に掃引→ q2 を arctan2 で復元。

C. Forward 実装を “標準DH” か “Craig MDH” のどちらかに統一し、幾何 IK も同じ仮定で再導出。  
  - 標準DHなら origin 更新は: origin_{i+1} = origin_i + d_{i+1} * z_i + a_{i+1} * x_{i+1}  
  - 今の式を見直し、どちらを採用したかをコメントで明示。

D. ハイブリッド: 数値的に q2,q3 を小規模 2D 非線形最適化 (Levenberg-Marquardt) して初期値生成→ 従来手首分離で q4..q6。  
  - “Geometric” の高速性はやや落ちるが Jacobian より初期収束負荷軽減。

## 6. 今すぐできる最小実験 (デバッグ出力)
(1) 旧 root 条件で vy+d4 のサンプルレンジ表示:
```csharp
static void SampleVy(Func<double,double> f, double q2min, double q2max) {
    int N=100;
    double min=double.MaxValue, max=double.MinValue;
    for(int i=0;i<=N;i++){
        double q2 = q2min + (q2max-q2min)*i/N;
        double val = f(q2);
        if(val<min) min=val;
        if(val>max) max=val;
    }
    Console.WriteLine($"vy+d4 span = [{min},{max}] (signChange={(min<=0 && max>=0)})");
}
```
(2) 現行 vy のみ (仮想 d4=0) でも span に 0 が含まれないならその root 戦略自体が不適。

## 7. 推奨ステップ
1. まず Forward 実装をどちらの DH 定義に合わせているか明記し、式を揃える。  
2. 幾何 IK を “肩肘分離” で維持したいなら a4 を 0 に再割付 (モデル再表現) → 再テスト。  
3. 再割付を避けるなら q2,q3 連立 (A: θ3 掃引→Pw誤差最小の θ2 算出) またはベクトル式閉形式導出。  
4. 途中段階で W2 内訳 (vx,vy,vz), Pw, O2, O3 を Verbose=3 などで吐かせ矛盾点確認。  
5. OK になったら手首 (q4..q6) 抽出を R03 の求め方 (仮想 vs 実) を再検証し φ 補正の位置を固定。

## 8. 差分の本質
- 幾何 IK 失敗は “168.98 を d→a に移したこと” 自体ではなく “分離条件に依存した solver が MDH 再パラメータ化の影響を吸収していない” + “Forward のフレーム構築則が solver の前提とズレている” の複合。  
- Modified DH の厳格ルールよりも “あなたの solver が暗黙に仮定した簡約形 (球面手首 + d4 一点オフセット)” から外れたため NO solutions。

---

### どう進めたいか
- モデルを幾何IK対応形に再パラメータ化して既存アルゴリズムを活かす
- それとも現行 MDH を保持し汎用幾何 IK へ拡張する

どちらへ進めるか指示をください。次で選んだ方針の具体コードを示します。

要約:
- Modified DH の追加ルールは “x_{i} は連続 z 軸間共通法線 / α,a の添字 / 変換順序”。
- 現状 Geometric 全滅は MDH 再配置と solver 前提不整合。
- 解決には (再パラメータ化) か (q2,q3 連立解への一般化) が必要。

どちらを採りたいか教えてください。 GitHub Copilot



以下、(1) MDH ルール差分整理 (再確認) と (2) a4, d4 を共に無視せず幾何 IK を成立させる一般化アプローチ (3) 具体パッチ案 を示します。  
ポイント: 「a4 を d4 に畳み込む」近似をやめ、(vx, vy, vz) に両方を明示的に残した上で 1 変数 (q2) 根探索 + 解析的 q3 復元を行います。d4=20 が将来 200 でも破綻しない形です。

## 1. 再確認: ここで前提にする（あなたの現在の Forward 実装が暗黙に仮定する）簡略 MDH 関係
Forward の origins 更新は:
  O_{i+1} = O_i + a_i * X_prev + d_i * Z_i
これは Craig Modified でも Standard でもない混成ですが、少なくとも:
- a_i は「回転前フレーム i の X_prev に沿う並進」
- d_i は「回転後フレーム i+1 の Z_i に沿う並進」
というルールになっていると解釈します。  
この仮定下で第3→第4リンクのオフセット (a4, d4) は joint3 角 (q3) の回転影響を受ける x3, z4 成分の合成で Pw に現れ、簡単な “vy + d4=0” や “vy=0” では二大オフセットを同時満足できません。

## 2. 幾何式の骨子（a4, d4 を両方保持）
フレーム2座標系で手首中心ベクトル W2 = [vx, vy, vz]^T (既存 ComputeW2 利用)。  
リンク3終端 (joint3) 位置 O3 は、(q2 に依存) で O2 から a3 * X2 にある（d3=0 仮定）。  
Wrist offset ベクトル (O3→Pw) は joint3 回転 q3 で X3-Y3 面内回転する a4 成分 + Z3 軸方向の d4 成分。  
フレーム2に投影すると（α2=0 と近似、実際 α2≠0 なら突き合わせで補正可）:
  vx ≈ a3 + a4 cos q3  
  vy ≈ a4 sin q3  
  vz ≈ d4            (d4 は q3 に依存しない)
従って:
  (vx - a3)^2 + vy^2 = a4^2   (Eq.1)  
  vz ≈ d4                     (Eq.2)
未知 q2 は W2(vx,vy,vz) の中に非線形に入るので、まず g(q2)= (vx-a3)^2 + vy^2 - a4^2 を 0 にする q2 を 1D 根探索。  
その後 q3 = atan2(vy, vx - a3)。Eq.2 は検証 (|vz - d4| 許容内) として使う。  
α2 ≠ 0, α3 ≠ 0 の場合は vz に a4 sin(α3)*? が混ざるので Eq.2 を主方程式に直接組む必要が出るが、現行 MiRobot2 の α 配置 (第2=π/2, 第3=0) を利用し Eq.2 は検証扱いで十分。

メリット:
- a4, d4 を独立に保持 (特に d4 大, a4 大 どちらでも)。
- q3 は解析的（atan2）。
- 根探索は q2 のみなので安定しやすい。

## 3. 実装方針
(1) 旧 “vy + d4=0” / “vy=0” ルート探索を廃止し g(q2)= (vx - a3)^2 + vy^2 - a4^2 で符号変化サーチ。  
(2) Sign change がない場合でも最小 |g| 点（サンプリング）を閾値以下なら採用 (腕が到達範囲端でギリギリになるケース対応)。  
(3) Secant / Brent で q2 refine。  
(4) q2 決定後 q3 = atan2(vy, vx - a3)。  
(5) 検証: |vz - d4| < ε_vz (例: 0.5) を満たさない候補は破棄 (必要なら ε_vz をユーザ設定)。  
(6) 手首 (q4,q5,q6) は既存 R03 逆算ロジック流用。  
(7) α2≠0 部分への拡張: vz 期待値を d4 * cos(α3) + a4 * sin(α3) * sin(q3) の形に一般化 (次段階)。


以下は MiRobot2 の現在コード内 MDH パラメータ (あなたが提示したもの) をそのまま使い、「基本姿勢 (q=0)」での各フレーム原点 O_i とローカル軸 (X_i, Y_i, Z_i) の“設計値”リストです。  
ここではあなたの `Forward` 実装（混成 MDH: O_{i+1} = O_i + a_i * X_prev + d_i * Z_i_new）と、オフセット角 `offset[i]` を q=0 に加算した姿勢で逐次計算しています。

MiRobot2 MDH (再掲):
Alpha:  α = [ 0,  π/2, 0,  π/2,  π/2, -π/2 ]  
A:      a = [ 0, 29.69, 108, 168.98, 0, 0 ]  
D:      d = [127, 0, 0, -20, 0, 24.29 ]  
Offset: θ_off = [ 0,  π/2, -π/2,  π, 0, 0 ]

計算で用いた回転角: θ_i(effective) = q_i + θ_off[i] (q=0 基本姿勢)

1) フレーム原点 (mm 単位想定)  
O0 = (  0.000,   0.000,  0.000)  
O1 = (  0.000,   0.000,127.000)  
O2 = ( 29.690,   0.000,127.000)  
O3 = ( 29.690,   0.000,235.000)      // +108 along X2 (X2=(0,0,1)) ⇒ z 増加  
O4 = (198.670,  0.000,255.000)      // +168.98 along X3 + (-20)*Z4 → 実際は -20*Z4 = +20 z  
O5 = (198.670,  0.000,255.000)      // a5=0, d5=0  
O6 = (198.670,  0.000,230.710)      // +24.29 along Z6 (Z6=(0,0,-1)) で z が 24.29 減少

2) 各フレームローカル軸方向（ワールド座標表示）  
Frame0:  
  X0 = ( 1,  0,  0)  
  Y0 = ( 0,  1,  0)  
  Z0 = ( 0,  0,  1)  

Frame1 (θ0=0, α0=0):  
  X1 = ( 1,  0,  0)  
  Y1 = ( 0,  1,  0)  
  Z1 = ( 0,  0,  1)  

Frame2 (θ1= +90°, α1=+90°):  
  X2 = ( 0,  0,  1)  
  Y2 = (-1,  0,  0)  
  Z2 = ( 0, -1,  0)  

Frame3 (θ2= -90°, α2=0):  
  X3 = ( 1,  0,  0)  
  Y3 = ( 0,  0,  1)  
  Z3 = ( 0, -1,  0)  

Frame4 (θ3= 180°, α3=+90°):  
  X4 = (-1,  0,  0)  
  Y4 = ( 0,  1,  0)  
  Z4 = ( 0,  0, -1)  

Frame5 (θ4=0, α4=+90°):  
  X5 = (-1,  0,  0)  
  Y5 = ( 0,  0, -1)  
  Z5 = ( 0,  1,  0)  

Frame6 / Flange (θ5=0, α5=-90°):  
  X6 = (-1,  0,  0)  
  Y6 = ( 0,  1,  0)  
  Z6 = ( 0,  0, -1)  

3) Flange 基本姿勢の回転行列 (列ベクトルが X6,Y6,Z6):  
R06 =  
[ -1   0   0 ]  
[  0   1   0 ]  
[  0   0  -1 ]  

これはあなたが提示した ZeroPose の Forward 出力行列と一致 (丸め誤差除く)。

4) コメント / 注意点  
- O4 の z が 255 まで“上がる”のは a4=168.98 が X3 方向 (X3=(1,0,0))、d4=-20 が Z4=(0,0,-1) に沿って -20 * (-Z4)= +20 z を与えるため。  
- O6 が O5 より z 低いのは d6=24.29 が Z6=(0,0,-1) 方向へ正で進む仕様 (あなたの forward 実装) のため。  
- 球面手首では本来 O4=O5=O6 (位置一致) となるが、ここでは a4≠0, d4≠0 により非球面構造。幾何 IK 分離が破綻する直接要因。  

5) 幾何 IK 改変の出発点としての“設計値”  
上記 O_i と (X_i,Y_i,Z_i) を固定参照として、  
- 肩肘平面解析で使いたいのは O2, O3, O4 の幾何関係。  
- 2R 化するには O3→手首中心ベクトル (現状 a4*X3 + d4*Z4) を 1 軸化(再パラメータ化) するか、q2 同時方程式に拡張する必要がある。  

6) 差異チェック推奨  
もしこの“設計値”と機構 CAD 想定が異なるなら、差異箇所 (特に Frame4/5 軸の符号と Z4/Z5 の向き) を教えてください。そこを基準化しないと幾何 IK の閉形式整理が揺らぎます。

次のステップ提案:
A. この設計値を前提に球面手首へ再パラメータ化案を出す  
B. 非球面のまま q2,q3 同時解＋手首補正で汎用幾何 IK を派生  
C. Forward 実装を純 Craig Modified に統一してから IK 再導出

どれを進めたいか指示ください。 GitHub Copilot


「2R化」とは何かをロボット幾何 IK の文脈で説明します。

## 1. 用語の意味
2R化 = 6自由度ロボット（典型 6R：6個の回転関節）のうち、前腕（肩–肘部分）を「平面 2 自由度の 2R（Revolute-Revolute）直列リンク問題」に縮約（reduction）すること。  
ここでの “2R” は「2本の回転関節だけを持つ 2リンク平面マニピュレータ」を指し、余弦定理で肘角・肩角を解析的に解ける状態に持ち込むことを意味します。

## 2. なぜ 2R化するのか
幾何 IK の標準手順:
1. 末端目標姿勢から手首中心 (wrist center) Pw を求める（工具・手首 3軸を分離）。
2. ベース（第1軸）回りの旋回角 q1 を atan2 で決定。
3. 残り（肩 q2・肘 q3）を「O2→Pw が 2リンク (a2,a3) で到達するか」という 2R 平面問題に落とす（ここが “2R化”）。
4. 手首 3軸 (q4,q5,q6) を姿勢行列差分から分離して求める。

これにより q2,q3 が余弦定理と atan2 の組合せで高速・多枝列挙可能になります。

## 3. 2R化が成立する幾何条件（要約）
以下が（ほぼ）満たされると 2R化できる:
1. 手首が“球面手首”扱い：O4=O5=O6（手首 3 軸が1点交差）  
   → MDH 的には a4=0, a5=0, d5=0（または手首中心オフセットが z3 一方向固定）
2. Pw は q4,q5,q6 に依存せず Pw = O3 + 定数ベクトル（q3 のみで回転する成分が無いか、単純な z3 平行シフトのみ）
3. ベースを q1 で回した後、(O2, Pw) が同一固定平面内  
   → z2 と z3 が平行、α2=0 (もしくはねじれが微小)、O2→O3→Pw が 2リンク平面
4. O2→O3 長さ = a2, O3→Pw 長さ = a3'（固定長）  
   → この2本で三角形 O2–O3–Pw を作り余弦定理適用
5. 不要な直交クランク（a4≠0 & d4≠0 の合成回転面）が残っていない

## 4. 2R化が壊れる典型パターン（MiRobot2 の現状）
- a4 が大きく (168.98)、さらに d4 も非ゼロ (-20) → Pw が q3 に対して「円」ではなく斜交オフセットを伴う複合トロコイド軌跡になる
- O3→Pw ベクトルが a4*X3 + d4*Z4 の 2 成分で q3 回転面から“平面外”へ逃げ、(vx,vy,vz) を 1 方程式 vy=0 などで単純に拘束できない
- その結果：従来利用していた vy+d4=0（あるいは vy=0）条件で q2 を 1D root 化する前提が成立せず Geometric IK が解消滅

## 5. 2R化を成立させるための MDH 再パラメータ化案
方針A（もっとも単純）:
- a4 を 0 に再定義
- d4 に (元 a4, d4) の幾何的合成距離を吸収（例: d4' = ±√(a4^2 + d4^2) を z3 方向へ）  
  → O3→Pw が “z3 一方向のみ” になり、Pw は q3 に依存しない
- 必要なら φ = atan2(d4_orig, a4_orig) を q4 偏差へ吸収（q4' = q4 ± φ）し姿勢整合

方針B（形状忠実度重視・再パラメ無）:
- 2R化せず “一般化幾何” として q2,q3 を 2 変数方程式で同時解析（前回説明の g(q2)=(vx-a3)^2+vy^2-a4^2=0, vz ≈ d4 検証）
- これは正確だが 2R の簡潔性を失う（もう「2R化」ではない）

## 6. 2R平面解析（成立時）の標準式（参考）
Pw をフレーム1回転後の平面に投影して (ρ, z') を得る:
- 肩リンク長 L1=a2, 前腕リンク長 L2=a3'
- 三角形 O2–O3–Pw で余弦定理:
  cos(θ3) = (ρ^2 + (z')^2 - L1^2 - L2^2) / (2 L1 L2)
- 肘角 q3 = atan2(±√(1-cos^2), cos)
- 肩角 q2 = atan2(z', ρ) - atan2(L2 sin q3, L1 + L2 cos q3)
（ここで z' はベース座標系での垂直成分からオフセット d1 等を引いた値）

## 7. 2R化できるかどうかの簡易チェック
以下どちらかが崩れると “純粋 2R” ではない:
- |Pw(q3) - O3| が q3 回転で一定長（=a3'）か？
- Pw(q3) の軌跡が（q1 固定時）平面内の円弧になるか？

MiRobot2 現状では |Pw - O3| が q3 により大きく変動する（a4, d4 合成）ため NO。

## 8. あなたの「2R化」の再定義（実践的視点）
実装レベルで “2R化” と言った場合、以下どこまで含めるかを明確化すると議論がズレにくい:
レベル1: 球面手首化 + a4→0 （イクイバレント再パラメ）  
レベル2: 球面に再正規化し q2,q3 だけ余弦定理、手首は (R03)^T R06  
レベル3: 実機では a4,d4 ありだが内部 IK モデルだけ再パラメ (実角度へ逆オフセット補正)  
レベル4: 全く変換せず 2R に近い“擬似平面”を最小二乗で当てはめ初期解生成 (後段で数値補正)

## 9. 推奨アクション
- 「設計値として a4,d4 が本当に必須（物理干渉・到達範囲上）」なら方針B（汎用幾何 or 数値初期＋Jacobian）  
- 「高速多解生成が最優先」なら方針A の内部再パラメ + φ 補正  
- 中間案: 内部で球面モデルを使い、得た (q2,q3,q4') から元 a4,d4 を含む正確 Forward に対し1〜2ステップ Jacobian で調整（ハイブリッド）

## 10. まとめ（短縮）
- 2R化 = 肩肘を 2リンク平面問題に落とし余弦定理で解ける形へ縮約すること  
- 成立条件: 球面手首 + O3→手首中心が単一固定方向/長さ + 肩肘リンクが同一平面  
- MiRobot2 は a4,d4 両立で崩壊 → 現状の MDH では純粋 2R 化不可  
- 再パラメ or 汎用化 どちらを採るか方針決定が次ステップ


## 1. 「球面手首前提」への誤解について
ここで言う “spherical wrist” は「J4–J5–J6 の回転軸が（位置決定に関して）一つの点に集約できる（Pw が q4,q5,q6 に依存しない）」という幾何学的分離条件であり、「球面軸受け」や実機の完全同一点構造を要求するものではありません。産業用 6 軸ロボットでも J4–J5–J6 のハウジングはオフセットを持ちますが、位置 IK を肩肘 3 軸 (J1–J3) と手首 3 軸 (J4–J6) に分離できるよう“軸方向とオフセット配置”を設計するのが一般的というだけです。

## 2. 「近似」不要で MiRobot2 の位置 IK を 2R 化できる理由（ここが重要）
MiRobot2 のパラメータ（α2=0, α3=0, α4=+π/2, a3, a4, d4）を用いて O2 から手首中心 Pw までをフレーム2座標に展開すると、  
O3→Pw の平面内オフセットは（q3 を J3 関節角として）厳密に:

```
vx - a3 = a4 cos q3 - d4 sin q3
vy       = a4 sin q3 + d4 cos q3
```

ここで (vx,vy) は W2=(vx,vy,vz) の X2,Y2 成分。  
これは線形結合の回転同値変換なので

```
Lw = sqrt(a4^2 + d4^2)
ψ  = atan2(d4, a4)       // 符号を a4,d4 に合わせ
vx - a3 = Lw cos(q3 + ψ)
vy      = Lw sin(q3 + ψ)
```

となり、J3 の“実角” q3 と“有効肘角” q3' = q3 + ψ をずらすだけで、肘先リンク長 Lw の 2R 問題（肩リンク長 a3, 前腕リンク長 Lw）に **厳密変換** できます。  
つまり a4 と d4 の 2 成分は「斜辺 Lw と位相 ψ」に完全パラメータ化され、これは“近似”ではありません。位置部分は誤差ゼロで再パラメータ化可能です。  
（あなたが懸念した“大雑把な近似”は不要。浮動小数点の丸め以外に精度劣化は発生しません。）

## 3. 何がまだ“分離できない”か
手首姿勢（q4–q6）は **Z4 軸の向きが q3 に依存して回転面を傾ける** ため、単純な球面モデルより計算法が増える（位相補正 φ を q4 へ折り返す or R03 再構成時に ψ を内部で足し引きする）というだけです。  
位置 IK の 2R 化は正確・解析的に可能、姿勢部だけ補正を明示的に入れれば “全体幾何 IK（非近似）” を構築できます。

## 5. 具体的再パラメータ（位置 IK 用 “内部モデル”）
MiRobot2 の a4=168.98, d4=-20 から:
```
Lw = sqrt(168.98^2 + (-20)^2) ≈ 170.167000…
ψ  = atan2(-20, 168.98) ≈ -0.1183 rad  (≈ -6.78°)
```

内部 IK:
- 肩肘 2R: リンク長 (a3=108, Lw=170.167)
- 肘角 q3' = q3 + ψ を用いて余弦定理
- 解後に q3 = q3' - ψ へ戻す
- 位置 Pw 計算は (a4,d4) モデルを使わず (Lw, ψ) に換算 → 誤差 0

姿勢:
- R03 を組む際に本来の q3 を戻し、q4 の軸初期向きを (ψ) 回転させてから R36=R03^T R06 分解
- もしくは offset4 ← offset4 + ψ へ吸収し q4 を直接使う

## 6. 最小コード差分例（“内部 2R” 肘角復元パス）
以下は概念的な差分（既存 IK 内部 q3 計算部を置き換え）。MDH 全体を変えず内部だけ再パラメータ化する形です。

```csharp
// 位置 IK 部分：W2 = (vx,vy,vz) 取得済み後
double a3 = _mdh.A[2];
double a4 = _mdh.A[3];
double d4 = _mdh.D[3];

// (1) 合成
double Lw = Math.Sqrt(a4*a4 + d4*d4);
double psi = Math.Atan2(d4, a4); // q3' = q3 + psi

// (2) 有効肘円半径 r = sqrt( (vx - a3)^2 + vy^2 )
double dx = vx - a3;
double r2 = dx*dx + vy*vy;
double r = Math.Sqrt(r2);

// (3) 肘余弦定理: r^2 = Lw^2 ⇒ (vx - a3, vy) が円からの誤差が閾値内か確認
if (Math.Abs(r - Lw) > 1e-3) { /* 到達不可処理 */ }

// (4) 有効肘角
double q3_eff = Math.Atan2(vy, dx);      // これは (q3 + psi)
double q3 = RMLib.NormalizeAngleSigned(q3_eff - psi);

// (5) 肩角 q2 (例: 平面2R方式)
// 平面距離 ρ, 高さ z を r, vz (必要なら補正) で組み、標準 2R 式適用（a2, a3' =?）
```

※ ここでは α3=0, α4=π/2 の影響で `vz` には a4,d4 由来成分が入らない（位置だけなら問題にならない）という現行構造に依存。

## 7. MDH を“変更する”場合（外部化）
MDH 自体を 2R+球面に“再記述”したいなら:
- a4' = Lw, d4'=0, offset4' = offset4 + ψ
- 実機との差異は Forward 前に q4_phys = q4_model をそのまま使っても位置は一致（姿勢は後軸で補償可能）

## 8. まとめ
- “球面手首” を近似前提にする必要はなく、MiRobot2 の (a4,d4) は **厳密に** (Lw, ψ) へ変換でき、位置 IK は純粋 2R に落ちる。
- 誤差の話は浮動小数点丸め以外不要。あなたの要求精度で“解析＋正規化”は十分。
- 問題はアルゴリズムがまだ (vx - a3 = a4 cos q3, vy = a4 sin q3) の単純円を想定し d4 を無視していた点。正しい円は半径 Lw の位相シフト付き。

ポイントは「MDH を一切書き換えず（= 物理そのまま）、a4 と d4 が同時に存在する非“球面”手首で幾何 IK を厳密（近似なし）に解く手順」を明確化することです。以下にアルゴリズムと根拠を整理します。

---

## 1. 本質：a4,d4 を Lw, ψ に内部変数変換するのは“再パラメータ化”ではなく“座標回転”
MiRobot2 の第3関節以降で手首中心へ向かうオフセットは  
O3→Pw_in_plane = a4 * X3 + d4 * Z4_inPlane  
（ここで Z4_inPlane は q4 以前の段階で X3-Y3 平面へ投影された成分とみなせる）  
実際にフレーム2座標 (W2 = (vx,vy, vz)) に現れる “肘平面内” 成分は厳密に次の形へ変換できます：

( vx - a3 ) = a4 cos q3 - d4 sin q3  
vy          = a4 sin q3 + d4 cos q3  

これは 2×2 の回転行列による線形結合なので
Lw = sqrt(a4^2 + d4^2), ψ = atan2(d4, a4) を定義すれば：

( vx - a3 ) = Lw cos( q3 + ψ )  
vy          = Lw sin( q3 + ψ )

ここには近似は一切ありません（数値誤差は double の丸めのみ）。  
よって “球面手首” という前提は不要。単なる回転位相ずらしです。

---

## 2. 位置 IK（q1,q2,q3）の厳密解法（MDH 変更なし）

手順:

1. Pw = P - d6 * z6（既存）  
2. q1 候補 = atan2(Pw_y, Pw_x) と +π 分枝の 2 つ  
3. 固定した q1 で W2(q2) = R02^T (Pw - O2) を計算（既存 `ComputeW2` のまま）  
   得られる W2 = (vx, vy, vz) は “まだ q3 を適用していない” 状態（あなたの実装構造で q3 はこの後 Z2 軸回り回転）  
4. 1 変数方程式  
   g(q2) = (vx(q2) - a3)^2 + vy(q2)^2 - (a4^2 + d4^2) = 0  
   これが肩関節 q2 の条件（Lw 半径円上に投影する条件）。  
   → サンプリング + 符号変化 + セカントで root 抽出（既に実装した g 方式と同じ原理）  
   → sign change が無い場合は |g| 最小点を近傍解として採用（許容閾値は 1e-2 〜 1e-1 * 単位）  
5. q2 が求まったら W2 を再計算し  
   q3_eff = atan2( vy, vx - a3 )  
   q3 = q3_eff - ψ （ψ = atan2(d4, a4)）を正規化  
   角度制限判定（q3 が範囲外なら破棄）  
6. vz について  
   z 成分は α/ねじれ配置の影響で (d1 など) と Lw の面外成分誤差を含む。vz と期待値「d4 の射影 + （構造定数）」の残差 |vz - vz_expected| をオプション検証に使う（厳しすぎる閾値で解を落とし過ぎないよう注意）

ここまでで (q1,q2,q3) は完全解析（近似なし）です。

---

## 3. 姿勢 IK（q4,q5,q6）で“球面”不要な理由
回転は純粋に回転行列の積で決まるため、途中に a4 や d4 の並進があっても回転部分 R03, R36 の抽出式は「角順序（あなたの MDH の回転順）」さえ守ればそのまま使える。  
つまり翻訳オフセット（a4,d4）は R36 の成分に直接影響しません。既存の：

$R_{36} = R_{03}^T * R_{06}$  
$q_5 = acos( R_{36}[1,2] )$  
$q_4 = atan2( \frac{R_{36}[2,2]}{\sin q_5}, \frac{R_{36}[0,2]}{\sin q_5} )$  
$q_6 = atan2( \frac{R_{36}[1,1]}{\sin q_5}, -\frac{R_{36}[1,0]}{\sin q_5} )$  

（特異 $q_5\approx0$ は別枝処理）

で OK。ここも近似不要。

---

## 5. 既存コードへの最小追加（q2,q3 中核のみ）

以下はあなたの現行 `InverseGeometric` に挿入可能な「$q_2 root → q_3$ 復元」部分の最小スニペット（MDH 未変更）。  
既に $g(q_2)$ 実装があるなら重複部は統合してください。

```csharp
// 事前: PwX,PwY,PwZ, q1 固定済み
double a3 = _mdh.A[2];
double a4 = _mdh.A[3];
double d4 = _mdh.D[3];
double Lw = Math.Sqrt(a4*a4 + d4*d4);
double psi = Math.Atan2(d4, a4);

// g(q2) root search
List<double> q2Candidates = SolveQ2ByRadius(q1, PwX, PwY, PwZ, a3, Lw, 61, verbose);

foreach (var q2 in q2Candidates)
{
    if (!WithinLimit(1, q2)) continue;

    var W2 = ComputeW2(q1, q2, PwX, PwY, PwZ);
    double vx = W2[0], vy = W2[1], vz = W2[2];

    double dx = vx - a3;
    double r2 = dx*dx + vy*vy;
    if (Math.Abs(r2 - Lw*Lw) > 1e-2) continue; // 位置円からのずれチェック（閾値調整可）

    double q3_eff = Math.Atan2(vy, dx);
    double q3 = RMLib.NormalizeAngleSigned(q3_eff - psi);
    if (!WithinLimit(2, q3)) continue;

    // 以降 q4,q5,q6 抽出 (既存ロジック再利用) ...
}

// ---- 半径方程式 g(q2)= (vx-a3)^2 + vy^2 - Lw^2 ----
private List<double> SolveQ2ByRadius(
    double q1, double PwX, double PwY, double PwZ,
    double a3, double Lw, int samples, int verbose)
{
    var roots = new List<double>();
    double q2Min = _minAnglesRad[1], q2Max = _maxAnglesRad[1];
    double step = (q2Max - q2Min) / (samples - 1);
    double prevQ = double.NaN, prevG = double.NaN;

    for (int i=0;i<samples;i++)
    {
        double q2 = q2Min + step*i;
        var W2 = ComputeW2(q1, q2, PwX, PwY, PwZ);
        double g = (W2[0]-a3)*(W2[0]-a3) + W2[1]*W2[1] - Lw*Lw;

        if (i>0 && prevG*g <= 0.0)
        {
            double r = RefineQ2RootSecant(q1, prevQ, q2, PwX, PwY, PwZ, a3, Lw);
            if (double.IsFinite(r))
            {
                var Wt = ComputeW2(q1, r, PwX, PwY, PwZ);
                double gt = (Wt[0]-a3)*(Wt[0]-a3) + Wt[1]*Wt[1] - Lw*Lw;
                if (Math.Abs(gt) < 1e-3)
                {
                    r = RMLib.NormalizeAngleSigned(r);
                    if (!roots.Any(x => Math.Abs(RMLib.NormalizeAngleSigned(x-r)) < 1e-3))
                        roots.Add(r);
                }
            }
        }
        prevQ = q2; prevG = g;
    }

    if (roots.Count==0)
    {
        // fallback: choose min |g|
        double bestQ = 0, bestAbs = double.MaxValue;
        for (int i=0;i<samples;i++)
        {
            double q2 = q2Min + step*i;
            var W2 = ComputeW2(q1, q2, PwX, PwY, PwZ);
            double g = (W2[0]-a3)*(W2[0]-a3) + W2[1]*W2[1] - Lw*Lw;
            double ag = Math.Abs(g);
            if (ag < bestAbs){ bestAbs=ag; bestQ=q2; }
        }
        if (bestAbs < 5e-2) roots.Add(bestQ); // 許容閾値調整
    }
    if (verbose>=2 && roots.Count>0)
        Console.WriteLine("   q2 radius roots: " + string.Join(", ", roots.Select(r => (r*180/Math.PI).ToString("F2"))));
    return roots;
}

private double RefineQ2RootSecant(double q1, double q2a, double q2b,
    double PwX, double PwY, double PwZ, double a3, double Lw,
    int maxIter=25, double tol=1e-6)
{
    double G(double q2)
    {
        var W = ComputeW2(q1, q2, PwX, PwY, PwZ);
        return (W[0]-a3)*(W[0]-a3) + W[1]*W[1] - Lw*Lw;
    }
    double fa = G(q2a), fb = G(q2b);
    double A=q2a, B=q2b;
    for (int i=0;i<maxIter;i++)
    {
        if (Math.Abs(fb-fa) < 1e-14) break;
        double C = B - fb*(B - A)/(fb - fa);
        double fc = G(C);
        A=B; fa=fb;
        B=C; fb=fc;
        if (Math.Abs(fc) < tol) return C;
    }
    return B;
}
```

※ これは “内部計算” であり MDH 配列は一切変更しません。

---

## 6. 許容閾値調整の考え方
| 項目 | 推奨初期値 | 説明 |
|------|-----------|------|
| |g| root 判定 | 1e-3 | 半径式残差 (mm^2 前提なら微小) |
| fallback |g| | 5e-2 | サンプリング粗さ救済。ロボットサイズスケールで調整 |
| vy,vx 円誤差 | 1e-2 | $sqrt(\vert g\vert)$ 相当 |
| 角度正規化 | ±π | RMLib.NormalizeAngleSigned を使う |

---

## 7. まとめ
- “球面”という構造前提は不要：a4,d4 の同時存在は Lw, ψ による座標回転で厳密に扱える。
- 近似は不要で、q2 は単一スカラー方程式 g(q2)=0、q3 は q3=atan2 - ψ で厳密。
- 姿勢部 (q4–q6) は回転のみを見るので翻訳オフセットは無影響。
- 精度要求（位置 1e-3, 角度 1e-4）に対し三角演算の丸め誤差は十分小さい。

---
# Craig “Modified Denavit–Hartenberg (MDH)” Convention

Below are the precise rules for constructing the (so‑called) Craig “Modified Denavit–Hartenberg (MDH)” frames, and how they differ from the original (classic) DH convention.

1. Core idea  
Classic DH associates $x_{i}$ with the common normal between $z_{i}$ and $z_{i+1}$, and builds $T_{i-1}^{i}$ with the sequence $Rot_z θ_i → Trans_z d_i → Trans_x a_i → Rot_x α_i$.  
Modified (Craig) DH instead associates $(a_{i-1}, α_{i-1})$ with link i−1 and $(θ_i, d_i)$ with joint i, giving a different construction order:  
$T_{i-1}^{i} = Rot_x(α_{i-1}) · Trans_x(a_{i-1}) · Rot_z(θ_i) · Trans_z(d_i)$.

2. Frame assignment algorithm (Modified DH)  
For i = 0..n (n joints):  
1) Choose $z_i$: the axis of joint i (for i = 1..n). $z_0$ is the base reference; $z_n$ may be tool/end-effector approach axis.  
2) For each $i ≥ 1$, choose $x_{i-1}$ along the (shortest) common normal from $z_{i-1}$ to $z_i$.  
   - If $z_{i-1}$ and $z_i$ intersect: $a_{i-1} = 0$; $x_{i-1}$ is any direction in the plane perpendicular to $z_{i-1}$ that yields a right‑handed frame.  
   - If they are skew: unique common normal → direction of $x_{i-1}$.  
   - If they are parallel: infinite common normals → pick one consistently (degeneracy).  
3) Set origin $O_{i-1}$ at the intersection of $z_{i-1}$ and $x_{i-1}$ (or at joint location if intersecting axes).  
4) Origin $O_i$ lies where $z_i$ meets $x_{i-1}$ (or at the joint if intersecting).  
5) Define $y_{i-1} = z_{i-1} × x_{i-1}$ to complete a right‑handed frame.

3. Parameter definitions (Modified DH)  
Given frames (i−1) and i constructed above:  
- $a_{i-1}$: link length = distance along $x_{i-1}$ from $z_{i-1}$ to $z_i$.  
- $α_{i-1}$: link twist = signed angle about $x_{i-1}$ rotating $z_{i-1}$ into $z_i$ (range usually (−π, π]).  
- $d_i$: joint offset = distance along $z_i$ from intersection with $x_{i-1}$ to origin $O_i$ (prismatic variable if joint i is prismatic).  
- $θ_i$: joint angle = rotation about $z_i$ from $x_{i-1}$ to $x_i$ (revolute variable if joint i is revolute).

4. Homogeneous transform (Modified DH)  
$T_{i-1}^{i} = Rot_x(α_{i-1}) · Trans_x(a_{i-1}) · Rot_z(θ_i) · Trans_z(d_i)$

Expanded 4×4 matrix:

[ 1        0           0          a_{i-1} ]   [  cosθ_i  -sinθ_i   0    0 ]   [ 1  0  0   0 ]
[ 0   cosα_{i-1}  -sinα_{i-1}     0       ] * [  sinθ_i   cosθ_i   0    0 ] * [ 0  0  1  d_i ]
[ 0   sinα_{i-1}   cosα_{i-1}     0       ]   [    0        0      1    0 ]   [ 0  1  0   0 ]
[ 0        0           0          1       ]   [    0        0      0    1 ]   [ 0  0  0   1 ]

Result (combined):
$T_{i-1}^{i}$ =
[  cosθ_i                      -sinθ_i                     0                  a_{i-1} ]
[  sinθ_i cosα_{i-1}   cosθ_i cosα_{i-1}   -sinα_{i-1}   -sinα_{i-1} d_i ]
[  sinθ_i sinα_{i-1}   cosθ_i sinα_{i-1}    cosα_{i-1}    cosα_{i-1} d_i ]
[       0                     0                     0                 1      ]

(You can reorder multiplication to derive same form; above matches the stated sequence.)

5. Comparison summary  
Classic DH: $T_{i-1}^{i} = Rot_z(θ_i) Trans_z(d_i) Trans_x(a_i) Rot_x(α_i)$.  
Modified DH: $T_{i-1}^{i} = Rot_x(α_{i-1}) Trans_x(a_{i-1}) Rot_z(θ_i) Trans_z(d_i)$.  
Index shift: (a, α) belong to preceding link; (θ, d) to current joint. The different order often yields simpler Jacobians and sometimes reduces singular parameter cases.

6. Degenerate / special cases  
- Intersecting axes: $a_{i-1} = 0$.  
- Parallel axes (same direction): $α_{i-1} = 0$, choose a consistent $x_{i-1}$; $a_{i-1}$ = shortest distance.  
- Parallel axes (opposite direction): $α_{i-1} = π$ (or $−π$); same handling.  
- Collinear axes: $a_{i-1} = 0$ and $α_{i-1} = 0$ (or $π$). $x_{i-1}$ arbitrary perpendicular; pick to maintain continuity.

7. Practical consistency rules  
- Always keep α continuous (avoid jumping between π and −π unless necessary).  
- When axes parallel, define a deterministic tie-break (e.g., choose $x_{i-1}$ to minimize angle to previous $x_{i-2}$).  
- Document your convention so externally supplied tables match.

8. Sample C# helper (Modified DH transform)  

```csharp
using System;
using System.Numerics;

public static class MDH
{
    // Returns T_{i-1}^{i} for Modified DH parameters (aPrev, alphaPrev, theta, d)
    public static Matrix4x4 Transform(float aPrev, float alphaPrev, float theta, float d)
    {
        float ca = MathF.Cos(alphaPrev);
        float sa = MathF.Sin(alphaPrev);
        float ct = MathF.Cos(theta);
        float st = MathF.Sin(theta);

        return new Matrix4x4(
            ct,             -st,            0f,         aPrev,
            st * ca,        ct * ca,       -sa,        -sa * d,
            st * sa,        ct * sa,        ca,         ca * d,
            0f,             0f,             0f,         1f
        );
    }
}
```

9. Cheat sheet (Modified DH)  
Given z_i axes:  
$x_{i-1}$: common normal $z_{i-1} ↔ z_i$  
$a_{i-1}$ = |common normal|  
$α_{i-1}$ = angle about $x_{i-1}$ from $z_{i-1}$ to $z_i$  
$θ_i$ = angle about $z_i$ from $x_{i-1}$ to $x_i$ (revolute variable)  
$d_i$ = displacement along $z_i$ from $x_{i-1}$ to origin $O_i$ (prismatic variable)  
$T_{i-1}^{i} = Rot_x(α_{i-1}) Trans_x(a_{i-1}) Rot_z(θ_i) Trans_z(d_i)$

That is the Craig / Modified DH construction. If you need a worked example with a specific 6‑DoF arm, ask and I can outline the full table.

Are those matrices row-major?
Yes (storage), but there is a subtlety:

1. Storage/layout
System.Numerics.Matrix4x4 is laid out row-major: the constructor arguments are M11 M12 M13 M14 (row 1), then M21..M24 (row 2), etc. So the snippet I gave is row-major in memory.

2. Multiplication convention
System.Numerics uses ROW vectors on the left in its helper methods (e.g. Vector3.Transform). Because of that, a pure translation created by Matrix4x4.CreateTranslation(tx,ty,tz) places tx,ty,tz in M41,M42,M43 (the last ROW), not the last column. This is the transpose of the conventional robotics (column‑vector) homogeneous transform where translation sits in the last column.

3. Implication for the MDH code I showed
The matrix I wrote was in the usual robotics (column‑vector) layout (translation in the last column). If you feed that directly to Vector3.Transform you will NOT get the intended result. You must either:
- Transpose it before using System.Numerics operations, or
- Build it directly in the System.Numerics row‑vector form.

4. Correct MDH (Modified DH) transform in System.Numerics (row‑vector convention)

Column‑vector (robotics) form (reference):
R =
[  ct        -st         0
   st*ca     ct*ca      -sa
   st*sa     ct*sa       ca ]

p = [ aPrev,  -sa*d,  ca*d ]^T

Row‑vector/System.Numerics form (rotation transposed, translation moved to last row):
R_row =
[  ct        st*ca      st*sa      0
  -st        ct*ca      ct*sa      0
   0         -sa         ca        0
   aPrev     -sa*d       ca*d      1 ]

Code (returns matrix ready for Vector3.Transform):

```csharp
using System;
using System.Numerics;

public static class MDH
{
    // Returns T_{i-1}^{i} in System.Numerics (row-vector) convention
    public static Matrix4x4 Transform(float aPrev, float alphaPrev, float theta, float d)
    {
        float ca = MathF.Cos(alphaPrev);
        float sa = MathF.Sin(alphaPrev);
        float ct = MathF.Cos(theta);
        float st = MathF.Sin(theta);

        // Row-major, row-vector form
        return new Matrix4x4(
            ct,          st * ca,        st * sa,       0f,
           -st,          ct * ca,        ct * sa,       0f,
            0f,         -sa,             ca,            0f,
            aPrev,      -sa * d,         ca * d,        1f
        );
    }
}
```

If instead you want the classical robotics (column‑vector) homogeneous matrix (translation last column), build it like this and transpose when using System.Numerics:

```csharp
public static Matrix4x4 TransformColumnVector(float aPrev, float alphaPrev, float theta, float d)
{
    float ca = MathF.Cos(alphaPrev);
    float sa = MathF.Sin(alphaPrev);
    float ct = MathF.Cos(theta);
    float st = MathF.Sin(theta);

    // Robotics convention (column-vector)
    var m = new Matrix4x4(
        ct,        -st,        0f,        aPrev,
        st * ca,    ct * ca,  -sa,       -sa * d,
        st * sa,    ct * sa,   ca,        ca * d,
        0f,         0f,        0f,        1f
    );

    // Transpose so that System.Numerics row-vector operations interpret it correctly
    return Matrix4x4.Transpose(m);
}
```

Summary:
- Matrix4x4 is row-major.
- System.Numerics expects row-vector multiplication (translation in last row).
- Adjust (or transpose) your MDH matrices accordingly to avoid subtle pose errors.

Let me know if you want a small test harness to verify both forms produce identical end-effector poses.