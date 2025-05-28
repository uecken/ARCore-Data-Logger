# ARCoreを使用したRFIDタグ位置推定システム

このプロジェクトは、ARCore Data LoggerにRFIDタグ検出と位置推定機能を追加した拡張版です。システムはARCoreのVisual-Inertial Odometry (VIO)を使用してデバイスの位置と向きを追跡し、複数の視点からの検出に基づいてRFIDタグの位置を推定します。

## 機能

- **ARCore統合**: 正確なデバイス姿勢追跡のためのARCore使用
- **RFIDタグ検出**: ボタン押下によるRFIDタグ検出のシミュレーション
- **タグ位置推定**: 三角測量を使用したRFIDタグの3D位置推定
- **データ可視化**: デバイス軌跡とタグ位置の3Dおよび2Dプロット
- **包括的解析**: 位置推定精度の統計解析

## システム要件

### Androidアプリ
- ARCore対応Androidデバイス
- Android Studio
- 最小SDKバージョン: 24 (Android 7.0)

### Python解析
- Python 3.7+
- 必要なパッケージ:
  ```
  numpy
  matplotlib
  pandas
  scipy
  ```

## インストール

### 1. Androidアプリのセットアップ

1. Android Studioでプロジェクトを開く
2. ARCore対応デバイスにアプリをビルド・インストール
3. プロンプトが表示されたらカメラと保存の権限を許可

### 2. Python環境のセットアップ

必要なPythonパッケージをインストール:
```bash
pip install numpy matplotlib pandas scipy
```

## 使用方法

### 1. データ収集

1. **Androidアプリを起動**
2. **記録開始**: "Start Recording"ボタンを押す
3. **移動**: RFIDタグを検出したいエリアを歩き回る
4. **タグ検出**: RFIDタグ検出をシミュレートしたい時に"Detect RFID Tag"ボタンを押す
   - より良い位置推定精度のため、複数の異なる位置からボタンを押してください
   - システムは各検出時のデバイスの姿勢（位置と向き）を記録します
5. **記録停止**: 終了時に"Stop Recording"ボタンを押す

### 2. データ解析

#### クイックテスト
テストスクリプトでデータを解析:
```bash
python test_rfid_localization.py
```

#### 手動解析
```python
from rfid_tag_localization import RFIDTagLocalizer

# セッションフォルダで初期化
localizer = RFIDTagLocalizer("path/to/your/session/folder")

# ポーズデータの読み込み
localizer.load_pose_data()

# 検出されたタグの位置推定
localizer.localize_all_tags(max_distance=5.0)

# レポート生成
localizer.generate_report()

# 可視化表示
localizer.plot_results()
```

## データ形式

### ARCoreポーズデータ (`ARCore_sensor_pose.txt`)

各行には以下が含まれます:
```
timestamp qx qy qz qw tx ty tz [tag_id]
```

ここで:
- `timestamp`: ナノ秒単位のタイムスタンプ
- `qx, qy, qz, qw`: デバイス向きのクォータニオン
- `tx, ty, tz`: 位置（メートル単位）
- `tag_id`: オプションのRFIDタグID（タグ検出時）

例:
```
1685188246123456789 0.001234 -0.005678 0.012345 0.999876 1.234567 -0.567890 1.500000
1685188246156789012 0.001235 -0.005679 0.012346 0.999875 1.234568 -0.567891 1.500001 TAG001
```

## ARCore座標系について

### ARCore世界座標系の定義
ARCoreは右手座標系を使用します:

- **X軸**: 右方向（+X）/ 左方向（-X）
- **Y軸**: 上方向（+Y）/ 下方向（-Y）
  - 重力ベクトルは-Y方向を指します
- **Z軸**: カメラ後方（+Z）/ カメラ前方（-Z）
  - カメラが向いている方向は-Z方向です

### 可視化における座標軸の表示

#### 1. 3D表示
- すべての軸がそのまま表示されます（X, Y, Z）

#### 2. Top View (X-Y平面)
- X軸: 右/左方向
- Y軸: 上/下方向

#### 3. Side View (X-Z平面)
- X軸: 右/左方向
- Y軸: -Z軸（カメラ前方/後方）を表示
- 注意: より直感的な表示のため、Z軸の符号を反転して表示しています

#### 4. カメラ方向矢印
- 赤い矢印はデバイスのカメラが向いている方向を示します
- ARCore座標系では、カメラ方向は-Z軸方向です

## 位置推定アルゴリズム

システムはタグ位置推定に2つの手法を使用します:

### 1. 加重重心法 (≥2検出)
- 検出位置の重心を計算
- 検出信頼度に基づく重み付け（現在は均一）

### 2. 光線交点法 (≥3検出)
- デバイス位置から視線方向に光線を投影
- すべての光線からの距離を最小化する点を探索
- 複数検出に対してより正確

## 可視化

システムは複数のプロットを生成します:

1. **3D軌跡**: 3D空間でのデバイスパスとタグ位置
2. **2D上面図**: 軌跡とタグの鳥瞰図
3. **2D側面図**: 軌跡とタグの側面図（修正された座標軸）
4. **検出タイムライン**: 時間経過によるタグ検出
5. **誤差解析**: 位置推定精度メトリクス

## 設定

### 位置推定パラメータ

`rfid_tag_localization.py`で調整可能:

```python
# 有効な検出の最大距離（メートル）
max_distance = 5.0

# 位置推定に必要な最小検出数
min_detections = 2

# 光線交点の許容誤差
intersection_tolerance = 0.1
```

### Androidアプリパラメータ

`MainActivity.java`内:

```java
// シミュレートされたタグID（動的に変更可能）
private static final String SIMULATED_TAG_ID = "TAG001";
```

## トラブルシューティング

### よくある問題

1. **"No ARCore session"エラー**: デバイスがARCoreに対応し、アプリにカメラ権限があることを確認
2. **"No tag detections"エラー**: 記録中に"Detect RFID Tag"ボタンを押すことを確認
3. **位置推定精度が低い**: 異なる位置・角度からより多くの検出を収集
4. **"File not found"エラー**: セッションフォルダに`ARCore_sensor_pose.txt`が含まれているか確認

### データ品質のコツ

1. **複数の視点**: 少なくとも3-4の異なる位置からタグを検出
2. **多様な角度**: タグエリアに異なる方向からアプローチ
3. **安定した移動**: 検出時の急激な動きを避ける
4. **良好な照明**: ARCoreトラッキングに十分な照明を確保

## 実際のRFIDリーダーとの統合

実際のRFIDハードウェアと統合するには:

1. **ボタン検出を置換**: `MainActivity.java`を修正してRFIDリーダーイベントを監視
2. **RFIDライブラリを追加**: 適切なRFIDリーダーSDKを含める
3. **タグIDを更新**: RFIDリーダーからの実際のタグIDを使用
4. **RSSI データを追加**: 位置推定改善のための信号強度情報を含める

統合例:
```java
// ボタンクリックをRFID検出コールバックで置換
private void onRFIDTagDetected(String tagId, float rssi) {
    if (mARCoreSession != null && mARCoreSession.isRecording()) {
        mARCoreSession.detectRFIDTag(tagId, rssi);
    }
}
```

## パフォーマンス考慮事項

- **記録頻度**: ARCoreポーズデータは約30Hzで記録
- **メモリ使用量**: 大きなセッションは解析時に大量のメモリを必要とする可能性
- **処理時間**: 位置推定時間は検出数に比例
- **精度**: 条件により典型的な精度は10-50cm

## 今後の機能拡張

1. **RSSI ベース位置推定**: 距離推定のための信号強度使用
2. **カルマンフィルタリング**: センサー融合による姿勢推定改善
3. **リアルタイム位置推定**: 記録中の検出処理
4. **マルチタグ対応**: 複数タグの同時位置推定
5. **機械学習**: 検出と位置推定改善のためのML使用

## ライセンス

このプロジェクトは元のARCore Data Loggerを拡張しています。元のライセンス条項を参照してください。

## 連絡先

質問や問題については、元のARCore Data Loggerドキュメントを参照するか、プロジェクトリポジトリでissueを作成してください。 