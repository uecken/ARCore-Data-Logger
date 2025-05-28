# ARCore Data Logger - Python可視化ツール

ARCore Data Loggerで記録したデータをPythonで可視化するためのスクリプトです。

## 機能

このPython可視化ツールは以下の機能を提供します：

### 1. 6-DoF姿勢データの可視化
- **3D軌跡表示**: デバイスの移動軌跡を3D空間で表示
- **座標フレーム表示**: 各時点でのデバイスの向きを座標軸で表示
- **時系列グラフ**: 位置、クォータニオン、速度、原点からの距離の時間変化

### 2. 3Dポイントクラウドの可視化
- **カラー付きポイントクラウド**: RGB情報を含む3Dポイントの表示
- **デバイス軌跡との重ね合わせ**: ポイントクラウドとデバイスパスの同時表示

### 3. データ統計レポート
- 記録時間、更新レート、移動距離などの統計情報
- データ範囲とサンプル数の表示

## インストール

### 1. 必要なライブラリのインストール
```bash
pip install -r requirements.txt
```

または個別にインストール：
```bash
pip install numpy matplotlib
```

### 2. スクリプトの実行権限設定（Linux/Mac）
```bash
chmod +x visualize_arcore_data.py
```

## 使用方法

### 基本的な使用方法

1. **ARCoreアプリでデータを記録**
   - アプリを起動してSTARTボタンを押す
   - データ記録後、STOPボタンを押す
   - ファイルが自動的にダウンロードされる

2. **Pythonスクリプトで可視化**
   ```bash
   # デフォルト（downloaded_logsフォルダを使用）
   python visualize_arcore_data.py
   
   # 特定のフォルダを指定
   python visualize_arcore_data.py path/to/your/data/folder
   
   # 特定のセッションフォルダを指定
   python visualize_arcore_data.py downloaded_logs/ARCore_Logs/20250527142630R_pjinkim_ARCore

### コマンドライン引数

```bash
python visualize_arcore_data.py [data_folder_path]
```

- `data_folder_path`: ARCoreデータファイルが含まれるフォルダのパス
- 省略した場合は `downloaded_logs` フォルダを使用
- 複数のセッションフォルダがある場合は最新のものを自動選択

## データファイル形式

### ARCore_sensor_pose.txt
```
# Created at [timestamp] in Burnaby Canada
[timestamp] [qx] [qy] [qz] [qw] [tx] [ty] [tz]
...
```
- `timestamp`: ナノ秒単位のタイムスタンプ
- `qx, qy, qz, qw`: クォータニオン（デバイスの向き）
- `tx, ty, tz`: 位置（メートル単位）

### ARCore_point_cloud.txt
```
# Created at [timestamp] in Burnaby Canada
[x] [y] [z] [R] [G] [B]
...
```
- `x, y, z`: 3D座標（メートル単位）
- `R, G, B`: RGB色情報（0-255）

## 可視化の詳細

### 1. 3D軌跡プロット
- **青線**: デバイスの移動軌跡
- **緑丸**: 開始点
- **赤四角**: 終了点
- **RGB軸**: 各時点でのデバイスの向き（赤=X軸、緑=Y軸、青=Z軸）

### 2. 時系列プロット
- **位置グラフ**: X, Y, Z座標の時間変化
- **クォータニオングラフ**: qx, qy, qz, qwの時間変化
- **速度グラフ**: デバイスの移動速度
- **距離グラフ**: 原点からの距離

### 3. ポイントクラウドプロット
- **カラー付き点群**: 実際のRGB色で表示
- **デバイス軌跡**: 赤線でデバイスの移動パスを表示

## ARCore座標系について

### ARCore世界座標系の定義
ARCoreは右手座標系を使用します:

- **X軸**: 右方向（+X）/ 左方向（-X）
- **Y軸**: 上方向（+Y）/ 下方向（-Y）
  - 重力ベクトルは-Y方向を指します
  - ARCore初期化時に重力方向に基づいてY軸が決定されます
- **Z軸**: カメラ後方（+Z）/ カメラ前方（-Z）
  - カメラが向いている方向は-Z方向です
  - 初期セッション開始時のカメラ向きに基づいて決定されます

### 可視化における座標軸の表示

#### 1. 3D表示
- すべての軸がそのまま表示されます（X, Y, Z）
- 座標フレーム矢印:
  - **赤**: X軸（右方向）
  - **緑**: Y軸（上方向）
  - **青**: Z軸（カメラ後方）

#### 2. Top View (X-Y平面)
- X軸: 右/左方向の移動
- Y軸: 上/下方向の移動
- この視点では重力方向（Y軸）と水平移動（X軸）が表示されます

#### 3. Side View (X-Z平面)
- X軸: 右/左方向の移動
- Y軸: -Z軸（カメラ前方/後方方向）を表示
- **重要**: より直感的な表示のため、Z軸の符号を反転（-Z）して表示しています
- これにより、カメラの前進/後退が上下方向として表示されます

#### 4. カメラ向きの解釈
- デバイスの向きは各時点での座標フレーム（RGB矢印）で表示されます
- 青い矢印（Z軸）はカメラの後方を指すため、カメラは青い矢印と逆方向を向いています

### 座標変換について

#### ARCore → 可視化座標変換
- 3D表示: 直接ARCore座標を使用
- Top View: X-Y平面をそのまま表示
- Side View: X軸はそのまま、Y軸として-Z軸を使用

#### RFIDタグ位置推定における座標系
RFID位置推定システムでは：
- カメラ方向ベクトルは-Z方向として計算されます
- 位置推定結果はARCore座標系で出力されます
- 可視化時には上記の座標変換ルールが適用されます

## トラブルシューティング

### よくある問題

1. **"No data could be loaded!"エラー**
   - データファイルのパスが正しいか確認
   - ARCore_sensor_pose.txtファイルが存在するか確認

2. **"No point cloud data available"警告**
   - 短時間の記録ではポイントクラウドデータが少ない場合があります
   - より長時間の記録を試してください

3. **グラフが表示されない**
   - matplotlibのバックエンドを確認：
     ```python
     import matplotlib
     matplotlib.use('TkAgg')  # または 'Qt5Agg'
     ```

4. **メモリエラー**
   - 大量のポイントクラウドデータの場合、メモリ不足が発生する可能性があります
   - より短時間の記録を試すか、データをサブサンプリングしてください

5. **座標軸の向きが期待と異なる**
   - ARCore座標系は右手座標系です
   - 重力方向がY軸、カメラ方向がZ軸の負の方向になります
   - 初期化時のデバイスの向きによって座標系が決定されます

## カスタマイズ

スクリプトは簡単にカスタマイズできます：

```python
# 可視化クラスのインスタンス作成
visualizer = ARCoreDataVisualizer('your_data_folder')

# 個別の可視化機能を実行
visualizer.load_pose_data()
visualizer.plot_trajectory_3d()
visualizer.plot_pose_components()
```

## 出力例

実行すると以下のような出力が表示されます：

```
Loading ARCore data...
Loaded 66 pose samples
Warning: No point cloud data found!

==================================================
ARCore Data Summary Report
==================================================
Pose Data:
  - Number of samples: 66
  - Duration: 2.18 seconds
  - Average update rate: 30.3 Hz
  - Total distance traveled: 0.045 m
  - Position range:
    X: [-0.002, 0.003] m
    Y: [-0.001, 0.002] m
    Z: [-0.003, 0.001] m
==================================================

Generating trajectory visualization...
Generating pose component plots...
Visualization complete!
```

## ライセンス

このスクリプトはMITライセンスの下で提供されています。 