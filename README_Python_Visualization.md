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
   python visualize_arcore_data.py downloaded_logs/20250527140912R_pjinkim_ARCore
   ```

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

## 座標系について

### ARCore世界座標系
- **Y軸**: 重力方向に対して上向き（+Y）
- **Z軸**: デバイスカメラが向く方向の逆（0,0,-1）
- **X軸**: 右手座標系によりY軸とZ軸から決定

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