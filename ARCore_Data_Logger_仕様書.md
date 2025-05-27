# ARCore Data Logger 仕様書

## 1. プロジェクト概要

### 1.1 プロジェクト名
ARCore Data Logger

### 1.2 目的
Android端末でGoogle ARCoreのVisual-Inertial Odometry（VIO）による6自由度（6-DoF）姿勢推定結果と3Dポイントクラウドデータをリアルタイムで記録し、オフライン解析用のテキストファイルとして保存するアプリケーション。

### 1.3 開発者情報
- 原作者: PyojinKim
- GitHub: https://github.com/PyojinKim/ARCore-Data-Logger
- ライセンス: MIT License

## 2. システム要件

### 2.1 ハードウェア要件

#### 2.1.1 必須要件
- ARCore対応Android端末
- カメラ機能搭載
- 加速度センサー、ジャイロスコープ搭載
- OpenGL ES 3.0以上対応GPU
- RAM: 3GB以上推奨
- ストレージ: 1GB以上の空き容量

#### 2.1.2 推奨端末
- Google Pixelシリーズ
- Samsung Galaxyシリーズ（ARCore対応モデル）
- その他ARCore公式対応端末

### 2.2 ソフトウェア要件

#### 2.2.1 Android OS
- 最小バージョン: Android 8.0 (API Level 26)
- 推奨バージョン: Android 9.0 (API Level 28)以上
- ターゲットバージョン: Android 13 (API Level 33)
- コンパイルSDK: API Level 34

#### 2.2.2 ARCore要件
- Google Play Services for AR (ARCore) インストール必須
- ARCore バージョン: 1.11.0以上
- ARCore対応端末リストに含まれている必要あり

#### 2.2.3 必要な権限
- `CAMERA`: カメラアクセス権限（必須）
- `WAKE_LOCK`: 画面スリープ防止権限

## 3. アプリケーション仕様

### 3.1 アーキテクチャ

#### 3.1.1 主要クラス構成
- `MainActivity`: メインアクティビティ、UI制御
- `ARCoreSession`: ARCoreセッション管理、データ取得
- `ARCoreResultStreamer`: データファイル出力管理
- `AccumulatedPointCloud`: ポイントクラウドデータ蓄積
- `OutputDirectoryManager`: 出力ディレクトリ管理
- `FileStreamer`: ファイル書き込み基底クラス

#### 3.1.2 データフロー
1. ARCoreセッション初期化
2. カメラフレーム取得
3. 6-DoF姿勢データ抽出
4. 3Dポイントクラウドデータ抽出
5. リアルタイムファイル書き込み
6. セッション終了時のファイル保存

### 3.2 ユーザーインターフェース

#### 3.2.1 メイン画面構成
- ARCoreカメラビュー（フルスクリーン）
- 情報表示パネル（半透明オーバーレイ）
  - 特徴点数表示
  - トラッキング状態表示
  - 失敗理由表示
  - 更新レート表示
- 制御パネル
  - START/STOPボタン
  - 記録時間表示
- ロゴ表示（SFU、GRUVI）

#### 3.2.2 状態表示
- **トラッキング状態**
  - TRACKING: 正常トラッキング中
  - PAUSED: 一時停止
  - STOPPED: 停止
- **失敗理由**
  - NONE: 問題なし
  - LOW FEATURES: 特徴点不足
  - FAST MOTION: 高速移動
  - LOW LIGHT: 照明不足
  - BAD STATE: 不正状態

### 3.3 データ記録仕様

#### 3.3.1 6-DoF姿勢データ (ARCore_sensor_pose.txt)
```
# Created at [timestamp] in Burnaby Canada
[timestamp] [qx] [qy] [qz] [qw] [tx] [ty] [tz]
```

**データ形式詳細:**
- `timestamp`: ナノ秒単位のタイムスタンプ
- `qx, qy, qz, qw`: クォータニオン（デバイス姿勢）
- `tx, ty, tz`: 位置座標（メートル単位）

**座標系:** ARCore世界座標系
- X軸: 右方向（+）/ 左方向（-）
- Y軸: 上方向（+）/ 下方向（-）、重力と逆方向
- Z軸: 前方向（+）/ 後方向（-）

#### 3.3.2 3Dポイントクラウドデータ (ARCore_point_cloud.txt)
```
# Created at [timestamp] in Burnaby Canada
[x] [y] [z] [R] [G] [B]
```

**データ形式詳細:**
- `x, y, z`: 3D座標（メートル単位）
- `R, G, B`: RGB色情報（0-255）

**フィルタリング条件:**
- 信頼度閾値: 0.5以上
- カメラ画像内の点のみ
- 有効な色情報を持つ点のみ

### 3.4 ファイル管理

#### 3.4.1 保存場所
- **内部ストレージ**: `/data/user/0/com.pjinkim.arcore_data_logger/files/`
- **外部アクセス用**: `Android/data/com.pjinkim.arcore_data_logger/files/ARCore_Logs/`

#### 3.4.2 フォルダ命名規則
```
[prefix]YYYYMMDDHHMMSS[suffix]
例: 20250527143046R_pjinkim_ARCore
```

#### 3.4.3 ファイル構成
```
セッションフォルダ/
├── ARCore_sensor_pose.txt    # 6-DoF姿勢データ
└── ARCore_point_cloud.txt    # 3Dポイントクラウドデータ
```

## 4. 開発環境仕様

### 4.1 開発ツール
- **Android Studio**: 最新安定版推奨
- **Gradle**: 8.2
- **Android Gradle Plugin**: 8.2.2
- **Build Tools**: 34.0.0

### 4.2 依存関係
```gradle
dependencies {
    implementation "com.google.ar.sceneform.ux:sceneform-ux:1.11.0"
    implementation 'androidx.appcompat:appcompat:1.6.1'
    implementation 'androidx.constraintlayout:constraintlayout:2.1.4'
    // テスト用
    testImplementation 'junit:junit:4.13.2'
    androidTestImplementation 'androidx.test:runner:1.5.2'
    androidTestImplementation 'androidx.test.espresso:espresso-core:3.5.1'
}
```

## 5. データ解析ツール仕様

### 5.1 Python可視化スクリプト

#### 5.1.1 ファイル: `visualize_arcore_data.py`
**機能:**
- 3D軌跡可視化
- 時系列データプロット
- ポイントクラウド表示
- 統計レポート生成

**必要ライブラリ:**
```
numpy>=1.20.0
matplotlib>=3.3.0
scipy>=1.7.0
```

#### 5.1.2 ファイル: `save_analysis_results.py`
**機能:**
- 詳細統計計算
- 可視化画像保存
- JSON/テキストレポート出力
- セッション別解析

### 5.2 解析結果出力

#### 5.2.1 生成ファイル
```
analysis_results/
├── trajectory_3d.png           # 3D軌跡画像
├── time_series_analysis.png    # 時系列解析画像
├── point_cloud_3d.png         # ポイントクラウド画像
├── analysis_report.json       # 機械読み取り用レポート
└── analysis_report.txt        # 人間読み取り用レポート
```

#### 5.2.2 統計項目
- **セッション情報**: 時間、サンプル数、更新レート
- **運動解析**: 移動距離、速度統計
- **回転解析**: ロール、ピッチ、ヨー範囲
- **位置統計**: 平均位置、標準偏差、範囲
- **ポイントクラウド統計**: 点数、座標範囲

## 6. パフォーマンス仕様

### 6.1 更新レート
- **ARCore**: 最大60Hz
- **UI更新**: 10Hz（100ms間隔）
- **ファイル書き込み**: リアルタイム

### 6.2 メモリ管理
- **ポイントクラウド**: 最大50,000点まで蓄積
- **メモリ使用量**: 画像処理とポイントクラウドデータに依存
- **長時間記録**: メモリ使用量監視が必要

### 6.3 ストレージ使用量
- **姿勢データ**: 約2KB/秒（30Hz時）
- **ポイントクラウド**: 可変（環境に依存）
- **推定値**: 1分間で約500KB-2MB

## 7. エラーハンドリング

### 7.1 ARCore関連エラー
- **ARCore未インストール**: Google Play Storeへの誘導
- **バージョン不適合**: アップデート要求
- **端末非対応**: 明確なエラーメッセージ表示

### 7.2 権限関連エラー
- **カメラ権限拒否**: アプリ終了
- **権限要求**: 実行時権限要求実装

### 7.3 ファイルI/Oエラー
- **ディスク容量不足**: エラー表示とグレースフル停止
- **書き込み失敗**: ログ出力と継続試行

## 8. セキュリティ仕様

### 8.1 データ保護
- **内部ストレージ使用**: 他アプリからのアクセス制限
- **権限最小化**: 必要最小限の権限のみ要求
- **データ暗号化**: 標準Android暗号化に依存

### 8.2 プライバシー
- **位置情報**: 相対座標のみ記録（絶対位置なし）
- **画像データ**: 直接保存なし（色情報のみ抽出）
- **個人識別情報**: 収集なし

## 9. テスト仕様

### 9.1 単体テスト
- **データ処理**: クォータニオン変換、座標変換
- **ファイルI/O**: 書き込み、読み込み機能
- **エラーハンドリング**: 例外処理の検証

### 9.2 統合テスト
- **ARCoreセッション**: 初期化から終了まで
- **UI操作**: ボタン操作、状態遷移
- **データ整合性**: 記録データの検証

### 9.3 端末テスト
- **複数端末**: 異なるARCore対応端末での動作確認
- **OS版**: 異なるAndroidバージョンでの互換性
- **長時間動作**: メモリリーク、パフォーマンス劣化の確認

## 10. 運用・保守

### 10.1 ログ出力
- **レベル**: INFO, WARN, ERROR
- **出力先**: Android Logcat
- **内容**: セッション状態、エラー詳細、パフォーマンス情報

### 10.2 バージョン管理
- **セマンティックバージョニング**: MAJOR.MINOR.PATCH
- **リリースノート**: 機能追加、バグ修正の記録
- **後方互換性**: データ形式の互換性維持

### 10.3 サポート
- **ドキュメント**: README、仕様書の維持
- **GitHub Issues**: バグ報告、機能要求の管理
- **コミュニティ**: オープンソースコミュニティでのサポート

## 11. 今後の拡張予定

### 11.1 機能拡張
- **リアルタイム可視化**: 記録中の3D表示
- **データ圧縮**: ファイルサイズ削減
- **クラウド同期**: データの自動バックアップ

### 11.2 解析機能強化
- **機械学習**: 軌跡パターン認識
- **統計解析**: より詳細な運動解析
- **比較機能**: 複数セッションの比較

### 11.3 プラットフォーム拡張
- **iOS対応**: ARKit版の開発
- **Web版**: WebXRを使用したブラウザ版
- **クロスプラットフォーム**: Flutter/React Native版

---

**最終更新日**: 2025年5月27日  
**バージョン**: 1.0  
**作成者**: ARCore Data Logger開発チーム 