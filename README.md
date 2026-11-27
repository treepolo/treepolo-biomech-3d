# treepolo-biomech-3d

**treepolo 3D 投球解剖與生物力學互動模擬平台（v2.0 — Z-Anatomy + Scale-then-DQS）**

基於真實 360 Hz 光學動作捕捉數據（Driveline OpenBiomechanics Project）與 1080 Hz 力板數據建構的瀏覽器端全身 3D 解剖生物力學視覺化平台。採用 **Z-Anatomy (CC BY-SA 4.0)** 高精解剖骨骼、肌肉與真實肌腱／腱膜／支持帶／韌帶網格，並將**視覺變形層（Scale-then-DQS 雙四元數蒙皮）**與**生物力學計算層（骨面解剖 MTU 路徑）**完全解耦，即時呈現投球過程中全身 419 條肌肉與 UCL 的**長度變化 ($L$)**、**伸長率 ($\Delta L\%$)**、**變化速度 ($dL/dt$)** 與**變化加速度 ($d^2L/dt^2$)**。

🔗 **線上體驗（GitHub Pages）**：https://treepolo.github.io/treepolo-biomech-3d/

---

## 核心特色（v2.0）

* **模組化多檔架構（GitHub Pages 原生支援）**
  * `index.html`、`css/style.css`
  * `js/core.js`（向量、矩陣與雙四元數運算）、`js/kinematics.js`（OBP 運動學、連續三次 Hermite 脊柱、2-DOF 肩胛-胸廓橢球貼合、零裂縫膝關節與滑車髕骨追蹤）、`js/skinning.js`（Scale-then-DQS 雙四元數蒙皮＋螺旋膨脹保護＋保層次骨面包繞）、`js/bio.js`（解耦解剖 MTU 生物力學計算）、`js/scene.js`（Three.js 渲染）、`js/ui.js`（介面與圖表）
  * `assets/`：`manifest.json`、`bones.bin`（39 節骨骼含獨立髕骨與蒙皮肋軟骨）、`muscles.bin`（419 條肌肉含 45,394 個原生解剖肌腱面）、`tendons.bin`（41 個獨立肌腱／腱膜／支持帶／韌帶）、`pitches.json`（內建 20 顆 OBP 投球與 1080 Hz GRF 力板資料）
* **真實解剖肌腱、腱膜與韌帶**
  * 每條 Z-Anatomy 肌肉網格保留原生**肌腹 vs 肌腱**雙材質分區：肌腹依 $dL/dt$、$\Delta L\%$ 或 $d^2L/dt^2$ 即時力學著色，肌腱端呈現象牙白纖維質感。
  * 獨立收錄 41 個真實解剖結構：阿基里斯腱（跟腱）、髂脛束、伸趾長肌腱、腹白線、胸腰筋膜（前／中／後層）、掌腱膜、足底腱膜、腕／踝伸屈與腓骨支持帶、腹股溝韌帶、前後十字韌帶（ACL/PCL）、內側副韌帶（MCL）、橈骨環狀韌帶，可一鍵切換顯示。
* **真實 360 Hz 動捕 ＋ 1080 Hz 力板驅動**
  * 內建 20 顆 Driveline OBP 投球（依球速由快至慢排序），並支援拖放匯入 `filtered_pitching_landmarks.csv` 與 `filtered_pitching_joint_angles.csv`。
  * 逐幀精準肢段追蹤、Z-Y-Z 肩角與 X-Y-Z 髖角實測軸向重建、髖寬校正（`HIP_K = 1.5`）、動態規劃足部貼丘與趾骨背屈避讓。

---

## 本機執行

由於本版採用 `fetch()` 並行載入 `assets/` 二進位與 JSON 資產，本機測試請以簡易 HTTP 伺服器開啟：

```bash
python3 -m http.server 8000
# 瀏覽器開啟 http://localhost:8000
```

---

## 資料來源與授權聲明（Attribution & Licenses）

* **動作捕捉與力板數據**：[Driveline Baseball - OpenBiomechanics Project (OBP)](https://github.com/drivelineresearch/openbiomechanics) — **CC BY-NC-SA 4.0**
* **3D 解剖骨骼、肌肉、肌腱與韌帶網格**：[Z-Anatomy](https://github.com/Z-Anatomy/Models-of-human-anatomy)（Gauthier Kervyn）— **CC BY-SA 4.0**（衍生自 [BodyParts3D](https://lifesciencedb.jp/bp3d/), DBCLS — **CC BY-SA 2.1 JP**）
* 本專案之骨骼與軟組織模型依標準解剖比例縮放，僅供教學、動作理解與研究視覺化參考，不作為臨床醫療診斷依據。

---

Maintained by **treepolo**
