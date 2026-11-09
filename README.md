# 16 L Tarımsal İlaçlama Dronu – ArduPilot SITL & Gazebo Kontrol Çalışması

[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04.4_LTS-orange.svg)](https://ubuntu.com/)
[![ArduCopter](https://img.shields.io/badge/ArduCopter-V4.8.0--dev-blue.svg)](https://ardupilot.org/)
[![Gazebo Sim](https://img.shields.io/badge/Gazebo_Sim-8.11.0-informational.svg)](https://gazebosim.org/)
[![Python 3.12](https://img.shields.io/badge/Python-3.12.3-green.svg)](https://www.python.org/)
[![Mission Planner](https://img.shields.io/badge/Mission_Planner-1.3.83-yellow.svg)](https://ardupilot.org/planner/)

**ArduPilot Copter SITL, Gazebo Sim ve Mission Planner kullanılarak 16 L tank kapasiteli tarımsal ilaçlama dronunda boş/dolu tank kütle ve ağırlık merkezi (CG) değişimlerinin kalkış, askı (hover), rota izleme ve iniş kontrolüne etkisinin incelenmesi ve iyileştirilmesi.**

> **Not**: Bu depodaki tüm araç kütleleri, atalet momentleri, itki ölçeklemeleri ve tank/bom geometrileri gerçek ürün verisi değil, simülasyon analizi için açıkça etiketlenmiş **varsayımlardır**.

---

## Sistem Çalışırken (Gazebo Sim + Mission Planner + MAVLink)

<div align="center">
  <table>
    <tr>
      <td width="50%">
        <img src="figures/demo_field_mission.gif" alt="16L İlaçlama Dronu Kalkış ve Tarla Görevi" width="100%"/>
        <br/>
        <em>Hassas iniş pedi üzerindeki 16 L ilaçlama dronu ve Mission Planner 43 waypoint'li tarla tarama görevi</em>
      </td>
      <td width="50%">
        <img src="figures/demo_sitl_scenarios.gif" alt="Otomatik Karşılaştırmalı Senaryo Koşusu" width="100%"/>
        <br/>
        <em>MAVLink senaryo koşucusu (empty / full / full_tuned) ve sarı hassas iniş pedine otonom iniş</em>
      </td>
    </tr>
    <tr>
      <td width="50%">
        <img src="figures/gazebo_farm_field.jpg" alt="Gazebo Tarım Alanı ve Mission Planner" width="100%"/>
        <br/>
        <em>Pist kaldırılarak görev koordinatlarıyla hizalanmış 65×95 m Gazebo tarlası ve Mission Planner rotası</em>
      </td>
      <td width="50%">
        <img src="figures/spray_tank_landing_pad.jpg" alt="16L İlaç Tankı ve Hassas İniş Pedi" width="100%"/>
        <br/>
        <em>16 L silindirik mavi tank, 80 cm ilaçlama bomu, 4 kırmızı nozul ve sarı/siyah hassas iniş pedi</em>
      </td>
    </tr>
  </table>
</div>

## Telemetri Karşılaştırması (Boş Tank vs. Ayarsız Dolu Tank vs. Ayarlı Dolu Tank)

<div align="center">
  <img src="figures/comparison.png" alt="Boş vs Dolu vs Ayarlı Dolu Karşılaştırma Grafiği" width="850"/>
  <br/>
  <em>Üç senaryo için irtifa takibi, 20×20 m yatay kare rota izleme, Roll/Pitch açısal yanıtları ve ortalama motor PWM sinyalleri</em>
</div>

---

## Proje Özeti

Tarımsal ilaçlama dronları, tek bir görev içerisinde toplam kalkış kütlesinin yarıdan fazlasını (16 L sıvı ≈ 16 kg) kaybeder. Bu çalışma, **12 kg boş gövde** ile **28 kg tam dolu tank** durumları arasındaki kütle, atalet momenti ve ağırlık merkezi (CG) değişimlerinin ArduPilot Copter kontrol döngülerine etkisini **Gazebo 3B fizik simülasyonu** ve **gerçek MAVLink telemetrisi** ile inceler:

- **Özel 3B Tarımsal Gazebo Ortamı**: Standart `iris_runway` dünyasındaki asfalt pist kaldırılmış; yerine Mission Planner'daki 43 waypoint'lik tarama (Survey Grid) koordinatlarıyla birebir örtüşen **65×95 m tarım arazisi (6 ekin sırası)** ve Home (`0, 0`) noktasında **hassas iniş pedi (`precision_landing_pad`)** yerleştirilmiştir.
- **16 L İlaç Tankı ve Bom Modellemesi**: `iris_agricultural` modeline silindirik sıvı tankı (`spray_tank_link`), 80 cm püskürtme bomu ve 4 adet nozul eklenmiş; dolu tank senaryosunda ek kütle, artırılmış $I_{xx}, I_{yy}, I_{zz}$ atalet momentleri ve kaçık ağırlık merkezi (**+2.5 cm X, +1.5 cm Y, −14 cm Z**) tanımlanmıştır.
- **Tekrarlanabilir Otomatik Uçuş ve Analiz Hattı**: `run_scenario.py` betiği MAVLink üzerinden ArduPilot SITL'e bağlanarak parametreleri ayarlar, tam otomatik `GUIDED kalkış (5 m) → 15 sn sabit hover → 20×20 m kare rota → LAND` görevini uçurur ve 20 Hz'de telemetri kaydeder. `analyze.py` ise bu verilerden 14 mühendislik metriğini ve 4 panelli karşılaştırma grafiğini (`comparison.png`) otomatik üretir.

---

## Sistem Mimarisi

```text
+-----------------------------------------------------------------------------------+
|                        Gazebo Sim 8.11 (worlds/farm_field.sdf)                    |
|   +-----------------------------+         +-----------------------------------+   |
|   |   iris_agricultural.sdf     |         | 65x95m Tarla (6 Ekin Sırası)      |   |
|   | - 16L Tank + Bom + Nozullar |         | + Sarı/Siyah Hassas İniş Pedi     |   |
|   | - Kütle, Atalet & CG Kayması|         +-----------------------------------+   |
|   +--------------+--------------+                                                 |
+------------------|----------------------------------------------------------------+
                   | JSON Fizik Köprüsü (UDP 9002 / 9003)
                   v
+-----------------------------------------------------------------------------------+
|                  ArduPilot Copter SITL V4.8.0-dev (JSON Физик Backend)            |
|     Kaskad Kontrol: Konum (PSC/WPNAV) -> Açı (ATC_ANG) -> Hız PID (ATC_RAT)       |
+------------------+------------------------------------+---------------------------+
                   | UDP 14550                          | UDP 14560
                   v                                    v
+----------------------------------+  +---------------------------------------------+
|     Mission Planner 1.3.83       |  |     run_scenario.py & analyze.py (Python)   |
| - 43-WP Tarla İlaçlama Görevi    |  | - Otomatik Kalkış -> Hover -> Rota -> İniş  |
| - Canlı HUD ve Parametre İzleme  |  | - CSV Loglama, Metrikler & comparison.png   |
+----------------------------------+  +---------------------------------------------+
```

---

## Karşılaştırmalı Simülasyon Sonuçları

Aşağıdaki metrikler `run_scenario.py` ile koşulan üç senaryonun (`logs/log_empty.csv`, `logs/log_full.csv`, `logs/log_full_tuned.csv`) `analyze.py` tarafından işlenmesiyle doğrudan elde edilmiştir (`metrics_summary.csv`):

| Metrik | `empty` (12 kg Boş) | `full` (28 kg Ayarsız) | `full_tuned` (28 kg Ayarlı) | Mühendislik Yorumu |
|---|---:|---:|---:|---|
| **`MOT_THST_HOVER`** | `0.39` | `0.39` | **`0.68`** | Dolu tank için gerçek askı gazı (hover throttle) ön-beslemesi |
| **`WPNAV_ACCEL` (cm/s²)** | `250` | `250` | **`150`** | Köşe dönüşlerinde motor doyumunu önlemek için yatay ivme sınırı |
| **`ATC_ACC_R/P_MAX` (cdeg/s²)** | `1100` | `1100` | **`360`** | Ağır tank ve sıvı çalkantısını yumuşatmak için açısal ivme sınırı |
| **Kalkış Aşımı (`takeoff_overshoot_m`)** | `0.022 m` | `0.006 m` | `0.004 m` | Ağır kütle dikey aşımı sönümler ancak tırmanmayı yavaşlatır |
| **Hover İrtifa Std (`hover_alt_std_m`)** | `0.004 m` | `0.025 m` | `0.027 m` | Askı fazında cm mertebesinde dikey irtifa kararlılığı |
| **Hover Roll/Pitch Std (`deg`)** | `0.22°` | `6.17°` | **`5.08°`** | Yüksek atalet sönümü düşürür; ivme ayarı salınımı **%18 azaltır** |
| **Hover XY Gezintisi (`hover_xy_std_m`)** | `0.000 m` | `0.346 m` | **`0.214 m`** | Askıda yatay konum gezintisinde **%38 iyileşme** |
| **Rota XY RMS Hatası (`route_xy_err_rms_m`)** | `9.29 m` | `9.05 m` | `9.03 m` | 20×20 m kare rota boyunca basamak referansına olan RMS mesafe |
| **Rota Maks. İrtifa Hatası (`route_alt_err_max_m`)** | `0.04 m` | `0.41 m` | **`0.31 m`** | Köşe dönüşlerindeki irtifa çökmesinde **%24 azalma** |
| **Ortalama Motor PWM (`pwm_mean_hover_route`)** | `1561.0 µs` | `1633.9 µs` | `1634.8 µs` | Dolu tankta askı ve seyir için **+73 µs** daha yüksek motor komutu |
| **Maksimum Motor PWM (`pwm_max`)** | `1665 µs` | `1950 µs` | `1950 µs` | Ağır yükle yapılan dönüşlerde motorlar üst limite yaklaşır |
| **Motor PWM Dengesizliği (`pwm_imbalance_pct`)** | `%0.52` | `%7.62` | `%7.85` | Kaçık CG (+2.5 cm X, +1.5 cm Y) motorları asimetrik yükler |
| **Doyum Süresi (`sat_pct_time_pwm>1900`)** | `%0.00` | `%2.48` | `%3.07` | Manevra sırasında `1900 µs` üzerindeki motor çalışma oranı |
| **İniş Ortalama / Maks. $V_z$ (son 2 m)** | `0.287 / 0.503 m/s` | `0.270 / 0.500 m/s` | `0.271 / 0.500 m/s` | Son 2 metrede kontrollü dikey iniş hızı |
| **Maks. Titreşim (`vibe_max_xyz`) & Clip** | `0.025 m/s²` (`0`) | `0.068 m/s²` (`0`) | `0.072 m/s²` (`0`) | SITL'de ivmeölçer doyumu (`clipping`) yok; ağır yükte gövde titreşimi 3 kat artar |

---

## Mühendislik Yaklaşımı ve Tasarım Kararları (Soru – Cevap)

### 1. Fiziksel Dron Özelliklerini ve Uçuş Metriklerini Hangi Dosyada, Hangi Değişkenlerle Tanımladık?
- **Soru:** Dronun fiziksel özelliklerini (16 L tank, bom, kütle, ağırlık merkezi, atalet) ve uçuş kontrol değişkenlerini hangi dosyalarda verdik?
- **Cevap:** Gerçek bir hava aracında gövde fiziği mekanik tasarımda, kontrol parametreleri ise uçuş kontrolcüsünde bulunduğu için iki ayrı katman kullandık:
  1. **3B Fizik ve Kütle/CG Katmanı (`models/iris_agricultural/model.sdf`):**
     - Gövde altına `spray_tank_link` (16 L mavi silindirik tank + 80 cm ilaçlama bomu + 4 kırmızı nozul) ekledik ve `spray_tank_joint` ile `base_link`'e bağladık.
     - **Boş Tank (`empty`)**: `spray_tank_link` kütlesi `<mass>0.001</mass>` ve ağırlık merkezi `<pose>0 0 0 0 0 0</pose>` (tam merkezde).
     - **Dolu Tank (`full` ve `full_tuned`)**: `spray_tank_link` içinde ölçekli sıvı kütlesi `<mass>0.85</mass>`, artırılmış atalet tensörü (`ixx=0.012, iyy=0.012, izz=0.015`) ve kaçık ağırlık merkezi `<pose>0.025 0.015 -0.14 0 0 0</pose>` (**+2.5 cm ileri, +1.5 cm sağa, 14 cm aşağı**) tanımlandı.
  2. **Otopilot Kontrol ve Senaryo Katmanı (`run_scenario.py`):**
     - `--set` argümanıyla verilen ArduPilot parametrelerini (`MOT_HOVER_LEARN`, `MOT_THST_HOVER`, `WPNAV_ACCEL`, `ATC_ACC_R_MAX`, `ATC_ACC_P_MAX`, `LAND_SPEED`) MAVLink `PARAM_SET` mesajlarıyla SITL'e yazdı ve 20 Hz telemetri kaydını başlattı.

### 2. Neden Karşılaştırma Testinde 43 Waypoint'li Tarlanın Tamamı Yerine 20×20 m Kare Rotayı Uçtuk?
- **Soru:** `missions/tarla_mission.txt` içinde 43 waypoint'li tam tarla ilaçlama görevimiz varken neden 3 senaryo karşılaştırmasını `kalkış → 15 sn hover → 20×20 m kare rota → iniş` şeklinde yaptık?
- **Cevap:**
  1. **Görev İsteri:** Teknik görev dokümanı açıkça *"SITL üzerinde en az iki farklı kütle/yük senaryosu için kalkış–hover–kısa rota–iniş karşılaştırması yapın"* şartını koşmaktadır.
  2. **Faz İzolasyonu (Step Response Analizi):** Tam tarla görevinde dron sürekli ileri uçar; 15 saniyelik saf **hover (askıda kalma)** fazı yoktur. Oysa kontrol mühendisliğinde dikey basamak aşımını (`takeoff`), dış bozucu yokken açısal ve yatay salınımı (`hover`), hem Roll hem Pitch ekseninde 90° keskin köşe dönüşlerini (`route`) ve son 2 metredeki iniş hızını (`land`) ayrı ayrı ölçebilmek için fazların net ayrılması gerekir.
  3. **Birebir Tekrarlanabilirlik:** ~85 saniyelik standart test her üç senaryoda saniyesi saniyesine aynı komutları uygular; böylece `comparison.png` üzerinde üç eğri üst üste bindirildiğinde farklar doğrudan okunur.

### 3. Grafikler (`comparison.png`) ve Metrik Tablosu (`metrics_summary.csv`) Nasıl Üretildi?
- **Soru:** Rapordaki ve README'deki grafikler ile tablo verileri nasıl üretildi?
- **Cevap:**
  1. **Veri Toplama (`run_scenario.py`):** Uçuş sırasında ArduPilot SITL'den `MAV_CMD_SET_MESSAGE_INTERVAL` komutuyla `LOCAL_POSITION_NED` (20 Hz), `ATTITUDE` (20 Hz), `SERVO_OUTPUT_RAW` (10 Hz), `VIBRATION` (5 Hz) ve `EKF_STATUS_REPORT` (2 Hz) mesajları çekildi. Her satıra o anki uçuş fazı (`arm`, `takeoff`, `hover`, `route`, `land`), hedef konum (`sp_x, sp_y, sp_z`), gerçek konum/hız (`x, y, z, vx, vy, vz`), açılar (`roll, pitch, yaw`) ve 4 motorun PWM değerleri (`pwm1..pwm4`) yazılarak `logs/log_*.csv` dosyaları oluşturuldu.
  2. **İstatistiksel Hesaplama ve Çizim (`analyze.py`):** `Pandas`, `NumPy` ve `Matplotlib` kullanan `analyze.py` betiği:
     - `d[d.phase == "takeoff"]` filtresiyle kalkıştaki maksimum irtifadan hedef irtifayı çıkararak **kalkış aşımını (`takeoff_overshoot_m`)**,
     - `d[d.phase == "hover"]` filtresiyle 15 saniyelik askı süresince $\sqrt{\sigma_{\text{roll}}^2 + \sigma_{\text{pitch}}^2}$ formülüyle **açısal salınımı (`hover_roll_pitch_std_deg`)** ve $\sqrt{\sigma_x^2 + \sigma_y^2}$ ile **yatay gezintiyi (`hover_xy_std_m`)**,
     - `d[d.phase == "route"]` filtresiyle köşe dönüşlerindeki **maksimum irtifa çökmesini (`route_alt_err_max_m`)**,
     - `pwm1..pwm4` sütunlarından ortalama PWM'i, maksimum PWM'i, `(max_motor_ort - min_motor_ort) / (genel_ort - 1000)` formülüyle **CG kaynaklı motor dengesizliğini (`pwm_imbalance_pct`)** ve `PWM > 1900 µs` geçen zaman oranını (`sat_pct_time_pwm>1900`),
     - `d[(d.phase == "land") & (alt < 2.0)]` filtresiyle **son 2 metredeki ortalama ve maksimum dikey iniş hızını (`landing_vz_*`)** hesapladı ve 4 alt grafikten (İrtifa, X-Y Yörünge, Roll/Pitch, Ortalama PWM) oluşan `figures/comparison.png` dosyasını çizdi.

### 4. Hangi Veriler Bizi Yönlendirdi ve `full_tuned` Parametrelerini Neye Göre Seçtik?
- **Soru:** Boş (`empty`) ve Ayarsız Dolu (`full`) koşular arasındaki hangi bulgular `full_tuned` ayarlarını belirledi?
- **Cevap:**
  - **Bulgu 1 (Askı Gazı Kayması):** `empty` koşusunda `1561.0 µs` olan ortalama PWM, `full` koşusunda **`1633.9 µs`**'ye çıktı (`+73 µs`). Kontrolcü `MOT_THST_HOVER=0.39` değerinde kaldığı için irtifa kontrolcüsünün I-terimi bu farkı kapatana kadar dron gecikmeli tepki verdi. **Karar:** `MOT_THST_HOVER=0.68` yapılarak ağır kütlenin askı gazı doğrudan ileri-besleme (feed-forward) olarak tanımlandı.
  - **Bulgu 2 (Köşe Dönüşlerinde Motor Doyumu ve 41 cm İrtifa Çökmesi):** `full` koşusunda 90° köşe dönüşlerinde motor PWM'i **`1950 µs`**'ye vurdu (`%2.48` doyum süresi) ve dron köşelerde **`0.41 m` irtifa kaybetti**. **Karar:** Yatay ivme sınırı `WPNAV_ACCEL` `250`'den **`150 cm/s²`**'ye, açısal ivme sınırları `ATC_ACC_R_MAX` ve `ATC_ACC_P_MAX` `1100`'den **`360`**'a düşürüldü. **Sonuç:** Köşe dönüşlerindeki irtifa çökmesi `0.41 m`'den **`0.31 m`'ye (%24 iyileşme)**, hover yatay gezintisi `0.346 m`'den **`0.214 m`'ye (%38 iyileşme)**, açısal salınım ise `6.17°`'den **`5.08°`'ye (%18 iyileşme)** düştü.

### 5. Kontrol Döngülerinin Rol Ayrımı ve PID Dışı Yöntemler (Gain Scheduling, LQR, MPC)
- **Soru:** ArduPilot'ta Rate PID, Position Loop, Feed-Forward, Acceleration Limit, Notch Filter ve AutoTune'un rolleri nedir? LQR veya MPC kullanılmalı mıdır?
- **Cevap:**
  - **Roll/Pitch/Yaw Rate PID (`ATC_RAT_*`, 400 Hz):** Jiroskop açısal hız hatasını düzelten en iç döngüdür; kütle/atalet artışına (`P`, `D`) ve kaçık CG momentine (`I` terimi) doğrudan karşı koyar.
  - **Altitude / Position Loop (`PSC_*`, `WPNAV_*`, 50–100 Hz):** Konum ve hız hatasını hedef eğim açısına ve hedef dikey ivmeye çeviren dış döngüdür.
  - **Feed-Forward (`ATC_RAT_*_FF`, `MOT_THST_HOVER`):** Hata oluşmasını beklemeden komut türevine ve bilinen kütleye göre anında motor çıktısı üretir.
  - **Acceleration Limit (`ATC_ACC_*`, `WPNAV_ACCEL`):** Komutların değişim hızını sınırlayarak ağır tankın savrulmasını, sıvı çalkantısını (sloshing) ve aktüatör doyumunu önler.
  - **Harmonik Notch / FFT Filtreleri (`INS_HNTCH_*`):** Motor devir frekansındaki mekanik titreşimleri D-terimine ulaşmadan bastırır.
  - **AutoTune / QuikTune:** `QuikTune` sahada güvenli ilk kazanç tahmini yapar; `AutoTune` ise gövdeye otomatik darbeler vererek kazançları ölçer. **Ancak yarı dolu sıvı tankıyla AutoTune yapılmamalıdır** (çalkantı ölçümü bozar); boş ve eşdeğer katı yükte ayrı ayrı yapılıp enterpole edilmelidir.
  - **Gain Scheduling vs. LQR / MPC:** İlaçlama sırasında sıvı kütlesi 16 L'den 0 L'ye yavaşça azaldığı için debimetre veya tank seviye sensörüne bağlı **Gain Scheduling (Kazanç Çizelgeleme)** ArduPilot ile tam uyumludur, düşük risklidir ve en doğru mühendislik çözümüdür. Buna karşılık **LQR ve MPC**, ArduPilot'un standart 400 Hz kaskad mimarisiyle doğrudan uyumlu değildir; sıvı çalkantısı durumlarının gerçek zamanlı tahminini gerektirir ve model belirsizliğinde yüksek kararsızlık riski taşır.

---

## Kurulum ve Çalıştırma (3 Terminal)

### Önkoşullar
- Ubuntu 24.04 LTS, ArduPilot SITL (`ArduCopter V4.8.0-dev`), Gazebo Sim (`8.11.0`) + `ardupilot_gazebo` eklentisi
- Python 3.12 sanal ortamı:
  ```bash
  source ~/venv-ardupilot/bin/activate
  pip install -r requirements.txt
  ```

### Adım 1: Model ve Dünya Dosyalarını Gazebo'ya Kopyalayın
```bash
cp -r models/iris_agricultural ~/ardupilot_gazebo/models/
cp worlds/farm_field.sdf ~/ardupilot_gazebo/worlds/
```

### Adım 2: Simülasyonu ve Senaryo Koşucusunu Başlatın

**Terminal 1 – Gazebo Sim (3B Tarla ve 16 L İlaçlama Dronu):**
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
gz sim -v4 -r farm_field.sdf
```

**Terminal 2 – ArduPilot Copter SITL:**
```bash
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot/Tools/autotest
./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map \
  --add-param-file=$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm
```
*(MAVProxy konsolunda `output add 127.0.0.1:14560` komutuyla senaryo koşucu portunu açın.)*

**Terminal 3 – Otomatik Karşılaştırma Koşuları ve Analiz:**
```bash
source ~/venv-ardupilot/bin/activate
cd ~/spray_drone_sitl

# 1. Boş Tank (12 kg)
python3 run_scenario.py --label empty --mass 12 --set MOT_HOVER_LEARN=0

# 2. Dolu Tank - Ayarsız (28 kg)
python3 run_scenario.py --label full --mass 28 --set MOT_HOVER_LEARN=0

# 3. Dolu Tank - Ayarlı (28 kg)
python3 run_scenario.py --label full_tuned --mass 28 \
  --set MOT_HOVER_LEARN=0 MOT_THST_HOVER=0.68 WPNAV_ACCEL=150 ATC_ACC_R_MAX=360 ATC_ACC_P_MAX=360 LAND_SPEED=40

# Metrik özet tablosu ve karşılaştırma grafiğini üret
mv log_*.csv log_*_meta.json log_*_events.txt logs/
python3 analyze.py logs/log_empty.csv logs/log_full.csv logs/log_full_tuned.csv
mv comparison.png figures/
```

---

## Proje Dizin Yapısı

```text
spray_drone_sitl/
├── README.md                          # Proje dokümantasyonu, mimari, metrikler ve Soru-Cevap
├── report_draft.md                    # Teknik raporun Markdown kaynağı
├── report/
│   └── DRONEQUBE_Teknik_Rapor_Begum_Aydogan_Cicin.pdf  # 5 sayfalık teslim raporu (PDF)
├── run_scenario.py                    # Otomatik MAVLink uçuş senaryosu ve telemetri kaydedici
├── analyze.py                         # İstatistiksel metrik ve karşılaştırma grafiği üretici
├── metrics_summary.csv                # Üç senaryonun hesaplanmış karşılaştırma metrikleri
├── requirements.txt                   # Python bağımlılıkları
├── versions.txt                       # Kullanılan yazılım, donanım ve simülatör sürümleri
├── figures/
│   ├── demo_field_mission.gif         # 16 L ilaçlama dronu kalkış ve tarla görevi animasyonu
│   ├── demo_sitl_scenarios.gif        # Otomatik senaryo koşusu ve hassas iniş animasyonu
│   ├── comparison.png                 # 4 panelli zaman serisi karşılaştırma grafiği
│   ├── gazebo_farm_field.jpg          # Gazebo tarlası + Mission Planner + MAVProxy görünümü
│   └── spray_tank_landing_pad.jpg     # 16 L mavi tank, bom, nozullar ve iniş pedi yakın planı
├── logs/
│   ├── log_empty.csv                  # Koşu A (12 kg Boş Tank) 20 Hz telemetri verisi
│   ├── log_full.csv                   # Koşu B (28 kg Dolu Tank - Ayarsız) telemetri verisi
│   ├── log_full_tuned.csv             # Koşu C (28 kg Dolu Tank - Ayarlı) telemetri verisi
│   ├── *_meta.json & *_events.txt     # Her koşuda okunan parametreler ve FC durum mesajları
│   └── failed/
│       └── failed_trials_notes.txt    # Başarısız denemeler (itki doyumu, atalet taklası) ve kök nedenleri
├── missions/
│   └── tarla_mission.txt              # Mission Planner 43 waypoint'li tarla ilaçlama görevi
├── models/
│   └── iris_agricultural/             # 16 L ilaç tankı, bom ve nozul eklenmiş Gazebo SDF modeli
├── params/
│   ├── backup_original.parm           # Orijinal fabrika parametre yedeği (1445 parametre)
│   ├── before_empty.parm              # Boş tank konfigürasyonu
│   └── before_full_tuned.parm         # Ayarlı dolu tank konfigürasyonu
└── worlds/
    └── farm_field.sdf                 # 65×95 m tarla ve hassas iniş pedi içeren Gazebo dünyası
```

---

## Başarısız Denemeler ve Simülatör Sınırlamaları

1. **İtki Ölçeklemesi Olmadan 28 kg Kalkış Denemesi (`Thr 100` Doyumu):**
   - Standart Iris rotor sabitleriyle (`forceConstant = 8.54858e-06`) kütle doğrudan 28 kg yapıldığında maksimum toplam itki (~34 N) yerçekimi kuvvetini (274.6 N) yenememiş ve araç yerden kesilememiştir.
2. **Atalet Tensörü Ölçeklenmeden İtki Artırımı (`AngErr=170>30` Taklası):**
   - Sadece rotor itkisi artırılıp $I_{xx}, I_{yy}$ atalet momentleri küçük bırakıldığında ilk waypoint dönüşünde aşırı açısal ivme nedeniyle araç takla atmış ve `Crash: Disarming: AngErr=170>30` hatasıyla motorları kesmiştir.
3. **Simülatör Sınırlamaları (Gerçek Saha Farkları):**
   - Gazebo'da tank içi sıvı katı bir kütle ve sabit CG ofseti olarak modellenmiştir; gerçek uçuşta **sıvı çalkantısı (sloshing)** ~0.8–1.5 Hz arasında gecikmeli faz kayması yaratır ve tank bölmeleri (baffle) + düşük ivme limitleri (`WPNAV_ACCEL`, `ATC_ACC_*`) gerektirir.
   - Gazebo zemin etkileşimi çok rotorlu **yer etkisini (ground effect)** ve ekin kanopisi türbülansını tam simüle etmez; sahada barometrik yastıklanmaya karşı aşağı bakan **1D LiDAR (`RNGFND1`)** kullanılmalıdır.
