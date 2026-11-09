# DRONEQUBE Teknik Görev Savunma ve Mülakat Rehberi (Soru – Cevap)

Bu doküman, **16 L Tarımsal İlaçlama Dronu Kalkış, İniş ve Kontrol İyileştirme** çalışmasında yaptığımız her teknik adımı, kodların ve grafiklerin nasıl üretildiğini ve mülakatta sorulabilecek tüm mühendislik sorularını temelden başlayarak **Soru – Cevap** formatında açıklar.

---

## BÖLÜM 1: Grafikler (`comparison.png`) ve Tablo (`metrics_summary.csv`) Nasıl Üretildi?

### Soru 1.1: "Bu grafikleri (`comparison.png`) ve tablodaki sayıları nasıl ürettin? Elle mi çizdin, Mission Planner'dan mı aldın, yoksa Python ile mi?"
- **Cevap:**
  - Grafiklerin ve tablodaki sayıların tamamı **gerçek ArduPilot SITL + Gazebo Sim uçuşlarından alınan canlı MAVLink telemetri verileriyle Python'da otomatik olarak** üretildi.
  - Süreç iki aşamalı çalıştı:
    1. **Uçuş ve Veri Kaydı (`run_scenario.py`):** Python'da `pymavlink` kütüphanesini kullanarak ArduPilot SITL'in `14560` UDP portuna bağlandım. Uçuş başlamadan önce otopilottan 5 farklı telemetri mesajını yüksek hızda göndermesini istedim (`MAV_CMD_SET_MESSAGE_INTERVAL`):
       - `LOCAL_POSITION_NED` (20 Hz – saniyede 20 kez $x, y, z$ konum ve $v_x, v_y, v_z$ hız),
       - `ATTITUDE` (20 Hz – saniyede 20 kez Roll, Pitch, Yaw açıları),
       - `SERVO_OUTPUT_RAW` (10 Hz – 4 motorun gerçek PWM sinyalleri: `pwm1..pwm4`),
       - `VIBRATION` (5 Hz – $X, Y, Z$ ivmeölçer titreşimleri ve `clipping` sayacı),
       - `EKF_STATUS_REPORT` (2 Hz – EKF3 sağlık bayrakları).
       Uçuş boyunca her 50 milisaniyede bir (`poll()` fonksiyonu ile) bu verileri o anki uçuş fazı (`takeoff`, `hover`, `route`, `land`) ve hedef koordinatlarla (`sp_x, sp_y, sp_z`) birleştirip `logs/log_empty.csv`, `logs/log_full.csv` ve `logs/log_full_tuned.csv` dosyalarına kaydettim. Her bir CSV dosyasında yaklaşık **1170–1198 satır gerçek uçuş verisi** bulunuyor.
    2. **Metrik Hesaplama ve Grafik Çizimi (`analyze.py`):** Bu üç CSV dosyasını `analyze.py` betiğine verdim (`python3 analyze.py logs/log_empty.csv logs/log_full.csv logs/log_full_tuned.csv`). Betik `Pandas` ve `NumPy` ile her fazı filtreleyip istatistikleri hesapladı, `metrics_summary.csv` tablosunu oluşturdu ve `Matplotlib` ile 4 panelli `figures/comparison.png` grafiğini çizdi.

---

### Soru 1.2: "`comparison.png` grafiğindeki 4 panel sırasıyla neyi gösteriyor ve grafikteki eğrileri nasıl okumalıyız?"
- **Cevap:** `figures/comparison.png` yukarıdan aşağıya 4 alt grafikten oluşur ve üzerinde **Mavi (`empty`)**, **Turuncu (`full` - ayarsız ağır)** ve **Yeşil (`full_tuned` - ayarlı ağır)** olmak üzere 3 uçuş üst üste çizilmiştir:
  1. **1. Panel – İrtifa (`irtifa [m]` – Zaman `t [s]`):**
     - $0–12\text{ sn}$ arası **Kalkış (`takeoff`)**: Hedef 5.0 metredir (`-d.z`).
     - $12–28\text{ sn}$ arası **Sabit Askı (`hover`)**: Üç senaryo da 5.0 metrede düz çizgi halinde tutunur.
     - $28–68\text{ sn}$ arası **Kare Rota (`route`)**: Köşe dönüşlerinde dron yana yattığında dikey itki azaldığı için turuncu eğride (`full`) **41 cm'ye varan irtifa çökmeleri/dalgalanmaları** görülür. Yeşil eğride (`full_tuned`) ise `MOT_THST_HOVER=0.68` ve ivme sınırlaması sayesinde bu çökme **31 cm'ye (%24 daha az)** inmiştir.
     - $68–85\text{ sn}$ arası **İniş (`land`)**: Araç yumuşak bir eğimle sıfıra (iniş pedine) iner.
  2. **2. Panel – Yatay Yörünge (`y [m]` – `x [m]`):**
     - Yukarıdan (kuşbakışı) 20×20 metrelik kare rotayı gösterir: $(0,0) \to (20,0) \to (20,20) \to (0,20) \to (0,0)$.
     - Mavi eğri (`empty`) köşeleri keskin ve savrulmadan dönerken, ağır yükte (`full` ve `full_tuned`) artan atalet nedeniyle köşelerde hafif kavis (yay çizme) görülür; `full_tuned` (`WPNAV_ACCEL=150`) köşeye daha kontrollü girer.
  3. **3. Panel – Roll ve Pitch Açıları (`aci [deg]` – Zaman `t [s]`):**
     - Dronun ileri/geri (Pitch) ve sağa/sola (Roll) yatma açılarını gösterir.
     - Boş dronda (`empty`) açılar çok temiz ve düşük genlikliyken, ağır dronda (`full`) köşe dönüşlerinde **27°'ye varan sert yatmalar** ve hover sırasında **`6.17°` salınım** oluşur. `full_tuned` koşusunda açısal ivme limiti (`ATC_ACC_R/P_MAX=360`) sayesinde açısal salınım **`5.08°`'ye (%18 azalma)** çekilmiştir.
  4. **4. Panel – Ortalama Motor PWM (`PWM [us]` – Zaman `t [s]`):**
     - En çarpıcı mühendislik grafiği budur:
     - **Mavi çizgi (`empty`):** Tüm uçuş boyunca **`1561 µs`** bandında seyreder.
     - **Turuncu ve Yeşil çizgiler (`full` ve `full_tuned`):** Ağır kütle nedeniyle **`1634 µs`** bandına (**+73 µs yukarı**) taşınmıştır ve köşe dönüşlerinde anlık olarak `1800–1950 µs` tepe noktalarına vurur. Bu grafik, dolu tankın motorlardan ne kadar ekstra güç çektiğini tek bakışta kanıtlar.

---

## BÖLÜM 2: Alınan Mühendislik Kararları ve Parametre Seçimleri

### Soru 2.1: "Dolu tank (`full_tuned`) için neden tam olarak bu 5 parametreyi (`MOT_THST_HOVER=0.68`, `WPNAV_ACCEL=150`, `ATC_ACC_R_MAX=360`, `ATC_ACC_P_MAX=360`, `LAND_SPEED=40`) seçtin?"
- **Cevap:** Her bir parametreyi `full` (ayarsız dolu tank) koşusunda gördüğümüz somut bir problemi çözmek için seçtim:
  1. **`MOT_THST_HOVER = 0.68` (Fabrika değeri: `0.39`):**
     - *Neden?* Boş dron `%39` gazda askıda kalırken, 28 kg dolu dron yaklaşık `%63–68` gazda askıda kalabiliyor. Eğer bu parametreyi `0.39` bırakırsak, ArduPilot her irtifa düzeltmesinde ve kalkışta motorlara önce `%39` gaz veriyor, dron çökünce `PSC_ACCZ_I` (dikey ivme integratörü) yavaş yavaş dolup gazı artırıyor. `0.68` vererek kontrolcüye *"bu aracın ağırlığını taşımak için başlangıç noktan %68 gazdır"* bilgisini (feed-forward) doğrudan verdik.
  2. **`WPNAV_ACCEL = 150 cm/s²` (Fabrika değeri: `250 cm/s²`):**
     - *Neden?* $F = m \cdot a$ gereği kütle 2.33 katına çıktığında, aynı `2.5 m/s²` yatay ivmeyi üretmek için 2.33 kat daha fazla yatay itki (dolayısıyla daha büyük yatma açısı ve motor PWM'i) gerekir. `full` koşusunda köşe dönüşlerinde motor PWM'inin `1950 µs` doyuma vurmasının ve irtifanın `41 cm` çökmesinin sebebi buydu. Yatay ivmeyi `1.5 m/s²`'ye (`150 cm/s²`) çekerek köşelere daha yumuşak girmesini sağladık ve irtifa çökmesini `%24` azalttık.
  3. **`ATC_ACC_R_MAX = 360` ve `ATC_ACC_P_MAX = 360` (Fabrika değeri: `1100`):**
     - *Neden?* Gövde altına asılı 16 L sıvı tankı hem $I_{xx}, I_{yy}$ atalet momentini büyütür hem de gerçekte sıvı çalkantısı (sloshing) yaratır. Gövdenin saniyede açısal hızlanma limitini `1100`'den `360`'a çekerek sert açısal darbeleri (jerk/twitch) engelledik; bu sayede hover açısal salınımı `%18`, yatay gezinti ise `%38` azaldı.
  4. **`LAND_SPEED = 40 cm/s` (Fabrika değeri: `50 cm/s`):**
     - *Neden?* İnişte kinetik enerji $E_k = \frac{1}{2} m v_z^2$'dir. Kütle 28 kg olduğunda `50 cm/s` ile yere vurmak iniş takımlarına 2.33 kat fazla darbe enerjisi bindirir ve yer etkisinde (ground effect) zıplamaya yol açar. Son aşama iniş hızını `40 cm/s`'ye düşürerek darbe enerjisini yaklaşık `%36` ($1 - 0.4^2/0.5^2$) azalttık.

---

### Soru 2.2: "Neden `MOT_HOVER_LEARN=0` yaptın? Normalde ArduPilot'ta `MOT_HOVER_LEARN=2` açık kalması daha iyi değil mi?"
- **Cevap:**
  - **Çok kritik bir test mühendisliği kararıdır:** Gerçek saha operasyonunda elbette `MOT_HOVER_LEARN=2` (öğren ve kaydet) açık olmalıdır ki ilaç azaldıkça otopilot yeni askı gazını öğrensin.
  - Ancak **kontrollü bilimsel deney (A/B testi)** yaparken eğer `MOT_HOVER_LEARN=2` açık bırakılırsa, `full` (ayarsız) koşusunun ortasında otopilot kendi kendine `MOT_THST_HOVER` değerini değiştirmeye başlar ve *"ayarsız"* koşu ile *"ayarlı"* koşu birbirine karışır. Deneyin tekrarlanabilir ve adil olması için test sırasında öğrenmeyi `MOT_HOVER_LEARN=0` ile dondurduk.

---

### Soru 2.3: "Ayarlı koşuda (`full_tuned`) açısal salınım `6.17°`'den `5.08°`'ye düştü ama boş drondaki `0.22°` seviyesine inmedi. Neden? Bunu tamamen `0.5°` altına indirmek için ne yapardın?"
- **Cevap:**
  - Bu sonuç kontrol teorisi açısından beklenen ve çok öğretici bir bulgudur:
  - Biz `full_tuned` koşusunda **dış döngü ivme sınırlarını (`WPNAV_ACCEL`, `ATC_ACC_R/P_MAX`) ve askı gazını (`MOT_THST_HOVER`)** iyileştirdik; ancak **en iç döngü olan 400 Hz'lik Rate PID kazançlarına (`ATC_RAT_RLL_P/I/D` ve `ATC_RAT_PIT_P/I/D`)** dokunmadık.
  - Gövde atalet momenti ($J$) büyüdüğünde, doğal frekans $\omega_n \propto \sqrt{K_p / J}$ ve sönüm oranı $\zeta \propto K_d / \sqrt{K_p J}$ düşer. Yani mevcut `ATC_RAT_RLL_D` (türev/sönüm kazancı) ağır gövdeyi frenlemeye yetmez.
  - Salınımı tamamen `< 0.5°` altına indirmek için sahada/simülasyonda **bir sonraki adım**, ağır kütleyle `QuikTune` veya `AutoTune` çalıştırarak `ATC_RAT_RLL_P`, `ATC_RAT_RLL_D` ve `ATC_RAT_PIT_P`, `ATC_RAT_PIT_D` kazançlarını artan atalet momenti oranında büyütmektir.

---

### Soru 2.4: "Gazebo'da karşılaştığın başarısız denemeler nelerdi ve bunları nasıl çözdün?"
- **Cevap:** Üç önemli başarısız deneme yaşadık ve hepsini `logs/failed/failed_trials_notes.txt` içinde belgeledik:
  1. **Deneme 1 – Motor İtkisi Ölçeklenmeden 28 kg Yükleme (`Thr 100` Hatası):**
     - Standart Gazebo `iris` modelinin motorları 1.5 kg'lık hobi dronu içindir (maksimum toplam itki ~3.5 kgf). Kütleyi doğrudan 28 kg yaptığımızda MAVProxy konsolunda gaz `%100`'e (`Thr 100`) vurduğu halde dron yerden 1 cm bile kalkamadı.
  2. **Deneme 2 – Sadece İtkiyi Artırıp Atalet Momentini ($I_{xx}, I_{yy}$) Küçük Bırakma (`AngErr=170>30` Taklası):**
     - Motor itki katsayısını (`forceConstant`) büyütüp atalet tensörünü büyütmediğimizde, en ufak bir Roll/Pitch komutunda $\ddot{\theta} = \tau / I$ çok devasa bir açısal ivme üretti; dron kalkıştan hemen sonra ilk waypoint'e dönerken havada takla atıp yere çakıldı (`AP: Crash: Disarming: AngErr=170>30`).
     - **Çözüm:** Kütle, rotor itki katsayısı ve $I_{xx}, I_{yy}, I_{zz}$ atalet tensörünü fiziksel olarak birbiriyle uyumlu ölçekledik.
  3. **Deneme 3 – `run_scenario.py` İçinde Parametre Okuma Kayması ve İniş Pedi Yüksekliği:**
     - `get_param()` fonksiyonu MAVLink tamponunda bekleyen eski `PARAM_VALUE` mesajını okuduğu için parametre değerleri bir sağa kayıyordu; ayrıca `precision_landing_pad` yerden `19 cm` yüksek olduğu için `-lp.z < 0.05` şartı sağlanmıyor ve script inişten sonra 300 saniye bekliyordu.
     - **Çözüm:** `get_param()` içine `msg.param_id == name` doğrulaması, iniş döngüsüne ise `Disarming motors` / `MAV_MODE_FLAG_SAFETY_ARMED` kontrolü ekleyerek sorunu çözdük.

---

### Soru 2.5: "Mülakatta 'Güvenlik fonksiyonlarıyla ilgili ne yaptın?' diye sorulursa ne söylemeliyim?"
- **Cevap:**
  - Görev dokümanında açıkça *"Güvenlik fonksiyonlarını geçersiz kılacak öneriler sunmayın"* uyarısı var.
  - Hem raporda hem testlerde `ARMING_CHECK=1` (tüm uçuş öncesi sensör/EKF/GPS kontrolleri aktif), `BATT_FS_LOW_ACT=2` (düşük pilde otomatik RTL), `BATT_FS_CRT_ACT=1` (kritik pilde otomatik Land), `FS_GCS_ENABLE=1` (yer istasyonu kopmasında RTL) ve `FENCE_ENABLE=1` (tarla sınırı ve 30 m tavan geofence) parametrelerinin **kesinlikle kapatılmaması gerektiğini** vurguladım. Ağır bir 28 kg ilaçlama dronunda güvenlik kontrollerini kapatmak sahada kabul edilemez bir risktir.
