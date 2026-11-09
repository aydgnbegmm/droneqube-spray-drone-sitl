# 16 L İlaçlama Dronu: Kalkış, İniş ve Kontrol İyileştirme Planı

**Aday:** Begüm Aydoğan Çiçin  
**Ortam ve Sürümler:** Ubuntu 24.04.4 LTS · ArduCopter V4.8.0-dev (`b0af1735`) · Gazebo Sim 8.11.0 (`ardupilot_gazebo`) · Mission Planner 1.3.83 · Python 3.12.3 (`pymavlink 2.4.50`).  
**Not:** Bu çalışmadaki tüm kütle, atalet, itki ve tank boyutları gerçek ürün verisi olmayıp açıkça etiketlenmiş **varsayımlardır**.

---

## 1. Amaç, Varsayımlar ve Kontrol Blok Diyagramı

Tarımsal ilaçlama dronlarında 16 L sıvı yükü, görev başında ve sonunda kütle, atalet momenti ve ağırlık merkezi (CG) üzerinde büyük bir değişim yaratır. ArduPilot'ta `MOT_THST_HOVER` fiziksel bir sensör okuması değil kontrolcü parametresi olduğundan, kütle değiştiğinde hover referansı, ivme sınırları ve iç döngü kazançları uyarlanmazsa irtifa çökmesi, dönüş aşımı ve motor doygunluğu oluşur.

### Varsayımlar Tablosu (Etiketli)
| Parametre | `empty` (Boş Tank - Varsayım) | `full` (16 L Dolu Tank - Varsayım) | Mühendislik Etkisi |
|---|---|---|---|
| **Toplam Kütle** | `12.0 kg` (Gazebo ölçekli taban) | `28.0 kg` (+16 L su yükü eşdeğeri) | ~2.33 kat kütle artışı, hover gazı yükselir |
| **Ağırlık Merkezi (CG)** | Merkez (`0, 0, 0 m`) | Ofsetli (`+2.5 cm X, +1.5 cm Y, -14 cm Z`) | Motorlar arası sabit PWM dengesizliği yaratır |
| **Atalet (`ixx, iyy, izz`)** | Taban atalet tensörü | Kütleyle orantılı büyütülmüş (+%150) | Açısal ivme ($\alpha=\tau/I$) düşer, faz gecikmesi artar |
| **Hedef Hover Gazı** | `~0.35 - 0.39` (~1560 µs PWM) | `~0.55 - 0.68` (~1635 µs PWM) | Dolu halde en az %35–40 itki rezervi korunmalı |

### Kontrol Döngüsü Blok Diyagramı
```
[Mission / Guided] ──► [Konum & İrtifa (PSC_POS / VEL / ACC)] ──► [İvme Limiti (WPNAV_ACCEL / ATC_ACC_*_MAX)]
                                                                                │
                                                                                ▼
[Dinamik (Kütle, CG, Rüzgâr)] ◄── [ESC / Motor] ◄── [Motor Karıştırıcı] ◄── [Notch / LPF] ◄── [Rate PID + FF] ◄── [Açı (ATC_ANG)]
              │                                      (THST_HOVER/EXPO)      (INS_HNTCH_*)     (ATC_RAT_*)
              └──────────────► [IMU, GPS, Baro ──► EKF3 Durum Kestirimi] ──────────────────────────────────────────┘
```

---

## 2. Boş/Dolu Tank Etkisi ve İnceleme Yöntemi

| Fiziksel Etki | Beklenen Davranış | İnceleme ve Ölçüm Yöntemi |
|---|---|---|
| **Kütle Artışı** | Hover gazı yükselir, dikey tepki yavaşlar, iniş momentumu artar | `run_scenario.py` ile `pwm_mean_hover_route`, `takeoff_overshoot_m`, `route_alt_err_max_m` |
| **Atalet Artışı** | Rate döngüsü sönümü azalır, açısal salınım ve köşe aşımı büyür | `hover_roll_pitch_std_deg`, `route_max_tilt_deg` ve `comparison.png` açı grafikleri |
| **CG Kayması** | Simetrik olmayan kol yükü nedeniyle motorlar farklı PWM üretir | `SERVO_OUTPUT_RAW` üzerinden `pwm_imbalance_pct` hesabı |
| **İtki Rezervi** | Manevra ve rüzgâr düzeltmesi için kalan tepe motor marjı daralır | `pwm_max` ve `sat_pct_time_pwm>1900` (1900 µs üzeri süre yüzdesi) |
| **Rüzgâr** | Sabit yatış açısı (tilt) ve rota YKE (RMS) hatası artar | `SIM_WIND_SPD` / `SIM_WIND_DIR` ile yan rüzgâr testi, `route_xy_err_rms_m` |
| **Yer Etkisi (IGE)** | Son 2 m'de yastıklanma veya ağır yükte yüksek temas hızı | İniş fazı son 2 m'de `landing_vz_mean_last2m_mps` ve `landing_vz_max_mps` |
| **Titreşim** | Kütle değişince motor devir frekansı kayar, EKF/IMU gürültüsü artar | `VIBRATION` mesajından `vibe_max_xyz`, `clipping_count`; sahada yüksek hızlı FFT logu |

---

## 3. Kontrol Yaklaşımı, Rol Ayrımı ve PID Dışı Yöntemlerin Tartışması

### 3.1 Kontrol Bileşenlerinin Rol Ayrımı
| Bileşen | Görevi | İlgili Parametreler | Kütle Değişimindeki Rolü |
|---|---|---|---|
| **Rate PID** | İç döngü; açısal hızı takip eder | `ATC_RAT_RLL/PIT/YAW_P/I/D` | Atalet büyüyünce sönüm azalır; P ve D yeniden ayarlanmalıdır |
| **Feed-Forward** | Hata oluşmasını beklemeden komut takibini hızlandırır | `ATC_RAT_*_FF` | Faz gecikmesini azaltır ancak tek başına kararlılık sağlamaz |
| **Açı Döngüsü** | Hedef açıyı açısal hız komutuna çevirir | `ATC_ANG_*_P` | Dolu tankta yumuşak geçiş için aşırı yüksek tutulmaz |
| **Altitude / Position** | İrtifa ve yatay konum takibi | `PSC_ACCZ_P/I`, `PSC_POSZ_P`, `WPNAV_*` | `MOT_THST_HOVER` doğruluğu kalkış/iniş ve dönüş irtifa kalitesini belirler |
| **İvme Limiti** | Manevra sertliğini ve eğimi sınırlar | `WPNAV_ACCEL`, `ATC_ACC_R/P_MAX`, `ANGLE_MAX` | Dolu halde düşürülerek (`360 deg/s²`, `150 cm/s²`) itki rezervi korunur ve çalkantı azaltılır |
| **Notch / FFT Filtre** | Motor/pervane gürültüsünü gyro'dan ayıklar | `INS_HNTCH_*`, `INS_GYRO_FILTER` | Kazanç artırmanın önkoşuludur; kütleyle RPM değiştiği için throttle/ESC-RPM tabanlı seçilir |
| **QuikTune / AutoTune** | Başlangıç ve nihai PID kazançlarını bulur | `QUIK_ENABLE`, `AUTOTUNE_AXES/AGGR` | Önce filtre doğrulanır, sonra QuikTune, ardından güvenli ortamda AutoTune koşulur |

### 3.2 PID Dışı Yaklaşımların Değerlendirmesi
| Yaklaşım | ArduPilot Uyumluluğu | Risk | Gerçekten Gerekli mi? |
|---|---|---|---|
| **Gain Scheduling** | Kütleye göre otomatik tablo yok; `MOT_THST_EXPO`, akü gerilim telafisi ve **iki parametre seti** (boş/dolu) ile pratik karşılığı var | **Düşük** | **Orta:** Boş/dolu farkı büyük olduğundan iki profil veya dolu ağırlıklı uzlaşma tune'u yeterlidir |
| **LQR** | Standart firmware'de yok; özel kontrolcü veya offboard gerekir | **Yüksek:** Doğrulama ve güvenlik yükü | **Hayır:** Bu görev profili için ek kazancı düşüktür |
| **MPC** | Özel mimari ve yüksek hesap yükü gerektirir; sıvı çalkantısında model uyumsuzluğuna hassastır | **Yüksek** | **Hayır:** ArduCopter'ın PID + FF + EKF3 yapısı ve doğru hover/filtre ayarı yeterlidir |

**Mühendislik Kararı:** Güvenlik fonksiyonları (failsafe, geofence, arming kontrolleri) olduğu gibi korunarak ArduCopter'in standart PID + doğru `MOT_THST_HOVER` + Notch filtre + ivme limiti yapısı tercih edilmiştir; kontrolcü mimarisini değiştirmek sahada orantısız doğrulama riski getirir.

---

## 4. SITL + Gazebo Test Sonuçları ve Başarısız Denemeler

Üç koşu (`empty`, `full`, `full_tuned`) aynı 5 m irtifa, 15 sn hover ve 20×20 m kare rota üzerinde `MOT_HOVER_LEARN=0` ile koşulmuş ve `analyze.py` ile aşağıdaki gerçek metrikler elde edilmiştir:

| Metrik (`analyze.py`) | `empty` (12 kg) | `full` (28 kg, Ayarsız) | `full_tuned` (28 kg, Ayarlı) | Gözlem → Neden → Öneri |
|---|---:|---:|---:|---|
| **`takeoff_overshoot_m` (m)** | `-0.20` | `-0.01` | `-0.18` | Kalkışta 5 m hedefine aşım yapmadan (%95 eşiği) oturmuştur |
| **`hover_alt_std_m` (m)** | `0.060` | `0.075` | `0.104` | Dolu kütlede hover irtifa oynaması 6 cm'den 7.5–10 cm'ye çıkmıştır |
| **`hover_roll_pitch_std_deg` (°)** | **`0.22°`** | **`6.17°`** | **`5.08°`** | Atalet artışıyla açı salınımı 0.22°'den 6.17°'ye çıkmış; ivme/hover ayarıyla **%18 düşmüştür** (tam çözüm: Rate PID AutoTune) |
| **`hover_xy_std_m` (m)** | **`0.000`** | **`0.346`** | **`0.214`** | Hover yatay gezintisi `full_tuned` ile **34.6 cm'den 21.4 cm'ye (%38 iyileşme)** inmiştir |
| **`route_xy_err_rms_m` (m)** | `10.63` | `10.46` | `10.45` | Noktasal hedef (`sp_x, sp_y`) geçişlerinde 3 koşuda benzer RMS izleme görülmüştür |
| **`route_alt_err_max_m` (m)** | **`0.04 m`** | **`0.41 m`** | **`0.31 m`** | Köşe dönüşlerindeki irtifa çökmesi `full`'de 41 cm iken `full_tuned`'da **31 cm'ye (%24 iyileşme)** düşmüştür |
| **`route_max_tilt_deg` (°)** | `20.99°` | `26.09°` | `36.62°` | Ağır yükte köşe dönüşlerinde eğim artmıştır; sahada `ANGLE_MAX=2500` (25°) ile sınırlandırılmalıdır |
| **`pwm_mean_hover_route` (µs)** | **`1561.0`** | **`1633.9`** | **`1634.8`** | Dolu tankta ortalama motor PWM çıkışı **+73 µs** yükselmiştir |
| **`pwm_max` (µs)** | **`1665`** | **`1950`** | **`1950`** | Boşta 1665 µs'yi geçmeyen motorlar, doluda dönüşlerde **1950 µs** tavanına ulaşmıştır |
| **`pwm_imbalance_pct` (%)** | **`%0.52`** | **`%7.62`** | **`%7.85`** | CG ofseti (`+2.5 cm X, +1.5 cm Y, -14 cm Z`) motorlar arasında **~%7.7 sabit yük farkı** yaratmıştır |
| **`sat_pct_time_pwm>1900` (%)** | **`%0.00`** | **`%2.48`** | **`%3.07`** | Dolu senaryoda sürenin ~%2.5–3'ü >1900 µs doygunlukta geçmiştir; itki rezervi sınırındadır |
| **`landing_vz_mean_last2m_mps` (m/s)** | `0.287` | `0.270` | `0.271` | Son 2 m'de ortalama iniş hızı ~0.27–0.29 m/s, azami `0.50 m/s` (`LAND_SPEED=50`) olarak ölçülmüştür |
| **`vibe_max_xyz` (m/s²) / `clip0`** | `15.68` / `2` | `11.93` / `2` | `12.36` / `4` | Simülatörde titreşim düşüktür; gerçek titreşim ve FFT sahada ölçülür |
| **`fc_events` (Adet)** | `4` | `4` | `4` | EKF yaw hizalama, `PrecLand: Failsafe Measures` ve `Disarming motors` |

### Başarısız Denemeler ve Simülatör Sınırlamaları
1. **Deneme 1 (Yetersiz İtki - `Thr 100`):** Gazebo modeline 16 kg tank eklendiğinde motor/pervane kaldırma katsayısı artırılmadan araç `Thr 100` (%100 gaz) olmasına rağmen yerden kesilememiştir.
2. **Deneme 2 (Atalet-İtki Uyumsuzluğu ve Takla - `AngErr=170>30`):** Pervane kaldırma alanı büyütülüp gövde atalet tensörü (`ixx, iyy, izz`) küçük bırakıldığında, araç kalkış sonrası ilk manevrada aşırı açısal ivmeyle ters dönmüş ve `AP: Crash: Disarming: AngErr=170>30` ile motorları kapatmıştır. Kütleyle birlikte atalet tensörü de ölçeklenerek sorun çözülmüştür.
3. **Simülatör Sınırlamaları:** Gazebo'da sıvı çalkalanması (slosh) modellenmez (sabit rijit kütle + CG ofseti kullanılır), yer etkisi ve rüzgâr türbülansı basittir, gerçek yapısal titreşim/FFT sahada doğrulanır.

---

## 5. Sahaya Devreye Alma, Geri Dönüş Planı ve Opsiyonel Hassas İniş

### 5.1 Kademeli Devreye Alma ve Saha Güvenliği Prosedürü
1. **Parametre Yedeği:** `param save` ve Mission Planner ile orijinal parametreleri (`backup_original.parm`), firmware sürümünü ve log klasörünü kaydet. Her tune adımı öncesi yeni yedek al.
2. **Motor ve ESC Doğrulama (Pervanesiz):** Masa üzerinde Motor Test ile motor sırası, dönüş yönü, ESC kalibrasyonu, PWM min/max (`MOT_SPIN_ARM/MIN/MAX`), akım ve sıcaklık kontrolü yap.
3. **Filtreleme (Tethered Hover):** Pervane takılı, emniyet bağıyla 1–2 m hover logu al (`INS_LOG_BAT_MASK=1`). FFT ile motor gürültü pikini bul, `INS_HNTCH_*` (throttle veya ESC-RPM tabanlı) ayarla. Hedef: `VIBE < 30 m/s²`, `clipping = 0`.
4. **Düşük Riskli Testler (Boş Tank):** 1–2 m hover (`Stabilize` → `AltHold` → `Loiter` → kısa `Auto`). RTL, RC kaybı ve düşük akü failsafe davranışlarını doğrula.
5. **Kademeli Su Yükü (İlaç Yok):** Sadece **su** kullanarak `%25 → %50 → %75 → %100` yükle test et. Her kademede logdan hover gazı (`< 0.60`), motor dengesizliği (`< %5`), maksimum tilt ve titreşimi kontrol et; temiz değilse bir üst yüke geçme.
6. **Başarısız Testten Geri Dönüş (Rollback):** Salınım, motor doygunluğu (`PWM > 1900`) veya titreşim artışında derhal `LAND` / `RTL` ile in, motorları kapat, son güvenli `backup.parm` dosyasını geri yükle, su yükünü bir kademe düşür ve logu incelemeden devam etme.
7. **Yapılmayacaklar (Açık Güvenlik Şartı):** Test kolaylığı için dahi **failsafe'ler, geofence (`FENCE_ENABLE`), arming kontrolleri (`ARMING_CHECK`), düşük akü korumaları ve RC kaybı davranışları kesinlikle kapatılmaz, gevşetilmez veya geçersiz kılınmaz.**

### 5.2 Opsiyonel Ek: Görsel İşaretçi ile Hassas İniş (Precision Landing)
Gazebo sahnemizde yer alan `precision_landing_pad` (dolum/şarj pedi) üzerine bir **ArUco / AprilTag** yerleştirilerek ve aşağı bakan kamera / IR-LOCK sensörü kullanılarak ArduPilot'un `PLND_ENABLED=1`, `PLND_TYPE` mimarisiyle son iniş hizalanabilir. **Güvenlik şartı:** Hassas iniş yalnızca sensör sağlıklıyken aktif olur; görsel işaret kaybolursa (testlerimizdeki `PrecLand: Failsafe Measures` mesajında olduğu gibi) otopilot güvenlik kontrollerini devre dışı bırakmadan standart `LAND` davranışına döner. Dolu tankta iniş momentumu yüksek olduğundan yatay düzeltme hızı sınırlandırılmalı ve yatay hizalama tamamlanmadan son 2 m'de dikey hız artırılmamalıdır.
