# Akıllı Acil Durum İstasyonu

Atölyeler için düşme, gaz ve duman uyarı sistemi. Raspberry Pi 3 kamerayla düşmeyi algılar. Arduino gaz ve duman sensörlerini okur, buzzer ve RGB ışıkla uyarır. Bildirimler MQTT ile mobil uygulamaya gider.

```
Kamera ──> Raspberry Pi (PythonApplication1.py) ──USB seri──> Arduino (sensörler, buzzer, RGB)
                    │
                    └── MQTT (Mosquitto, Pi'nin üzerinde) ──Wi-Fi──> Mobil uygulama
```

## Raspberry Pi kurulumu

```bash
sudo apt install mosquitto mosquitto-clients python3-picamera2
pip install mediapipe opencv-python numpy paho-mqtt pyserial
```

Mosquitto'nun aynı ağdaki telefonlardan bağlantı kabul etmesi için `/etc/mosquitto/conf.d/atolye.conf` dosyasına şunu yazın, sonra `sudo systemctl restart mosquitto` komutunu çalıştırın:

```
listener 1883
allow_anonymous true
```

> Not: Bu ayar okul ağı içinde deneme içindir. Kalıcı kullanımda kullanıcı adı ve şifre eklenmeli.

Çalıştırma: `python3 PythonApplication1.py`

İlk açılışta **5 dakika kalibrasyon** yapılır. Bu sürede oda boş bırakılır ve atölyenin boş hali `arka_plan.png` olarak kaydedilir.

Kalibrasyondan sonra ortam resmi **canlı güncellenir**, ama yalnızca güvenli piksellerde:

- İnsan görülen bölge (iskelet ve çevresindeki pay) kilitlenir, hiç güncellenmez. Kilit, o bölge tekrar eski haline dönünce (kişi gidince) açılır. Model kişiyi sonradan kaybetse bile (yarım görünme, hareketsiz yatma) bölge kilitli kalır.
- Bir değişiklik (taşınan dolap, ışık) ancak 60 sn (`ARKA_SABIT_SN`) hiç kıpırdamadan durursa ortama işlenir. Yürüyen biri hiç işlenmez.
- Güncel ortam 10 dakikada bir `arka_plan.png` dosyasına yazılır. Pi 3'ü yormamak için bu güncelleme saniyede yalnızca bir kez yapılır.

İstenirse yeniden kalibrasyon şu üç yoldan biriyle başlatılır (örneğin kilitli kalan bir bölgede eşya yer değiştirdiyse):

- Pencerede `k` tuşuna basmak
- Program klasörüne `arka_plan_kaydet` adlı boş bir dosya koymak
- MQTT komutu göndermek (aşağıda)

## Düşme algılama

- **Görünür düşme:** Gövde yatay durumdaysa ve kişi 5 sn kıpırdamazsa `yerde_hareketsiz` alarmı verilir.
- **Görüş dışına düşme (masa ya da tezgah arkası):** Kişi ani bir düşüş yaptıktan ya da yattıktan sonra görüş alanından kaybolursa veya yalnızca başı ve omuzları görünüp duruşu ölçülemezse 10 sn (`GIZLI_BEKLEME_SN`) beklenir. Bu sürede ayakta görünmezse `gorus_disi_dusme` alarmı verilir. Çerçevenin kenarından çıkan kişi için alarm verilmez, odadan çıkmış sayılır. Masada dik oturan kişi için de alarm verilmez (baş omuzların üstündeyse).
- **Acil durum butonu (Arduino):** Kamera kişiyi hiç göremese bile butona basılarak yardım çağrılabilir.

Eşikler (`ANI_DUSUS_ORANI`, `GIZLI_BEKLEME_SN`, `UST_DIK_ACISI`) atölyede yapılacak denemelerle ayarlanmalıdır.

## MQTT konuları

`<istasyon>` yerine `ISTASYON_ADI` yazılır (varsayılan değeri `atolye-1`). Bütün mesajlar JSON biçimindedir.

| Konu | Yön | İçerik |
| --- | --- | --- |
| `atolye/<istasyon>/dusme` | Pi → uygulama | `yerde_hareketsiz` ve `gorus_disi_dusme` (iskelet resmi `iskelet_jpg_base64` alanında), `kisi_kalkti`, `acil_buton`, `acil_buton_iptal`, `alarm_kapatildi` |
| `atolye/<istasyon>/gaz` | Pi → uygulama | `gaz_alarmi`, `duman_alarmi` (`deger` ile birlikte), `gaz_temiz`, `duman_temiz` |
| `atolye/<istasyon>/olcum` | Pi → uygulama | `{"olay":"olcum","gaz":412,"duman":300}`, 5 saniyede bir (son değer broker'da saklanır) |
| `atolye/<istasyon>/durum` | Pi → uygulama | `cevrimici` / `cevrimdisi` (saklanır), `kalibrasyon_basladi`, `arka_plan_kaydedildi`, `arduino_hazir`, `arduino_koptu`, `sensor_isiniyor` |
| `atolye/<istasyon>/komut` | uygulama → Pi | `arka_plan_kaydet` (ortamı yeniden kaydet), `sustur` (buzzer'ı 5 dk sustur), `alarm_kapat` (yardım geldi: düşme ve acil buton alarmlarını kapat). Düz metin ya da `{"komut":"..."}` |

Telefon olmadan deneme:

```bash
mosquitto_sub -h <pi-ip> -t 'atolye/#' -v
mosquitto_pub -h <pi-ip> -t atolye/atolye-1/komut -m sustur
```

## Arduino

Kod `arduino/acil_durum_istasyonu/acil_durum_istasyonu.ino` dosyasında. Arduino IDE ile Uno ya da Nano'ya yüklenir. USB kablosuyla Pi'ye takılır. Port otomatik bulunur, gerekirse `ARDUINO_PORT` ayarından elle girilir.

| Parça | Arduino pini |
| --- | --- |
| Gaz sensörü (örn. MQ-5/MQ-6) analog çıkışı (AO) | A0 |
| Duman sensörü (örn. MQ-2) analog çıkışı (AO) | A1 |
| Buzzer (+) | D8 |
| RGB LED kırmızı / yeşil / mavi (220 Ω dirençle) | D9 / D10 / D11 |
| Acil durum butonu (bir ucu D2'ye, diğer ucu GND'ye) | D2 |
| Sensör VCC / GND | 5V / GND |

Ayarlar dosyanın başında bulunur: eşik değerleri (`GAZ_ESIK`, `DUMAN_ESIK`), ortak anotlu LED ve pasif buzzer seçimi. MQ sensörleri ilk 60 saniye ısınır, bu sürede alarm verilmez ve ışık mavi yanar. Eşikler atölyede `OLCUM` değerlerine bakılarak ayarlanmalıdır.

Işık ve ses:

| Durum | Işık | Buzzer |
| --- | --- | --- |
| Acil buton | kırmızı, hızlı yanıp söner | hızlı bip |
| Duman | mor, hızlı yanıp söner | hızlı bip |
| Gaz | turuncu, hızlı yanıp söner | hızlı bip |
| Düşme (Pi'den `DUSME`) | kırmızı, yavaş yanıp söner | kesik bip |
| Pi'den 30 sn haber yok | mavi, yanıp söner | sessiz |
| Sensörler ısınıyor | mavi | sessiz |
| Normal | yeşil | sessiz |

Seri protokol (9600 baud, her mesaj bir satır):

- Pi → Arduino: `DUSME`, `TAMAM`, `SUSTUR`, `SIFIRLA`, `PING` (10 sn'de bir)
- Arduino → Pi: `HAZIR`, `ISINIYOR kalan=45`, `OLCUM gaz=412 duman=300`, `ALARM gaz=612`, `ALARM duman=700`, `TEMIZ gaz`, `TEMIZ duman`, `SUSTURULDU`, `ACIL` (butona kısa basış), `ACIL_IPTAL` (3 sn basılı tutma)

Arduino, Pi'den bağımsız olarak da çalışır: gaz ya da duman algılarsa Pi kapalı olsa bile buzzer ve ışıkla uyarır.
