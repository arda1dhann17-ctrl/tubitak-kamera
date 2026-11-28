#!/usr/bin/env python3

# =====================================================================
#  RASPBERRY PI KODU - Kamera ile dusme algilama
#  Akilli Acil Durum Istasyonu (TUBITAK 4006-A)
#
#  Ne yapar: Kameradan iskelet cikarir (MediaPipe), dusmeyi ve gorus
#  disina dusmeyi algilar, alarm resmini (atolye + iskelet) MQTT ile
#  uygulamaya yollar, Arduino'ya seri porttan haber verir.
#  Nerede calisir: Raspberry Pi 3 (ya da deneme icin bilgisayar).
#  Calistirma: python3 kamera_dusme_algilama.py
#  Arduino tarafi: arduino/sensor_ve_alarm/sensor_ve_alarm.ino
# =====================================================================

import base64
import json
import math
import os
import sys
import threading
import time
import urllib.request
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

def _raspberry_pi_mi():
    try:
        with open("/proc/device-tree/model") as f:
            return "raspberry pi" in f.read().lower()
    except OSError:
        return False


PI_MI = _raspberry_pi_mi()

ISTASYON_ADI = "atolye-1"
KAMERA_KAYNAGI = "otomatik"
KAMERA_NO = 0
GENISLIK, YUKSEKLIK = 320, 240
HEDEF_FPS = 5 if PI_MI else 15   # Pi 3 yavas; bilgisayarda daha akici
GORUNTU_GOSTER = True
EN_FAZLA_KISI = 5
ESLESME_MESAFESI = 1.0
# Pi 3'te hafif "lite" model, bilgisayarda daha isabetli "full" model.
MODEL_TURU = "lite" if PI_MI else "full"
MODEL_DOSYASI = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), f"pose_landmarker_{MODEL_TURU}.task")
MODEL_ADRESI = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
                f"pose_landmarker_{MODEL_TURU}/float16/1/pose_landmarker_{MODEL_TURU}.task")
MIN_GORUNURLUK = 0.5
YATIS_ACISI = 50
YATIS_EN_BOY = 1.0
YATIS_DIK_ORANI = 0.6      # bas, kalcanin (govde boyu x bu oran)dan fazla yukaridaysa yatmiyor
YATIS_PENCERESI_SN = 1.5   # yatma karari son 1.5 sn'deki karelere bakilarak verilir
YATIS_KARAR_ORANI = 0.7    # karelerin %70'i "yatiyor" derse yatiyor, %30'u derse kalkti
YATIS_MIN_ORNEK = 3
HAREKET_PENCERESI = 1.0
HAREKET_ESIGI = 0.04
BEKLEME_SANIYESI = 5
IPTAL_TEYIT_SN = 1.0
KAYIP_TOLERANS_SN = 2.0
ALARM_KAYIP_TOLERANS_SN = 300.0  # alarmdaki kisi gorunmese de alarm 5 dk surer
# --- Gorus disina dusme (masa/tezgah arkasi) ---
# Kisi ani bir dusus yapip ya da yatarken gorus alanindan kaybolursa (veya sadece
# ust govdesi gorunup durusu olculemezse) ve GIZLI_BEKLEME_SN boyunca tekrar
# ayakta gorunmezse "gorus_disi_dusme" alarmi verilir. Cerceve kenarindan
# cikan kisi icin alarm verilmez (odadan cikmistir).
ANI_DUSUS_SN = 1.0         # bu sure icinde...
ANI_DUSUS_ORANI = 0.5      # ...ust govde, boyunun bu orani kadar asagi inerse "ani dusus"
GIZLI_PENCERE_SN = 3.0     # ani dusus ile kaybolma arasindaki en fazla sure
GIZLI_BEKLEME_SN = 10.0    # bu kadar gorunmezse alarm
GENIS_ESLESME = 2.5        # supheli kisi kaybolunca yakininda gorunen kisi ayni sayilir
UST_DIK_ACISI = 40         # bas-omuz cizgisi dikeye bu acidan yakinsa "dik oturuyor"
KENAR_PAYI = 0.08          # cerceve kenarina bu oranda yakin kaybolan kisi cikmis sayilir
KENAR_TEMAS_PAYI = 0.03    # vucudu cerceve kenarina degen kisi (yarisi disarida) icin de
# GORUNTU: Ekranda ve alarm resminde CANLI kamera goruntusu kullanilir (koltuk
# oynarsa oda da aninda oynar), ama insanlarin bulundugu bolgeler her karede
# silinir: oraya ogrenilen bos ortam (bilinmiyorsa gri) konur, ustune iskelet
# cizilir. Boylece kimsenin yuzu/vucudu gorunmez. Video kaydedilmez.
# Silinen bolge: o karedeki insanlar + son GIZLEME_HAFIZA_SN icinde gorulenler
# (model bir kare kacirsa bile kisi gorunmesin) + dusmus/supheli kisinin son
# goruldugu yer (masa arkasinda yarim kalsa bile gorunmesin).
CANLI_GORUNTU = True          # False: kamera hic kullanilmaz, sadece ogrenilen ortam
GIZLEME_HAFIZA_SN = 2.0
# Arka plan (atolye ortami): insanlarin arkasini doldurmak icin canli ogrenilir.
# Insanlar ortam resmine HIC islenmez.
#  - Insan goruldugu bolge (iskelet + guvenlik payi) "kilitlenir". Kilit, o piksel
#    eski ortam rengine donunce (kisi gidince) acilir. Model kisiyi sonradan
#    kaybetse de (yarim gorunme, hareketsiz yatma) o bolge karismaz.
#  - Henuz bilinmeyen yerler ARKA_ILK_SABIT_SN kipirdamadan durunca ogrenilir.
#  - Bilinen yerdeki bir degisiklik (tasinan dolap, isik) ARKA_SABIT_SN boyunca
#    hic kipirdamazsa ortama islenir. Yuruyen biri hic islenmez.
# Ortam resmi sifirlanip bastan ogrenilebilir: pencerede "k" tusu, klasore
# "arka_plan_kaydet" adli bos dosya koymak veya MQTT "arka_plan_kaydet".
ISKELET_RESMI_EKLE = True
ISKELET_RESMI_OLCEK = 2
ISKELET_RESMI_KALITE = 70
ISKELET_ZEMIN_RENGI = (60, 60, 60)  # ortami henuz bilinmeyen yerler
ARKA_PLAN_KULLAN = True
ARKA_PLAN_DOSYASI = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "arka_plan.png")
ARKA_SIFIRLA_TETIK_DOSYASI = "arka_plan_kaydet"
ARKA_CANLI_GUNCELLE = True
ARKA_ILK_SABIT_SN = 3        # bilinmeyen yer bu kadar sabitse hemen ogrenilir
ARKA_SABIT_SN = 15           # bilinen yerdeki degisiklik bu kadar sabit kalirsa islenir
ARKA_KILITLI_BOS_SN = 30     # insan gittikten sonra bos kalan bilinmeyen yer
ARKA_HAREKET_ESIGI = 18      # kareler arasi renk farki (0-255) bundan buyukse "hareket"
ARKA_KILIT_ACMA_ESIGI = 20   # piksel eski ortama bu kadar yakinsa kilit acilir
ARKA_OGRENME_HIZI = 0.3      # sabit bolgeler ortama ne hizla islensin (saniyede bir)
ARKA_INSAN_PAYI = 0.25       # insan bolgesine eklenen pay (vucut boyunun orani)
ARKA_KAYDET_SN = 600         # canli guncellenen ortam 10 dk'da bir diske yazilir
ARKA_GUNCELLEME_SN = 1.0     # canli guncelleme saniyede 1 kez (Pi 3'u yormamak icin)

# --- MQTT: broker (Mosquitto) Pi'nin kendisinde calisir, uygulama ona baglanir ---
#   atolye/<istasyon>/dusme  : yerde_hareketsiz (iskelet resmiyle), kisi_kalkti
#   atolye/<istasyon>/gaz    : gaz_alarmi, duman_alarmi, gaz_temiz, duman_temiz
#   atolye/<istasyon>/olcum  : sensor degerleri (son deger saklanir / retain)
#   atolye/<istasyon>/durum  : cevrimici/cevrimdisi (retain), arka_plan_sifirlandi, arduino
#   atolye/<istasyon>/komut  : uygulamadan gelen komutlar: arka_plan_kaydet, sustur
# paho-mqtt kurulu degilse MQTT kapali kalir, program yine calisir.
MQTT_KULLAN = True
MQTT_SUNUCU, MQTT_PORT = "localhost", 1883
MQTT_KONU_KOKU = f"atolye/{ISTASYON_ADI}"

# --- Arduino (USB seri): gaz/duman sensorleri, buzzer, RGB isik ---
# pyserial kurulu degilse veya Arduino takili degilse bu kisim kapali kalir.
ARDUINO_KULLAN = True
ARDUINO_PORT = "otomatik"   # ya da "/dev/ttyACM0", Windows'ta "COM3" gibi
ARDUINO_HIZ = 9600
ARDUINO_PING_SN = 10        # Arduino, Pi'den 30 sn haber alamazsa mavi yanip soner
ISKELET_DOSYAYA_KAYDET = False
TAKIP_MIN_GORUNURLUK = 0.1
FPS_RAPOR_SN = 10


OLAY_KONULARI = {
    "yerde_hareketsiz": "dusme", "kisi_kalkti": "dusme", "gorus_disi_dusme": "dusme",
    "acil_buton": "dusme", "acil_buton_iptal": "dusme", "alarm_kapatildi": "dusme",
    "gaz_alarmi": "gaz", "duman_alarmi": "gaz", "gaz_temiz": "gaz", "duman_temiz": "gaz",
    "olcum": "olcum",
}
MQTT = None  # main() icinde MqttBaglanti ile doldurulur


def veri_gonder(veri):
    # veri: sozluk. Iskelet resmi "iskelet_jpg_base64" anahtarinda (base64 JPEG).
    alt_konu = OLAY_KONULARI.get(veri.get("olay"), "durum")
    if alt_konu != "olcum":
        gosterilecek = dict(veri)
        if gosterilecek.get("iskelet_jpg_base64"):
            kb = len(gosterilecek["iskelet_jpg_base64"]) * 3 // 4 / 1024
            gosterilecek["iskelet_jpg_base64"] = f"<JPEG iskelet resmi, yaklasik {kb:.1f} KB>"
        print(f"[VERI] {alt_konu} ->", json.dumps(gosterilecek, ensure_ascii=False))
    if MQTT is not None:
        MQTT.yayinla(alt_konu, veri, saklansin=(alt_konu == "olcum"))


def guvenli_gonder(veri):
    try:
        veri_gonder(veri)
    except Exception as hata:
        print("[HATA] Veri gonderilemedi:", hata)


KLASOR = os.path.dirname(os.path.abspath(__file__))
PENCERE_ADI = "Iskelet (video yok)"
KANIT_PENCERESI = "Alarm iskelet resmi"
FONT = cv2.FONT_HERSHEY_SIMPLEX
BURUN = 0
SOL_OMUZ, SAG_OMUZ = 11, 12
SOL_KALCA, SAG_KALCA = 23, 24
HAREKET_NOKTALARI = (0, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28)
BAGLANTILAR = (
    (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8), (9, 10),
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
    (15, 17), (15, 19), (15, 21), (17, 19), (16, 18), (16, 20), (16, 22), (18, 20),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (25, 27), (24, 26), (26, 28),
    (27, 29), (29, 31), (27, 31), (28, 30), (30, 32), (28, 32),
)


def model_hazirla():
    if os.path.exists(MODEL_DOSYASI):
        return
    print(f"[MODEL] {os.path.basename(MODEL_DOSYASI)} bulunamadi, indiriliyor (tek seferlik)...")
    gecici = MODEL_DOSYASI + ".indiriliyor"
    try:
        urllib.request.urlretrieve(MODEL_ADRESI, gecici)
        os.replace(gecici, MODEL_DOSYASI)
        print("[MODEL] Indirildi:", MODEL_DOSYASI)
    except Exception as hata:
        if os.path.exists(gecici):
            os.remove(gecici)
        print("[HATA] Model indirilemedi:", hata)
        print("       Elle indir ve bu dosyanin yanina koy:")
        print("       " + MODEL_ADRESI)
        sys.exit(1)


class Kamera:

    def __init__(self):
        self.pi = None
        self.cap = None

        if KAMERA_KAYNAGI in ("otomatik", "picamera2"):
            try:
                from picamera2 import Picamera2
                self.pi = Picamera2()
                ayar = self.pi.create_preview_configuration(
                    main={"size": (GENISLIK, YUKSEKLIK), "format": "RGB888"})
                self.pi.configure(ayar)
                self.pi.start()
                print("[KAMERA] Pi kamerasi (picamera2) acildi.")
                return
            except Exception as hata:
                if self.pi is not None:
                    try:
                        self.pi.close()
                    except Exception:
                        pass
                self.pi = None
                if KAMERA_KAYNAGI == "picamera2":
                    raise RuntimeError(f"Pi kamerasi acilamadi: {hata}")
                print(f"[KAMERA] picamera2 kullanilamadi ({hata}); OpenCV deneniyor.")

        self.cap = cv2.VideoCapture(KAMERA_NO)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, GENISLIK)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, YUKSEKLIK)
        if not self.cap.isOpened():
            raise RuntimeError(
                "Kamera acilamadi. Kamera takili mi? KAMERA_NO dogru mu? "
                "(Pi kamera modulu icin: sudo apt install python3-picamera2)")
        print(f"[KAMERA] OpenCV kamerasi (no={KAMERA_NO}) acildi.")

    def oku(self):
        if self.pi is not None:
            return self.pi.capture_array().copy()
        basarili, kare = self.cap.read()
        if not basarili:
            return None
        if kare.shape[1] != GENISLIK or kare.shape[0] != YUKSEKLIK:
            kare = cv2.resize(kare, (GENISLIK, YUKSEKLIK))
        return kare

    def kapat(self):
        if self.pi is not None:
            self.pi.stop()
            self.pi.close()
        if self.cap is not None:
            self.cap.release()


def ortalama_nokta(noktalar, numaralar):
    secilen = [noktalar[i] for i in numaralar if i in noktalar]
    if not secilen:
        return None
    return (sum(p[0] for p in secilen) / len(secilen),
            sum(p[1] for p in secilen) / len(secilen))


def durus_olc(noktalar):
    omuz = ortalama_nokta(noktalar, (SOL_OMUZ, SAG_OMUZ))
    kalca = ortalama_nokta(noktalar, (SOL_KALCA, SAG_KALCA))
    if omuz is None or kalca is None:
        return None

    dx = kalca[0] - omuz[0]
    dy = kalca[1] - omuz[1]
    aci = math.degrees(math.atan2(abs(dx), abs(dy)))

    xler = [p[0] for p in noktalar.values()]
    yler = [p[1] for p in noktalar.values()]
    genislik = max(xler) - min(xler)
    yukseklik = max(yler) - min(yler)

    en_boy = genislik / max(yukseklik, 1.0)
    boyut = max(genislik, yukseklik, 1.0)
    # Bas kalcadan ne kadar yukarida? (govde boyuna oranla) Ayakta ~1.3, yatarken ~0.
    bas = noktalar.get(BURUN, omuz)
    dik_oran = (kalca[1] - bas[1]) / max(math.dist(omuz, kalca), 1.0)
    return aci, en_boy, boyut, dik_oran


def kenara_degiyor(noktalar):
    """Vucudun bir kismi cerceve disinda mi? (kisi kadrajdan cikiyor/cikmis)"""
    if not noktalar:
        return False
    xler = [p[0] for p in noktalar.values()]
    yler = [p[1] for p in noktalar.values()]
    px, py = KENAR_TEMAS_PAYI * GENISLIK, KENAR_TEMAS_PAYI * YUKSEKLIK
    return (min(xler) < px or max(xler) > GENISLIK - px
            or min(yler) < py or max(yler) > YUKSEKLIK - py)


def ust_govde_dik(noktalar):
    """Bas omuzlarin ustunde ve govde dik mi? (kalca gorunmese de)"""
    omuz = ortalama_nokta(noktalar, (SOL_OMUZ, SAG_OMUZ))
    if omuz is None or BURUN not in noktalar:
        return False
    bx, by = noktalar[BURUN]
    yukari = omuz[1] - by
    if yukari <= 0:
        return False
    return math.degrees(math.atan2(abs(bx - omuz[0]), yukari)) < UST_DIK_ACISI


class HareketOlcer:

    def __init__(self, pencere_sn):
        self.pencere = pencere_sn
        self.gecmis = deque()

    def olc(self, t, noktalar, boyut):
        secili = {i: noktalar[i] for i in HAREKET_NOKTALARI if i in noktalar}
        self.gecmis.append((t, secili))

        while len(self.gecmis) > 2 and t - self.gecmis[1][0] >= self.pencere:
            self.gecmis.popleft()

        t_eski, eski = self.gecmis[0]
        if len(self.gecmis) < 2 or t - t_eski < self.pencere * 0.5:
            return None

        ortak = [i for i in secili if i in eski]
        if len(ortak) < 4:
            return None

        farklar = sorted(math.dist(secili[i], eski[i]) for i in ortak)
        medyan = farklar[len(farklar) // 2]
        return medyan / boyut


class Kisi:

    def __init__(self, nokta, tum):
        self.nokta = nokta
        self.tum = tum
        kaynak = nokta or tum
        xler = [p[0] for p in kaynak.values()]
        yler = [p[1] for p in kaynak.values()]
        self.merkez = (sum(xler) / len(xler), sum(yler) / len(yler))
        self.boyut = max(max(xler) - min(xler), max(yler) - min(yler), 1.0)


def kisileri_al(sonuc, genislik, yukseklik):
    kisiler = []
    alt_sinir = min(MIN_GORUNURLUK, TAKIP_MIN_GORUNURLUK)
    for poz in (sonuc.pose_landmarks or []):
        nokta, tum = {}, {}
        for no, n in enumerate(poz):
            gorunurluk = 1.0 if n.visibility is None else n.visibility
            x = min(max(n.x * genislik, -genislik), 2 * genislik)
            y = min(max(n.y * yukseklik, -yukseklik), 2 * yukseklik)
            if gorunurluk >= MIN_GORUNURLUK:
                nokta[no] = (x, y)
            if gorunurluk >= alt_sinir:
                tum[no] = (x, y)
        if nokta or tum:
            kisiler.append(Kisi(nokta, tum))
    return kisiler


def bos_bilgi():
    return {"aci": None, "en_boy": None, "hareket": None, "yatiyor": None, "gecen": None}


class DusmeTakipcisi:

    def __init__(self, kimlik=0):
        self.kimlik = kimlik
        self.olcer = HareketOlcer(HAREKET_PENCERESI)
        self.durum = "NORMAL"
        self.sayac_basi = None
        self.iptal_basi = None
        self.son_gorulen = None
        self.merkez = None
        self.boyut = None
        self.kisi = None
        self.goruntu = None
        self.bilgi = bos_bilgi()
        self.yazilan_durum = "NORMAL"
        self.yatis_gecmis = deque()  # (t, o karede yatiyor mu)
        self.ust_gecmis = deque()    # (t, ust govde y, boyut)
        self.son_ani_dusus = None
        self.son_yatiyor = False
        self.kenarda = False
        self.gizli_basi = None
        self.alarm_turu = None
        self.son_kisi = None
        self.son_kisi_t = None

    @property
    def supheli(self):
        return self.durum != "NORMAL" or self.gizli_basi is not None or self.son_yatiyor

    def sifirla(self):
        self.durum = "NORMAL"
        self.sayac_basi = None
        self.iptal_basi = None
        self.gizli_basi = None
        self.alarm_turu = None

    def kayip_mi(self, t):
        if self.durum == "ALARM":
            tolerans = ALARM_KAYIP_TOLERANS_SN
        elif self.gizli_basi is not None:
            tolerans = GIZLI_BEKLEME_SN + 5.0
        else:
            tolerans = KAYIP_TOLERANS_SN
        return self.son_gorulen is None or (t - self.son_gorulen) > tolerans

    def olay_olustur(self, tur, t):
        return {
            "istasyon": ISTASYON_ADI,
            "olay": tur,
            "kisi_no": self.kimlik,
            "zaman": datetime.now().isoformat(timespec="seconds"),
            "yerde_kalma_sn": round(t - (self.sayac_basi or t), 1),
        }

    def _yatis_karari(self, t, anlik):
        """Tek karelik titremeler karari bozmasin: son birkac kareye bakilir.
        True = yatiyor, False = kalkti/ayakta, None = henuz belli degil."""
        self.yatis_gecmis.append((t, anlik))
        while self.yatis_gecmis and t - self.yatis_gecmis[0][0] > YATIS_PENCERESI_SN:
            self.yatis_gecmis.popleft()
        if len(self.yatis_gecmis) < YATIS_MIN_ORNEK:
            return None
        oran = sum(1 for _, y in self.yatis_gecmis if y) / len(self.yatis_gecmis)
        if oran >= YATIS_KARAR_ORANI:
            return True
        if oran <= 1.0 - YATIS_KARAR_ORANI:
            return False
        return None

    def _dusus_izle(self, t, noktalar, boyut):
        ust = ortalama_nokta(noktalar, (SOL_OMUZ, SAG_OMUZ))
        if ust is None and BURUN in noktalar:
            ust = noktalar[BURUN]
        if ust is None:
            return
        onceki_y = self.ust_gecmis[-1][1] if self.ust_gecmis else None
        self.ust_gecmis.append((t, ust[1], boyut))
        while self.ust_gecmis and t - self.ust_gecmis[0][0] > ANI_DUSUS_SN:
            self.ust_gecmis.popleft()
        if len(self.ust_gecmis) < 3 or onceki_y is None:
            return
        en_ust = min(y for _, y, _ in self.ust_gecmis)
        ref = max(b for _, _, b in self.ust_gecmis)
        # Tek karelik sicrama degil: bir onceki kareden de asagi inmis olmali
        # (titreyen nokta bir asagi bir yukari gider, dusen kisi hep asagi).
        if ((ust[1] - en_ust) / max(ref, 1.0) >= ANI_DUSUS_ORANI
                and onceki_y - en_ust >= 0.25 * ref):
            if self.son_ani_dusus is None or t - self.son_ani_dusus > ANI_DUSUS_SN:
                print(f"[DUSUS] #{self.kimlik}: ani dusus algilandi")
            self.son_ani_dusus = t

    def _merkez_kenarda(self):
        if self.merkez is None:
            return False
        x, y = self.merkez
        px, py = KENAR_PAYI * GENISLIK, KENAR_PAYI * YUKSEKLIK
        return x < px or x > GENISLIK - px or y < py or y > YUKSEKLIK - py

    def _kalkti(self, t, dusus_unut=False):
        """Kisi kalkmis gorunuyor: sayac iptal, alarm ise teyitten sonra kapanir."""
        self.son_yatiyor = False
        if dusus_unut:
            self.son_ani_dusus = None
        if self.gizli_basi is not None and self.durum != "ALARM":
            print(f"[DUSUS] #{self.kimlik}: ayakta goruldu, bekleme iptal")
            self.gizli_basi = None
        if self.durum == "SAYIYOR":
            print(f"[DUSUS] #{self.kimlik}: kalkti, sayac iptal")
            self.sifirla()
        elif self.durum == "ALARM":
            if self.iptal_basi is None:
                self.iptal_basi = t
            elif t - self.iptal_basi >= IPTAL_TEYIT_SN:
                olay = self.olay_olustur("kisi_kalkti", t)
                self.sifirla()
                return olay
        return None

    def _olculemiyor(self, t, noktalar=None):
        """Kisi gorunmuyor ya da durusu olculemiyor (or. masa arkasinda)."""
        if noktalar is not None and ust_govde_dik(noktalar):
            return self._kalkti(t, dusus_unut=True)  # sadece ust govde gorunuyor ama dik
        if self.durum == "ALARM":
            return None
        if self.durum == "SAYIYOR":
            # Yatarken durusu olculemez oldu: sayac durur, gorus disi beklemeye gecer.
            self.durum = "NORMAL"
            self.sayac_basi = None
            self.iptal_basi = None
            self.son_yatiyor = True
        if self.gizli_basi is None:
            yeni_dusus = (self.son_ani_dusus is not None
                          and t - self.son_ani_dusus <= GIZLI_PENCERE_SN)
            if ((yeni_dusus or self.son_yatiyor)
                    and not self.kenarda and not self._merkez_kenarda()):
                self.gizli_basi = t
                print(f"[DUSUS] #{self.kimlik}: gorus disinda kaldi, "
                      f"{GIZLI_BEKLEME_SN:.0f} sn bekleniyor")
            return None
        if t - self.gizli_basi >= GIZLI_BEKLEME_SN:
            self.durum = "ALARM"
            self.alarm_turu = "gorus_disi_dusme"
            self.sayac_basi = self.gizli_basi
            self.iptal_basi = None
            return self.olay_olustur("gorus_disi_dusme", t)
        return None

    def guncelle(self, t, noktalar):
        bilgi = bos_bilgi()
        olay = None

        if noktalar is None:
            if self.kayip_mi(t):
                self.sifirla()
                return self.durum, olay, bilgi
            olay = self._olculemiyor(t)
            return self.durum, olay, bilgi
        self.son_gorulen = t
        self.kenarda = kenara_degiyor(noktalar)
        if self.boyut is not None:
            self._dusus_izle(t, noktalar, self.boyut)

        olcum = durus_olc(noktalar)
        if olcum is None:
            olay = self._olculemiyor(t, noktalar)
            return self.durum, olay, bilgi
        aci, en_boy, boyut, dik_oran = olcum

        anlik = (aci >= YATIS_ACISI and en_boy >= YATIS_EN_BOY
                 and dik_oran < YATIS_DIK_ORANI)
        karar = self._yatis_karari(t, anlik)
        hareket = self.olcer.olc(t, noktalar, boyut)
        hareketsiz = (hareket is None) or (hareket < HAREKET_ESIGI)
        bilgi.update(aci=aci, en_boy=en_boy, hareket=hareket, yatiyor=(karar is True))

        if karar is False:
            olay = self._kalkti(t)
        elif karar is True:
            self.son_yatiyor = True
            if self.gizli_basi is not None and self.durum != "ALARM":
                self.gizli_basi = None  # tekrar gorundu; asagida normal sayac isler
            if self.durum == "NORMAL":
                if hareketsiz:
                    self.durum = "SAYIYOR"
                    self.sayac_basi = t
                    self.iptal_basi = None
            elif self.durum == "SAYIYOR":
                if hareketsiz:
                    self.iptal_basi = None
                    if t - self.sayac_basi >= BEKLEME_SANIYESI:
                        self.durum = "ALARM"
                        self.alarm_turu = "yerde_hareketsiz"
                        olay = self.olay_olustur("yerde_hareketsiz", t)
                else:
                    # Yerde hareket ediyor (bilinci yerinde): teyitten sonra sayac sifirlanir.
                    if self.iptal_basi is None:
                        self.iptal_basi = t
                    elif t - self.iptal_basi >= IPTAL_TEYIT_SN:
                        self.sifirla()
            elif self.durum == "ALARM":
                self.iptal_basi = None

        if self.durum == "SAYIYOR":
            bilgi["gecen"] = t - self.sayac_basi
        return self.durum, olay, bilgi


class Izleyici:

    def __init__(self):
        self.izler = {}
        self.son_kimlik = 0

    def eslestir(self, kisiler):
        adaylar = []
        for kimlik, iz in self.izler.items():
            if iz.merkez is None:
                continue
            for sira, kisi in enumerate(kisiler):
                uzaklik = math.dist(iz.merkez, kisi.merkez)
                sinir = ESLESME_MESAFESI * max(iz.boyut, kisi.boyut)
                if uzaklik <= sinir:
                    adaylar.append((uzaklik, kimlik, sira))
        adaylar.sort()

        eslesme, kullanilan = {}, set()
        for uzaklik, kimlik, sira in adaylar:
            if kimlik in eslesme or sira in kullanilan:
                continue
            eslesme[kimlik] = sira
            kullanilan.add(sira)

        # Dusmus/supheli bir kisi kalkarken iskeleti cok degisir; yakinda yeni
        # biri belirirse ayni kisi say (yoksa eski kayit "gorus disi" alarm verir).
        genis = []
        for kimlik, iz in self.izler.items():
            if kimlik in eslesme or iz.merkez is None or not iz.supheli:
                continue
            for sira, kisi in enumerate(kisiler):
                if sira in kullanilan:
                    continue
                uzaklik = math.dist(iz.merkez, kisi.merkez)
                if uzaklik <= GENIS_ESLESME * max(iz.boyut, kisi.boyut):
                    genis.append((uzaklik, kimlik, sira))
        for uzaklik, kimlik, sira in sorted(genis):
            if kimlik in eslesme or sira in kullanilan:
                continue
            eslesme[kimlik] = sira
            kullanilan.add(sira)
        return eslesme

    def guncelle(self, t, kisiler):
        eslesme = self.eslestir(kisiler)
        eslesenler = set(eslesme.values())

        for sira in range(len(kisiler)):
            if sira not in eslesenler:
                self.son_kimlik += 1
                iz = DusmeTakipcisi(self.son_kimlik)
                self.izler[iz.kimlik] = iz
                eslesme[iz.kimlik] = sira

        olaylar = []
        for kimlik, iz in list(self.izler.items()):
            sira = eslesme.get(kimlik)
            if sira is None:
                iz.kisi = None
                _, olay, iz.bilgi = iz.guncelle(t, None)
            else:
                kisi = kisiler[sira]
                iz.kisi = kisi
                iz.son_kisi, iz.son_kisi_t = kisi, t
                iz.merkez, iz.boyut = kisi.merkez, kisi.boyut
                iz.goruntu = (kisiler, sira)
                _, olay, iz.bilgi = iz.guncelle(t, kisi.nokta)
            if olay is not None:
                olaylar.append((iz, olay))
            if iz.kayip_mi(t):
                del self.izler[kimlik]
        return olaylar

    def alarmlari_kapat(self):
        for iz in self.izler.values():
            iz.sifirla()
            iz.son_yatiyor = False
            iz.son_ani_dusus = None
            iz.yatis_gecmis.clear()

    def en_kritik(self):
        oncelik = {"ALARM": 2, "SAYIYOR": 1, "NORMAL": 0}
        en_iyi, en_puan = None, None
        for iz in self.izler.values():
            aci = iz.bilgi["aci"] if iz.bilgi["aci"] is not None else -1
            puan = (oncelik[iz.durum], aci)
            if en_puan is None or puan > en_puan:
                en_iyi, en_puan = iz, puan
        return en_iyi


def yazi_yaz(resim, metin, konum, renk, olcek=0.35):
    kalin = 2 if olcek >= 0.5 else 1
    cv2.putText(resim, metin, konum, FONT, olcek, (0, 0, 0), kalin + 3, cv2.LINE_AA)
    cv2.putText(resim, metin, konum, FONT, olcek, renk, kalin, cv2.LINE_AA)


def insan_maskesi(maske, kisi):
    noktalar = dict(kisi.tum or kisi.nokta)
    omuz = ortalama_nokta(noktalar, (SOL_OMUZ, SAG_OMUZ))
    if BURUN in noktalar and omuz is not None:
        bx, by = noktalar[BURUN]
        noktalar["bas_ustu"] = (bx + 0.8 * (bx - omuz[0]), by + 0.8 * (by - omuz[1]))
    if not noktalar:
        return
    pts = np.array(list(noktalar.values()), dtype=np.float32)
    boyut = max(float(pts[:, 0].max() - pts[:, 0].min()),
                float(pts[:, 1].max() - pts[:, 1].min()), 1.0)
    pay = max(8, int(ARKA_INSAN_PAYI * boyut))
    kabuk = cv2.convexHull(pts).astype(np.int32)
    cv2.fillConvexPoly(maske, kabuk, 255)
    cv2.polylines(maske, [kabuk], True, 255, 2 * pay + 1)
    for x, y in pts.astype(np.int32):
        cv2.circle(maske, (int(x), int(y)), pay, 255, -1)


class ArkaPlan:
    """Atolye ortamini canli ogrenir; insan bolgelerini hic ogrenmez."""

    def __init__(self, genislik, yukseklik):
        self.boyut = (yukseklik, genislik)
        self._sifirla()
        if os.path.exists(ARKA_PLAN_DOSYASI):
            resim = cv2.imread(ARKA_PLAN_DOSYASI)
            if resim is not None and resim.shape[:2] == self.boyut:
                self.resim_f[:] = resim
                self.bilinen[:] = 255
                print("[ARKA PLAN] Kayitli atolye resmi yuklendi, canli guncellenecek.")

    def _sifirla(self):
        h, w = self.boyut
        self.resim_f = np.zeros((h, w, 3), dtype=np.float32)
        self.bilinen = np.zeros((h, w), dtype=np.uint8)
        self.onceki = None
        self.son_hareket = np.zeros((h, w), dtype=np.float32)
        self.kilit = np.zeros((h, w), dtype=np.uint8)
        self.maske = np.zeros((h, w), dtype=np.uint8)
        self.son_canli = None
        self.son_kayit = None
        self._onbellek = None

    def sifirla_ve_yeniden_ogren(self):
        self._sifirla()
        print("[ARKA PLAN] Ortam resmi sifirlandi, bastan ogreniliyor.")

    def bilinen_orani(self):
        return cv2.countNonZero(self.bilinen) / self.bilinen.size

    def guncelle(self, t, kare, kisiler):
        if self.son_canli is not None and t - self.son_canli < ARKA_GUNCELLEME_SN:
            return
        self.son_canli = t
        self._onbellek = None
        if self.son_kayit is None:
            self.son_kayit = t

        # 1) Bu karedeki insan bolgeleri -> kilitle.
        self.maske[:] = 0
        for kisi in kisiler:
            insan_maskesi(self.maske, kisi)
        cv2.bitwise_or(self.kilit, self.maske, dst=self.kilit)
        insan_yok = self.maske == 0

        # 2) Hareket: onceki ornege gore belirgin degisen pikseller.
        if self.onceki is None:
            self.onceki = kare.copy()
            self.son_hareket[:] = t
            return
        hareket = cv2.absdiff(kare, self.onceki).max(axis=2) > ARKA_HAREKET_ESIGI
        self.son_hareket[hareket] = t
        self.onceki[:] = kare
        sabit_sure = t - self.son_hareket
        bilinen = self.bilinen > 0
        kilitli = self.kilit > 0

        # 3) Kilit acma. Bilinen yer: piksel eski ortama donunce (kisi gidince).
        resim = self.resim_f.astype(np.uint8)
        ortama_yakin = cv2.absdiff(kare, resim).max(axis=2) <= ARKA_KILIT_ACMA_ESIGI
        self.kilit[bilinen & kilitli & insan_yok & ortama_yakin] = 0
        #    Bilinmeyen yer: insan gittikten sonra uzun sure bos ve sabit kalinca.
        self.kilit[~bilinen & kilitli & insan_yok
                   & (sabit_sure >= ARKA_KILITLI_BOS_SN)] = 0
        kilitsiz = self.kilit == 0

        # 4) Bilinmeyen, kilitsiz ve birkac saniyedir sabit yerler hemen ogrenilir.
        ilk = ~bilinen & kilitsiz & insan_yok & (sabit_sure >= ARKA_ILK_SABIT_SN)
        if ilk.any():
            self.resim_f[ilk] = kare[ilk]
            self.bilinen[ilk] = 255

        # 5) Bilinen yerlerde uzun sure sabit kalan degisiklik ortama islenir.
        ogren = (bilinen & kilitsiz & insan_yok
                 & (sabit_sure >= ARKA_SABIT_SN)).astype(np.uint8) * 255
        if cv2.countNonZero(ogren):
            cv2.accumulateWeighted(kare, self.resim_f, ARKA_OGRENME_HIZI, mask=ogren)

        if t - self.son_kayit >= ARKA_KAYDET_SN and self.bilinen_orani() > 0.95:
            self.son_kayit = t
            try:
                cv2.imwrite(ARKA_PLAN_DOSYASI, self.goruntu())
            except Exception as hata:
                print("[HATA] Arka plan kaydedilemedi:", hata)

    def goruntu(self):
        if self._onbellek is None:
            resim = self.resim_f.astype(np.uint8)
            resim[self.bilinen == 0] = ISKELET_ZEMIN_RENGI
            self._onbellek = resim
        return self._onbellek


class MqttBaglanti:
    """Pi'deki broker'a baglanir; olaylari yayinlar, komut konusunu dinler.
    Broker o an yoksa arka planda tekrar tekrar baglanmayi dener."""

    def __init__(self):
        self.istemci = None
        self.komutlar = deque()
        self.bagli = False
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            print("[MQTT] paho-mqtt kurulu degil; MQTT kapali (pip install paho-mqtt).")
            return
        try:
            istemci = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id=f"istasyon-{ISTASYON_ADI}")
        except AttributeError:
            istemci = mqtt.Client(client_id=f"istasyon-{ISTASYON_ADI}")
        istemci.will_set(f"{MQTT_KONU_KOKU}/durum",
                         json.dumps({"istasyon": ISTASYON_ADI, "olay": "cevrimdisi"}),
                         qos=1, retain=True)
        istemci.on_connect = self._baglandi
        istemci.on_disconnect = self._koptu
        istemci.on_message = self._mesaj
        istemci.reconnect_delay_set(1, 30)
        try:
            istemci.connect_async(MQTT_SUNUCU, MQTT_PORT, 30)
            istemci.loop_start()
        except Exception as hata:
            print(f"[MQTT] Baslatilamadi ({hata}); MQTT kapali.")
            return
        self.istemci = istemci

    def _baglandi(self, istemci, _veri, _bayrak, kod, *_):
        if getattr(kod, "is_failure", kod != 0):
            print("[MQTT] Baglanti reddedildi:", kod)
            return
        self.bagli = True
        istemci.subscribe(f"{MQTT_KONU_KOKU}/komut", qos=1)
        self.yayinla("durum", {"istasyon": ISTASYON_ADI, "olay": "cevrimici"}, saklansin=True)
        print(f"[MQTT] Baglandi: {MQTT_SUNUCU}:{MQTT_PORT}, konular {MQTT_KONU_KOKU}/...")

    def _koptu(self, *_):
        if self.bagli:
            print("[MQTT] Baglanti koptu, tekrar denenecek.")
        self.bagli = False

    def _mesaj(self, _istemci, _veri, mesaj):
        metin = mesaj.payload.decode("utf-8", "ignore").strip()
        try:
            metin = json.loads(metin).get("komut", metin)
        except (ValueError, AttributeError):
            pass
        self.komutlar.append(str(metin).strip().lower())

    def yayinla(self, alt_konu, veri, saklansin=False):
        if self.istemci is None:
            return
        self.istemci.publish(f"{MQTT_KONU_KOKU}/{alt_konu}",
                             json.dumps(veri, ensure_ascii=False), qos=1, retain=saklansin)

    def kapat(self):
        if self.istemci is None:
            return
        if self.bagli:
            bilgi = self.istemci.publish(
                f"{MQTT_KONU_KOKU}/durum",
                json.dumps({"istasyon": ISTASYON_ADI, "olay": "cevrimdisi"}),
                qos=1, retain=True)
            try:
                bilgi.wait_for_publish(2)
            except Exception:
                pass
        self.bagli = False
        self.istemci.loop_stop()
        self.istemci.disconnect()


class KomutDinleyici:
    """Disaridan gelen komutlari toplar (MQTT ve tetik dosyasi)."""

    def __init__(self, mqtt=None):
        self.mqtt = mqtt

    def al(self):
        komutlar = []
        if self.mqtt is not None:
            while self.mqtt.komutlar:
                komutlar.append(self.mqtt.komutlar.popleft())
        tetik = os.path.join(KLASOR, ARKA_SIFIRLA_TETIK_DOSYASI)
        if os.path.exists(tetik):
            try:
                os.remove(tetik)
            except OSError:
                pass
            komutlar.append("arka_plan_kaydet")
        return komutlar


def arduino_satiri_coz(satir):
    """Arduino'dan gelen satiri olaya cevirir.
    Protokol (Arduino -> Pi, satir satir, 9600 baud):
      HAZIR                      acildi
      ISINIYOR kalan=45          sensorler isiniyor (sn)
      OLCUM gaz=412 duman=300    periyodik olcum
      ALARM gaz=612              esik asildi (ya da: ALARM duman=700)
      TEMIZ gaz                  deger normale dondu (ya da: TEMIZ duman)
      SUSTURULDU                 buzzer susturuldu
      ACIL / ACIL_IPTAL          acil durum butonuna basildi / iptal edildi
    """
    parcalar = satir.strip().split()
    if not parcalar:
        return None
    komut, degerler = parcalar[0].upper(), {}
    for parca in parcalar[1:]:
        if "=" in parca:
            anahtar, deger = parca.split("=", 1)
            try:
                degerler[anahtar.lower()] = int(deger)
            except ValueError:
                degerler[anahtar.lower()] = deger
        else:
            degerler[parca.lower()] = None
    if komut == "HAZIR":
        return {"olay": "arduino_hazir"}
    if komut == "ISINIYOR":
        return {"olay": "sensor_isiniyor", **degerler}
    if komut == "OLCUM":
        return {"olay": "olcum", **degerler}
    if komut in ("ALARM", "TEMIZ"):
        for tur in ("gaz", "duman"):
            if tur in degerler:
                sonek = "alarmi" if komut == "ALARM" else "temiz"
                olay = {"olay": f"{tur}_{sonek}"}
                if degerler[tur] is not None:
                    olay["deger"] = degerler[tur]
                return olay
    if komut == "SUSTURULDU":
        return {"olay": "buzzer_susturuldu"}
    if komut == "ACIL":
        return {"olay": "acil_buton"}
    if komut == "ACIL_IPTAL":
        return {"olay": "acil_buton_iptal"}
    return None


class Arduino:
    """USB seri ile Arduino. Okuma ayri bir is parcaciginda yapilir;
    kablo cikarilip takilirsa kendiliginden yeniden baglanir."""

    def __init__(self):
        self.seri = None
        self.gelen = deque()
        self.calisiyor = False
        self.kilit = threading.Lock()
        try:
            import serial  # pyserial
            import serial.tools.list_ports
        except ImportError:
            print("[ARDUINO] pyserial kurulu degil; Arduino kapali (pip install pyserial).")
            return
        self.serial = serial
        self.calisiyor = True
        self.is_parcacigi = threading.Thread(target=self._dongu, daemon=True)
        self.is_parcacigi.start()

    def _port_bul(self):
        if ARDUINO_PORT != "otomatik":
            return ARDUINO_PORT
        for port in self.serial.tools.list_ports.comports():
            ad = f"{port.device} {port.description} {port.manufacturer or ''}".lower()
            if any(k in ad for k in ("ttyacm", "ttyusb", "arduino", "ch340", "usb serial")):
                return port.device
        return None

    def _dongu(self):
        uyarildi = False
        while self.calisiyor:
            if self.seri is None:
                port = self._port_bul()
                try:
                    if port is None:
                        raise OSError("Arduino portu bulunamadi")
                    seri = self.serial.Serial(port, ARDUINO_HIZ, timeout=1)
                    with self.kilit:
                        self.seri = seri
                    print(f"[ARDUINO] Baglandi: {port}")
                    uyarildi = False
                except Exception as hata:
                    if not uyarildi:
                        print(f"[ARDUINO] Baglanilamadi ({hata}); 5 sn'de bir denenecek.")
                        uyarildi = True
                    time.sleep(5)
                    continue
            try:
                satir = self.seri.readline().decode("ascii", "ignore").strip()
                if satir:
                    self.gelen.append(satir)
            except Exception as hata:
                if not self.calisiyor:
                    break
                print(f"[ARDUINO] Baglanti koptu ({hata}).")
                with self.kilit:
                    try:
                        self.seri.close()
                    except Exception:
                        pass
                    self.seri = None
                self.gelen.append("KOPTU")

    def gonder(self, komut):
        with self.kilit:
            if self.seri is None:
                return False
            try:
                self.seri.write((komut + "\n").encode("ascii"))
                return True
            except Exception as hata:
                print(f"[ARDUINO] Yazilamadi ({hata}).")
                return False

    def al(self):
        satirlar = []
        while self.gelen:
            satirlar.append(self.gelen.popleft())
        return satirlar

    def kapat(self):
        self.calisiyor = False
        if self.seri is not None:
            try:
                self.seri.close()
            except Exception:
                pass


def gizlenecek_maske(t, kisiler, izleyici):
    """Bu karede silinecek (insan olan / olabilecek) bolgeler."""
    maske = np.zeros((YUKSEKLIK, GENISLIK), dtype=np.uint8)
    for kisi in kisiler:
        insan_maskesi(maske, kisi)
    for iz in izleyici.izler.values():
        if iz.kisi is not None or iz.son_kisi is None:
            continue
        yakin_zamanda = t - iz.son_kisi_t <= GIZLEME_HAFIZA_SN
        if yakin_zamanda or iz.supheli:
            insan_maskesi(maske, iz.son_kisi)
    return maske


def canli_sahne(kare, maske, arka_plan):
    """Canli kare; insan bolgeleri bos ortamla (bilinmiyorsa gri) degistirilir."""
    if arka_plan is not None:
        dolgu = arka_plan.goruntu()
    else:
        dolgu = np.full_like(kare, ISKELET_ZEMIN_RENGI)
    if not CANLI_GORUNTU:
        return dolgu.copy()
    sahne = kare.copy()
    np.copyto(sahne, dolgu, where=(maske > 0)[..., None])
    return sahne


def iskelet_resmi(iz, zemin=None):
    """Alarm resmi: zemin (canli sahne, insanlar silinmis) + iskeletler."""
    kisiler, sira = iz.goruntu
    k = ISKELET_RESMI_OLCEK
    w, h = GENISLIK * k, YUKSEKLIK * k
    if zemin is not None:
        resim = cv2.resize(zemin, (w, h), interpolation=cv2.INTER_LINEAR)
    else:
        resim = np.full((h, w, 3), ISKELET_ZEMIN_RENGI, dtype=np.uint8)

    def olcekle(noktalar):
        return {i: (p[0] * k, p[1] * k) for i, p in noktalar.items()}

    for i, kisi in enumerate(kisiler):
        if i != sira:
            iskelet_ciz(resim, olcekle(kisi.nokta), (0, 200, 0), (0, 200, 0))
    dusen = kisiler[sira]
    dusen_nokta = olcekle(dusen.nokta)
    iskelet_ciz(resim, dusen_nokta, (0, 0, 255), (0, 0, 255))

    xler = [p[0] for p in dusen_nokta.values()] or [dusen.merkez[0] * k]
    yler = [p[1] for p in dusen_nokta.values()] or [dusen.merkez[1] * k]
    ex = int(min(max((min(xler) + max(xler)) / 2 - 40, 2), w - 100))
    ey = int(min(max(min(yler) - 10, 16), h - 4))
    yazi_yaz(resim, f"DUSTU #{iz.kimlik}", (ex, ey), (0, 220, 255), 0.6)
    damga = f"{ISTASYON_ADI}  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
    yazi_yaz(resim, damga, (6, h - 8), (255, 255, 255), 0.5)

    basarili, tampon = cv2.imencode(
        ".jpg", resim, [int(cv2.IMWRITE_JPEG_QUALITY), ISKELET_RESMI_KALITE])
    return resim, (tampon.tobytes() if basarili else None)


def _tam(p):
    return (int(p[0]), int(p[1]))


def iskelet_ciz(tuval, noktalar, cizgi_renk=(0, 255, 0), nokta_renk=(0, 0, 255)):
    if not noktalar:
        return
    for a, b in BAGLANTILAR:
        if a in noktalar and b in noktalar:
            cv2.line(tuval, _tam(noktalar[a]), _tam(noktalar[b]), cizgi_renk, 2)
    for p in noktalar.values():
        cv2.circle(tuval, _tam(p), 3, nokta_renk, -1)


DURUM_RENKLERI = {"NORMAL": (0, 255, 0), "SAYIYOR": (0, 165, 255), "ALARM": (0, 0, 255)}


def canli_ciz(tuval, izleyici, k=1):
    for iz in izleyici.izler.values():
        if iz.kisi is None:
            continue
        renk = DURUM_RENKLERI[iz.durum]
        iskelet_ciz(tuval, {i: (p[0] * k, p[1] * k) for i, p in iz.kisi.nokta.items()},
                    renk, renk)
        x, y = iz.kisi.merkez
        yazi_yaz(tuval, f"#{iz.kimlik}", (int(x * k) + 6, int(y * k)), renk, 0.55)


def bilgi_yaz(tuval, izleyici, t, kisi_sayisi):
    izler = list(izleyici.izler.values())
    alarm = [iz for iz in izler if iz.durum == "ALARM"]
    sayan = [iz for iz in izler if iz.durum == "SAYIYOR"]
    if alarm:
        durum = "ALARM"
        metin = "ALARM  #" + ",".join(str(iz.kimlik) for iz in alarm)
    elif sayan:
        iz = max(sayan, key=lambda i: t - i.sayac_basi)
        durum = "SAYIYOR"
        metin = f"SAYIYOR #{iz.kimlik}  {t - iz.sayac_basi:.1f}/{BEKLEME_SANIYESI} sn"
    else:
        gizli = [iz for iz in izler if iz.gizli_basi is not None]
        if gizli:
            iz = gizli[0]
            durum = "SAYIYOR"
            metin = (f"GORUS DISI #{iz.kimlik}  "
                     f"{t - iz.gizli_basi:.0f}/{GIZLI_BEKLEME_SN:.0f} sn")
        else:
            durum, metin = "NORMAL", "NORMAL"
    cv2.putText(tuval, metin, (10, 32), FONT, 0.9, (0, 0, 0), 5)
    cv2.putText(tuval, metin, (10, 32), FONT, 0.9, DURUM_RENKLERI[durum], 2)

    satir = f"kisi:{kisi_sayisi}"
    iz = izleyici.en_kritik()
    if iz is not None and iz.bilgi["aci"] is not None:
        hareket = "-" if iz.bilgi["hareket"] is None else f"{iz.bilgi['hareket']:.3f}"
        satir += (f"  #{iz.kimlik} aci:{iz.bilgi['aci']:.0f} "
                  f"en/boy:{iz.bilgi['en_boy']:.2f} hareket:{hareket}")
        if iz.bilgi["yatiyor"]:
            satir += " YATIYOR"
    yazi_yaz(tuval, satir, (10, tuval.shape[0] - 12), (230, 230, 230), 0.5)


def ekran_var_mi():
    if sys.platform.startswith("linux"):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    return True


def main():
    model_hazirla()
    goster = GORUNTU_GOSTER and ekran_var_mi()

    try:
        kamera = Kamera()
    except RuntimeError as hata:
        print("[HATA]", hata)
        sys.exit(1)

    izleyici = Izleyici()
    arka_plan = ArkaPlan(GENISLIK, YUKSEKLIK) if ARKA_PLAN_KULLAN else None
    print(f"[AYAR] {'Raspberry Pi' if PI_MI else 'Bilgisayar'}: "
          f"model={MODEL_TURU}, hedef {HEDEF_FPS} kare/sn")
    global MQTT
    MQTT = MqttBaglanti() if MQTT_KULLAN else None
    komutlar = KomutDinleyici(MQTT)
    arduino = Arduino() if ARDUINO_KULLAN else None
    arka_sifirla_iste = False
    arduinoya_dusme = None  # Arduino'ya en son bildirilen dusme durumu
    son_ping = 0.0
    secenekler = vision.PoseLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=MODEL_DOSYASI),
        running_mode=vision.RunningMode.VIDEO,
        num_poses=EN_FAZLA_KISI,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    kare_suresi = 1.0 / HEDEF_FPS
    okuma_hatasi = 0
    rapor_zamani = time.monotonic()
    rapor_kare = 0

    if goster:
        cv2.namedWindow(PENCERE_ADI, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(PENCERE_ADI, 640, 480)

    print(f"[BASLADI] En fazla {EN_FAZLA_KISI} kisi izlenecek. Cikmak icin Ctrl+C"
          + (" (ya da pencerede q)." if goster else "."))
    try:
        with vision.PoseLandmarker.create_from_options(secenekler) as iskelet_bulucu:
            while True:
                dongu_basi = time.monotonic()

                kare = kamera.oku()
                if kare is None:
                    okuma_hatasi += 1
                    if okuma_hatasi >= 30:
                        print("[HATA] Kameradan goruntu alinamiyor, cikiliyor.")
                        break
                    time.sleep(0.1)
                    continue
                okuma_hatasi = 0

                rgb = cv2.cvtColor(kare, cv2.COLOR_BGR2RGB)
                mp_goruntu = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                zaman_ms = int(time.monotonic() * 1000)
                sonuc = iskelet_bulucu.detect_for_video(mp_goruntu, zaman_ms)

                t = time.monotonic()
                kisiler = kisileri_al(sonuc, GENISLIK, YUKSEKLIK)
                olaylar = izleyici.guncelle(t, kisiler)
                for komut in komutlar.al():
                    print("[KOMUT] Geldi:", komut)
                    if komut == "arka_plan_kaydet":
                        arka_sifirla_iste = True
                    elif komut == "sustur" and arduino is not None:
                        arduino.gonder("SUSTUR")
                    elif komut == "alarm_kapat":
                        # Uygulamadan "yardim geldi / alarmi kapat"
                        izleyici.alarmlari_kapat()
                        if arduino is not None:
                            arduino.gonder("SIFIRLA")
                        guvenli_gonder({"istasyon": ISTASYON_ADI, "olay": "alarm_kapatildi",
                                        "zaman": datetime.now().isoformat(timespec="seconds")})
                    else:
                        print("[KOMUT] Bilinmeyen komut:", komut)

                if arduino is not None:
                    for satir in arduino.al():
                        if satir == "KOPTU":
                            olay = {"olay": "arduino_koptu"}
                            arduinoya_dusme = None
                        else:
                            olay = arduino_satiri_coz(satir)
                            if olay is None:
                                print("[ARDUINO] Anlasilmayan satir:", satir)
                                continue
                            if olay["olay"] == "arduino_hazir":
                                arduinoya_dusme = None  # durumu yeniden bildir
                        olay["istasyon"] = ISTASYON_ADI
                        olay["zaman"] = datetime.now().isoformat(timespec="seconds")
                        guvenli_gonder(olay)
                if arka_plan is not None:
                    if arka_sifirla_iste:
                        arka_plan.sifirla_ve_yeniden_ogren()
                        guvenli_gonder({"istasyon": ISTASYON_ADI, "olay": "arka_plan_sifirlandi",
                                        "zaman": datetime.now().isoformat(timespec="seconds")})
                    arka_plan.guncelle(t, kare, kisiler)
                arka_sifirla_iste = False

                for iz in izleyici.izler.values():
                    if iz.durum != iz.yazilan_durum:
                        print(f"[DURUM] #{iz.kimlik}: {iz.yazilan_durum} -> {iz.durum}")
                        iz.yazilan_durum = iz.durum

                gizle = gizlenecek_maske(t, kisiler, izleyici)
                sahne = canli_sahne(kare, gizle, arka_plan)

                for iz, olay in olaylar:
                    olay["gorunen_kisi_sayisi"] = len(kisiler)
                    if (olay["olay"] in ("yerde_hareketsiz", "gorus_disi_dusme")
                            and ISKELET_RESMI_EKLE and iz.goruntu is not None):
                        resim, jpeg = iskelet_resmi(iz, sahne)
                        if jpeg is not None:
                            olay["iskelet_jpg_base64"] = base64.b64encode(jpeg).decode("ascii")
                            if ISKELET_DOSYAYA_KAYDET:
                                with open(os.path.join(KLASOR, "son_iskelet.jpg"), "wb") as f:
                                    f.write(jpeg)
                            if goster:
                                cv2.namedWindow(KANIT_PENCERESI, cv2.WINDOW_NORMAL)
                                cv2.resizeWindow(KANIT_PENCERESI, 640, 480)
                                cv2.imshow(KANIT_PENCERESI, resim)
                    guvenli_gonder(olay)

                if arduino is not None:
                    dusme_var = any(iz.durum == "ALARM" for iz in izleyici.izler.values())
                    if dusme_var != arduinoya_dusme:
                        if arduino.gonder("DUSME" if dusme_var else "TAMAM"):
                            arduinoya_dusme = dusme_var
                    if t - son_ping >= ARDUINO_PING_SN:
                        arduino.gonder("PING")
                        son_ping = t

                if goster:
                    # Canli pencere: canli oda goruntusu, insanlar silinmis + iskeletler.
                    k = 2  # 640x480'de ciz, yazilar net olsun
                    tuval = cv2.resize(sahne, (GENISLIK * k, YUKSEKLIK * k))
                    canli_ciz(tuval, izleyici, k)
                    bilgi_yaz(tuval, izleyici, t, len(kisiler))
                    cv2.imshow(PENCERE_ADI, tuval)
                    tus = cv2.waitKey(1) & 0xFF
                    if tus == ord("q"):
                        break
                    if tus == ord("k"):
                        arka_sifirla_iste = True

                rapor_kare += 1
                if FPS_RAPOR_SN > 0 and t - rapor_zamani >= FPS_RAPOR_SN:
                    print(f"[HIZ] {rapor_kare / (t - rapor_zamani):.1f} kare/sn "
                          f"(son karede {len(kisiler)} kisi)")
                    rapor_zamani, rapor_kare = t, 0

                kalan = kare_suresi - (time.monotonic() - dongu_basi)
                if kalan > 0:
                    time.sleep(kalan)

    except KeyboardInterrupt:
        print("\n[DURDU] Kullanici durdurdu.")
    finally:
        kamera.kapat()
        if arduino is not None:
            arduino.kapat()
        if MQTT is not None:
            MQTT.kapat()
        if goster:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
