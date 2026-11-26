#!/usr/bin/env python3

import base64
import json
import math
import os
import sys
import time
import urllib.request
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

ISTASYON_ADI = "atolye-1"
KAMERA_KAYNAGI = "otomatik"
KAMERA_NO = 0
GENISLIK, YUKSEKLIK = 320, 240
HEDEF_FPS = 5
GORUNTU_GOSTER = True
EN_FAZLA_KISI = 5
ESLESME_MESAFESI = 1.0
MODEL_DOSYASI = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "pose_landmarker_lite.task")
MODEL_ADRESI = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
                "pose_landmarker_lite/float16/1/pose_landmarker_lite.task")
MIN_GORUNURLUK = 0.5
YATIS_ACISI = 50
YATIS_EN_BOY = 1.0
HAREKET_PENCERESI = 1.0
HAREKET_ESIGI = 0.04
BEKLEME_SANIYESI = 5
IPTAL_TEYIT_SN = 1.0
KAYIP_TOLERANS_SN = 2.0
ALARM_KAYIP_TOLERANS_SN = 30.0
# Alarm resmi: arka planda atolye ortami, insanlar YALNIZCA iskelet olarak.
# Ortam resmi bir kez "kalibrasyon" ile kaydedilir: oda bosken birkac dakika
# boyunca, icinde hic insan bulunmayan kareler toplanip medyanlari alinir.
# Sonra resim sabit kalir (surekli ogrenme yok), boyle insanlar ona karisamaz.
# Ortam degisirse yeniden kalibrasyon baslatilir (pencerede "k" tusu,
# klasore "arka_plan_kaydet" adli bos dosya koymak veya MQTT komutu).
ISKELET_RESMI_EKLE = True
ISKELET_RESMI_OLCEK = 2
ISKELET_RESMI_KALITE = 70
ISKELET_ZEMIN_RENGI = (60, 60, 60)  # ortami henuz bilinmeyen yerler
ARKA_PLAN_KULLAN = True
ARKA_PLAN_DOSYASI = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "arka_plan.png")
ACILISTA_KALIBRASYON = True   # kayitli arka plan yoksa acilista kalibrasyon yap
KALIBRASYON_SN = 300          # 5 dakika; bu surede oda bos birakilmali
KALIBRASYON_ORNEK_ARALIGI = 3.0  # kac saniyede bir ornek kare alinsin
KALIBRASYON_MIN_ORNEK = 20    # bu kadar insansiz kare toplanmadan bitmez
KALIBRASYON_TETIK_DOSYASI = "arka_plan_kaydet"
# Ileride mobil uygulamadaki "ortami kaydet" tusu bu konuya mesaj atacak.
# paho-mqtt kurulu degilse veya broker yoksa bu ozellik sessizce kapali kalir.
MQTT_KOMUT_DINLE = True
MQTT_SUNUCU, MQTT_PORT = "localhost", 1883
MQTT_KOMUT_KONUSU = f"atolye/{ISTASYON_ADI}/komut"
ISKELET_DOSYAYA_KAYDET = False
TAKIP_MIN_GORUNURLUK = 0.1
FPS_RAPOR_SN = 10


def veri_gonder(veri):
    # ╔═══════════════════════════════════════════════════════════════╗
    # ║  >>>>>  VERİNİN GİDECEĞİ YER BURASI (SİZ DOLDURACAKSINIZ)  <<<<<  ║
    # ╚═══════════════════════════════════════════════════════════════╝
    # veri: sozluk. Iskelet resmi "iskelet_jpg_base64" anahtarinda (base64 JPEG).
    gosterilecek = dict(veri)
    if gosterilecek.get("iskelet_jpg_base64"):
        kb = len(gosterilecek["iskelet_jpg_base64"]) * 3 // 4 / 1024
        gosterilecek["iskelet_jpg_base64"] = f"<JPEG iskelet resmi, yaklasik {kb:.1f} KB>"
    print("[VERI] ->", json.dumps(gosterilecek, ensure_ascii=False))


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
    print("[MODEL] pose_landmarker_lite.task bulunamadi, indiriliyor (tek seferlik)...")
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
    return aci, en_boy, boyut


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
        self.bilgi = {"aci": None, "en_boy": None, "hareket": None,
                      "yatiyor": None, "gecen": None}
        self.yazilan_durum = "NORMAL"

    def sifirla(self):
        self.durum = "NORMAL"
        self.sayac_basi = None
        self.iptal_basi = None

    def kayip_mi(self, t):
        tolerans = ALARM_KAYIP_TOLERANS_SN if self.durum == "ALARM" else KAYIP_TOLERANS_SN
        return self.son_gorulen is None or (t - self.son_gorulen) > tolerans

    def olay_olustur(self, tur, t):
        return {
            "istasyon": ISTASYON_ADI,
            "olay": tur,
            "kisi_no": self.kimlik,
            "zaman": datetime.now().isoformat(timespec="seconds"),
            "yerde_kalma_sn": round(t - self.sayac_basi, 1),
        }

    def guncelle(self, t, noktalar):
        bilgi = {"aci": None, "en_boy": None, "hareket": None,
                 "yatiyor": None, "gecen": None}
        olay = None

        if noktalar is None:
            if self.kayip_mi(t):
                self.sifirla()
            return self.durum, olay, bilgi
        self.son_gorulen = t

        olcum = durus_olc(noktalar)
        if olcum is None:
            return self.durum, olay, bilgi
        aci, en_boy, boyut = olcum

        yatiyor = (aci >= YATIS_ACISI) and (en_boy >= YATIS_EN_BOY)
        hareket = self.olcer.olc(t, noktalar, boyut)
        hareketsiz = (hareket is None) or (hareket < HAREKET_ESIGI)

        bilgi.update(aci=aci, en_boy=en_boy, hareket=hareket, yatiyor=yatiyor)

        if self.durum == "NORMAL":
            if yatiyor and hareketsiz:
                self.durum = "SAYIYOR"
                self.sayac_basi = t
                self.iptal_basi = None

        elif self.durum == "SAYIYOR":
            if yatiyor and hareketsiz:
                self.iptal_basi = None
                if t - self.sayac_basi >= BEKLEME_SANIYESI:
                    self.durum = "ALARM"
                    olay = self.olay_olustur("yerde_hareketsiz", t)
            else:
                if self.iptal_basi is None:
                    self.iptal_basi = t
                elif t - self.iptal_basi >= IPTAL_TEYIT_SN:
                    self.sifirla()

        elif self.durum == "ALARM":
            if yatiyor:
                self.iptal_basi = None
            else:
                if self.iptal_basi is None:
                    self.iptal_basi = t
                elif t - self.iptal_basi >= IPTAL_TEYIT_SN:
                    olay = self.olay_olustur("kisi_kalkti", t)
                    self.sifirla()

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
                iz.merkez, iz.boyut = kisi.merkez, kisi.boyut
                iz.goruntu = (kisiler, sira)
                _, olay, iz.bilgi = iz.guncelle(t, kisi.nokta)
            if olay is not None:
                olaylar.append((iz, olay))
            if iz.kayip_mi(t):
                del self.izler[kimlik]
        return olaylar

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
    cv2.putText(resim, metin, konum, FONT, olcek, (0, 0, 0), 3)
    cv2.putText(resim, metin, konum, FONT, olcek, renk, 1)


class ArkaPlan:
    """Atolyenin bos halini kalibrasyonla kaydeder; sonra sabit kalir."""

    def __init__(self, genislik, yukseklik):
        self.boyut = (yukseklik, genislik)
        self.resim = None
        self.ornekler = []
        self.kalib_basi = None
        self.son_ornek = None
        self.uyarildi = False
        if os.path.exists(ARKA_PLAN_DOSYASI):
            resim = cv2.imread(ARKA_PLAN_DOSYASI)
            if resim is not None and resim.shape[:2] == self.boyut:
                self.resim = resim
                print("[ARKA PLAN] Kayitli atolye resmi yuklendi:", ARKA_PLAN_DOSYASI)

    @property
    def kalibre_ediliyor(self):
        return self.kalib_basi is not None

    def kalibrasyon_baslat(self, t):
        self.ornekler = []
        self.kalib_basi = t
        self.son_ornek = None
        self.uyarildi = False
        print(f"[ARKA PLAN] Kalibrasyon basladi: {KALIBRASYON_SN} sn boyunca "
              "oda bos birakilmali.")

    def kalan_sn(self, t):
        return max(0.0, KALIBRASYON_SN - (t - self.kalib_basi))

    def guncelle(self, t, kare, kisiler):
        """Kalibrasyon bittiyse olay adi dondurur, yoksa None."""
        if not self.kalibre_ediliyor:
            return None
        bos_oda = not kisiler
        if bos_oda and (self.son_ornek is None
                        or t - self.son_ornek >= KALIBRASYON_ORNEK_ARALIGI):
            self.ornekler.append(kare.copy())
            self.son_ornek = t
        if self.kalan_sn(t) > 0:
            return None
        if len(self.ornekler) < KALIBRASYON_MIN_ORNEK:
            if not self.uyarildi:
                print(f"[ARKA PLAN] Odada insan goruldugu icin yeterli bos kare yok "
                      f"({len(self.ornekler)}/{KALIBRASYON_MIN_ORNEK}). "
                      "Oda bosalana kadar devam ediliyor.")
                self.uyarildi = True
            return None
        # Medyan: ornek karelerin birinde kisa sureligine gorunen biri olsa bile silinir.
        self.resim = np.median(np.stack(self.ornekler), axis=0).astype(np.uint8)
        self.ornekler = []
        self.kalib_basi = None
        try:
            cv2.imwrite(ARKA_PLAN_DOSYASI, self.resim)
            print("[ARKA PLAN] Kalibrasyon bitti, kaydedildi:", ARKA_PLAN_DOSYASI)
        except Exception as hata:
            print("[HATA] Arka plan kaydedilemedi:", hata)
        return "arka_plan_kaydedildi"

    def goruntu(self):
        if self.resim is None:
            return np.full(self.boyut + (3,), ISKELET_ZEMIN_RENGI, dtype=np.uint8)
        return self.resim


class KomutDinleyici:
    """Disaridan gelen komutlari toplar (MQTT ve tetik dosyasi)."""

    def __init__(self):
        self.bekleyen = []
        self.istemci = None
        if MQTT_KOMUT_DINLE:
            self._mqtt_baslat()

    def _mqtt_baslat(self):
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            print("[KOMUT] paho-mqtt kurulu degil; MQTT komutlari kapali.")
            return
        try:
            try:
                istemci = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            except AttributeError:
                istemci = mqtt.Client()

            def mesaj_geldi(_istemci, _veri, mesaj):
                self.bekleyen.append(mesaj.payload.decode("utf-8", "ignore").strip())

            istemci.on_message = mesaj_geldi
            istemci.connect(MQTT_SUNUCU, MQTT_PORT, 30)
            istemci.subscribe(MQTT_KOMUT_KONUSU)
            istemci.loop_start()
            self.istemci = istemci
            print(f"[KOMUT] MQTT dinleniyor: {MQTT_KOMUT_KONUSU}")
        except Exception as hata:
            print(f"[KOMUT] MQTT baglantisi kurulamadi ({hata}); MQTT komutlari kapali.")

    def al(self):
        tetik = os.path.join(KLASOR, KALIBRASYON_TETIK_DOSYASI)
        if os.path.exists(tetik):
            try:
                os.remove(tetik)
            except OSError:
                pass
            self.bekleyen.append("arka_plan_kaydet")
        komutlar, self.bekleyen = self.bekleyen, []
        return komutlar

    def kapat(self):
        if self.istemci is not None:
            self.istemci.loop_stop()
            self.istemci.disconnect()


def iskelet_resmi(iz, arka_plan=None):
    kisiler, sira = iz.goruntu
    k = ISKELET_RESMI_OLCEK
    w, h = GENISLIK * k, YUKSEKLIK * k
    if arka_plan is not None:
        resim = cv2.resize(arka_plan.goruntu(), (w, h), interpolation=cv2.INTER_LINEAR)
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


def canli_ciz(tuval, izleyici):
    for iz in izleyici.izler.values():
        if iz.kisi is None:
            continue
        renk = DURUM_RENKLERI[iz.durum]
        iskelet_ciz(tuval, iz.kisi.nokta, renk, renk)
        x, y = iz.kisi.merkez
        cv2.putText(tuval, f"#{iz.kimlik}", (int(x) + 4, int(y)), FONT, 0.4, renk, 1)


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
        durum, metin = "NORMAL", "NORMAL"
    cv2.putText(tuval, metin, (6, 20), FONT, 0.6, DURUM_RENKLERI[durum], 2)

    satir = f"kisi:{kisi_sayisi}"
    iz = izleyici.en_kritik()
    if iz is not None and iz.bilgi["aci"] is not None:
        hareket = "-" if iz.bilgi["hareket"] is None else f"{iz.bilgi['hareket']:.3f}"
        satir += (f"  #{iz.kimlik} aci:{iz.bilgi['aci']:.0f} "
                  f"en/boy:{iz.bilgi['en_boy']:.2f} hareket:{hareket}")
        if iz.bilgi["yatiyor"]:
            satir += " YATIYOR"
    cv2.putText(tuval, satir, (6, YUKSEKLIK - 8), FONT, 0.38, (170, 170, 170), 1)


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
    if arka_plan is not None and arka_plan.resim is None and ACILISTA_KALIBRASYON:
        arka_plan.kalibrasyon_baslat(time.monotonic())
    komutlar = KomutDinleyici()
    kalibrasyon_iste = False
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
                    if komut == "arka_plan_kaydet":
                        kalibrasyon_iste = True
                    else:
                        print("[KOMUT] Bilinmeyen komut:", komut)
                if arka_plan is not None:
                    if kalibrasyon_iste:
                        arka_plan.kalibrasyon_baslat(t)
                        guvenli_gonder({"istasyon": ISTASYON_ADI, "olay": "kalibrasyon_basladi",
                                     "sure_sn": KALIBRASYON_SN,
                                     "zaman": datetime.now().isoformat(timespec="seconds")})
                    kalib_olay = arka_plan.guncelle(t, kare, kisiler)
                    if kalib_olay is not None:
                        guvenli_gonder({"istasyon": ISTASYON_ADI, "olay": kalib_olay,
                                     "zaman": datetime.now().isoformat(timespec="seconds")})
                kalibrasyon_iste = False

                for iz in izleyici.izler.values():
                    if iz.durum != iz.yazilan_durum:
                        print(f"[DURUM] #{iz.kimlik}: {iz.yazilan_durum} -> {iz.durum}")
                        iz.yazilan_durum = iz.durum

                for iz, olay in olaylar:
                    olay["gorunen_kisi_sayisi"] = len(kisiler)
                    if olay["olay"] == "yerde_hareketsiz" and ISKELET_RESMI_EKLE:
                        resim, jpeg = iskelet_resmi(iz, arka_plan)
                        if jpeg is not None:
                            olay["iskelet_jpg_base64"] = base64.b64encode(jpeg).decode("ascii")
                            if ISKELET_DOSYAYA_KAYDET:
                                with open(os.path.join(KLASOR, "son_iskelet.jpg"), "wb") as f:
                                    f.write(jpeg)
                            if goster:
                                cv2.namedWindow(KANIT_PENCERESI, cv2.WINDOW_NORMAL)
                                cv2.resizeWindow(KANIT_PENCERESI, 640, 480)
                                cv2.imshow(KANIT_PENCERESI, resim)
                    try:
                        veri_gonder(olay)
                    except Exception as hata:
                        print("[HATA] Veri gonderilemedi:", hata)

                if goster:
                    tuval = np.zeros((YUKSEKLIK, GENISLIK, 3), dtype=np.uint8)
                    canli_ciz(tuval, izleyici)
                    bilgi_yaz(tuval, izleyici, t, len(kisiler))
                    if arka_plan is not None and arka_plan.kalibre_ediliyor:
                        cv2.putText(tuval, f"KALIBRASYON: odayi bosaltin "
                                    f"{arka_plan.kalan_sn(t):.0f} sn",
                                    (6, 40), FONT, 0.45, (255, 200, 0), 1)
                    cv2.imshow(PENCERE_ADI, tuval)
                    tus = cv2.waitKey(1) & 0xFF
                    if tus == ord("q"):
                        break
                    if tus == ord("k"):
                        kalibrasyon_iste = True

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
        komutlar.kapat()
        if goster:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
