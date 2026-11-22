"""Dusme algoritmasi icin sentetik iskeletlerle testler.

Kamera veya model gerekmez:  python -m unittest discover -s tests
"""
import importlib.util
import math
import os
import random
import sys
import types
import unittest

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _modul_yukle():
    # mediapipe kurulu degilse (or. gelistirme bilgisayari) sahte modul koy;
    # algoritma mediapipe'e dokunmaz.
    try:
        import mediapipe  # noqa: F401
    except ImportError:
        mp = types.ModuleType("mediapipe")
        tasks = types.ModuleType("mediapipe.tasks")
        tpy = types.ModuleType("mediapipe.tasks.python")
        tpy.vision = types.ModuleType("mediapipe.tasks.python.vision")
        tasks.python = tpy
        mp.tasks = tasks
        sys.modules.update({"mediapipe": mp, "mediapipe.tasks": tasks,
                            "mediapipe.tasks.python": tpy,
                            "mediapipe.tasks.python.vision": tpy.vision})
    yol = os.path.join(KOK, "PythonApplication1.py")
    spec = importlib.util.spec_from_file_location("uygulama", yol)
    modul = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modul)
    return modul


U = _modul_yukle()
FPS = 5


def iskelet(kalca_x, kalca_y, govde_aci, bacak_aci, govde=50, bacak=60, titreme=0.0):
    """govde_aci / bacak_aci: dikeyden derece. Govde kalcadan yukari/sola,
    bacaklar kalcadan asagi/saga dogru doner."""
    g = math.radians(govde_aci)
    b = math.radians(bacak_aci)
    yukari = (-math.sin(g), -math.cos(g))
    asagi = (math.sin(b), math.cos(b))
    yan = (math.cos(g), -math.sin(g))
    n = {}

    def koy(no, x, y):
        n[no] = (x + random.uniform(-titreme, titreme),
                 y + random.uniform(-titreme, titreme))

    for no, isaret in ((U.SOL_KALCA, -1), (U.SAG_KALCA, 1)):
        koy(no, kalca_x + isaret * 8 * yan[0], kalca_y + isaret * 8 * yan[1])
    omuz = (kalca_x + govde * yukari[0], kalca_y + govde * yukari[1])
    for no, isaret in ((U.SOL_OMUZ, -1), (U.SAG_OMUZ, 1)):
        koy(no, omuz[0] + isaret * 12 * yan[0], omuz[1] + isaret * 12 * yan[1])
    koy(U.BURUN, omuz[0] + 18 * yukari[0], omuz[1] + 18 * yukari[1])
    for no, oran in ((U.SOL_DIZ, 0.5), (U.SAG_DIZ, 0.5),
                     (U.SOL_AYAK_BILEGI, 1.0), (U.SAG_AYAK_BILEGI, 1.0)):
        koy(no, kalca_x + bacak * oran * asagi[0], kalca_y + bacak * oran * asagi[1])
    return n


def ara(a, b, oran):
    return a + (b - a) * oran


class Senaryo:
    def __init__(self):
        self.iz = U.DusmeTakipcisi(1)
        self.t = 100.0
        self.olaylar = []
        self.alarm_zamani = None
        self.bacak = 60

    def bekle(self, sure, *konum, titreme=0.3):
        for _ in range(int(round(sure * FPS))):
            self.kare(*konum, titreme=titreme)

    def gecis(self, sure, bas, son, titreme=0.3):
        adim = int(round(sure * FPS))
        for i in range(1, adim + 1):
            o = i / adim
            self.kare(*[ara(a, b, o) for a, b in zip(bas, son)], titreme=titreme)

    def kare(self, x, y, govde_aci, bacak_aci, titreme=0.3):
        self.t += 1.0 / FPS
        durum, olay, _ = self.iz.guncelle(
            self.t, iskelet(x, y, govde_aci, bacak_aci, bacak=self.bacak, titreme=titreme))
        if olay is not None:
            self.olaylar.append((self.t, olay))
        if durum == "ALARM" and self.alarm_zamani is None:
            self.alarm_zamani = self.t
        return durum

    def turler(self):
        return [o["olay"] for _, o in self.olaylar]


AYAKTA = (160, 140, 0, 0)
YERDE = (160, 200, 88, 88)


class DusmeTestleri(unittest.TestCase):

    def setUp(self):
        random.seed(4)

    def test_ani_dusus_3_saniyede_alarm(self):
        s = Senaryo()
        s.bekle(2, *AYAKTA)
        dusus_basi = s.t
        s.gecis(0.6, AYAKTA, YERDE)
        s.bekle(4, *YERDE, titreme=3.0)  # yerde kipirdiyor
        self.assertEqual(s.turler()[:1], ["yerde_hareketsiz"])
        self.assertTrue(s.olaylar[0][1]["ani_dusus"])
        self.assertLessEqual(s.alarm_zamani - dusus_basi, 3.0)

    def test_yavas_uzanma_hareketsizse_sonradan_alarm(self):
        s = Senaryo()
        s.bekle(2, *AYAKTA)
        s.gecis(3.0, AYAKTA, YERDE)
        uzanma_sonu = s.t
        s.bekle(7, *YERDE)
        self.assertEqual(s.turler(), ["yerde_hareketsiz"])
        self.assertFalse(s.olaylar[0][1]["ani_dusus"])
        self.assertGreaterEqual(s.alarm_zamani - uzanma_sonu, U.BEKLEME_SANIYESI - 0.5)

    def test_yavas_uzanip_kipirdayana_alarm_yok(self):
        s = Senaryo()
        s.bekle(2, *AYAKTA)
        s.gecis(3.0, AYAKTA, YERDE)
        s.bekle(8, *YERDE, titreme=6.0)
        self.assertEqual(s.turler(), [])

    def test_hizli_egilme_alarm_degil(self):
        s = Senaryo()
        s.bacak = 40  # kisa/bukuk bacak: kutu yatay gorunur, sadece bacak acisi ayirir
        s.bekle(2, *AYAKTA)
        egik = (160, 140, 90, 0)  # govde yatay, bacaklar dik
        s.gecis(0.4, AYAKTA, egik)
        s.bekle(8, *egik)
        self.assertEqual(s.turler(), [])
        self.assertIsNone(s.alarm_zamani)

    def test_ayakkabi_baglama_alarm_degil(self):
        s = Senaryo()
        s.bekle(2, *AYAKTA)
        comelme = (160, 175, 65, 10)  # kalca asagida, govde one egik, ayaklar altta
        s.gecis(0.6, AYAKTA, comelme)
        s.bekle(8, *comelme)
        self.assertEqual(s.turler(), [])

    def test_dusup_kalkinca_kalkti_olayi(self):
        s = Senaryo()
        s.bekle(2, *AYAKTA)
        s.gecis(0.6, AYAKTA, YERDE)
        s.bekle(3, *YERDE)
        s.gecis(2.0, YERDE, AYAKTA)
        s.bekle(2, *AYAKTA)
        self.assertEqual(s.turler(), ["yerde_hareketsiz", "kisi_kalkti"])
        self.assertEqual(s.iz.durum, "NORMAL")

    def test_dusuk_fps_ile_de_ani_dusus(self):
        global FPS
        eski, FPS = FPS, 3
        try:
            s = Senaryo()
            s.bekle(2, *AYAKTA)
            dusus_basi = s.t
            s.gecis(0.7, AYAKTA, YERDE)
            s.bekle(4, *YERDE, titreme=3.0)
            self.assertTrue(s.olaylar and s.olaylar[0][1]["ani_dusus"])
            self.assertLessEqual(s.alarm_zamani - dusus_basi, 3.0)
        finally:
            FPS = eski


if __name__ == "__main__":
    unittest.main()
