// =====================================================================
//  ARDUINO KODU - Gaz/duman sensoru, buzzer, RGB isik, acil buton
//  Akilli Acil Durum Istasyonu (TUBITAK 4006-A)
//  Nereye yuklenir: Arduino Uno / Nano (Arduino IDE ile)
//  Raspberry Pi tarafi: raspberry_pi/kamera_dusme_algilama.py
// =====================================================================
// Gorevi: gaz ve duman sensorlerini okumak, tehlikede buzzer ve RGB isikla
// uyarmak, Raspberry Pi'ye USB seri uzerinden bilgi vermek ve Pi'den gelen
// "DUSME" komutunda uyari vermek.
//
// Seri protokol (9600 baud, her mesaj bir satir):
//   Pi -> Arduino : DUSME | TAMAM | SUSTUR | SIFIRLA | PING
//   Arduino -> Pi : HAZIR | ISINIYOR kalan=45 | OLCUM gaz=412 duman=300
//                   ALARM gaz=612 | ALARM duman=700 | TEMIZ gaz | TEMIZ duman
//                   SUSTURULDU | ACIL | ACIL_IPTAL
//
// Acil durum butonu: kisa basis = ACIL (yardim cagir), 3 sn basili tutmak =
// ACIL_IPTAL. Kamera kisiyi goremese bile (masa arkasi vb.) yardim cagrilabilir.
//
// Isik renkleri (oncelik sirasiyla):
//   acil buton : kirmizi, hizli yanip soner + hizli bip
//   duman  : mor, hizli yanip soner + hizli bip
//   gaz    : turuncu, hizli yanip soner + hizli bip
//   dusme  : kirmizi, yavas yanip soner + kesik bip
//   Pi'den 30 sn haber yok : mavi yanip soner (sessiz)
//   sensorler isiniyor     : mavi sabit
//   her sey normal         : yesil sabit

// ---------------- AYARLAR ----------------
const int GAZ_PIN = A0;        // gaz sensoru (orn. MQ-5 / MQ-6) analog cikisi
const int DUMAN_PIN = A1;      // duman sensoru (orn. MQ-2) analog cikisi
const int BUZZER_PIN = 8;
const int KIRMIZI_PIN = 9;     // RGB LED (PWM pinleri)
const int YESIL_PIN = 10;
const int MAVI_PIN = 11;
const int BUTON_PIN = 2;       // acil durum butonu: bir ucu D2, diger ucu GND
const unsigned long BUTON_IPTAL_MS = 3000UL;

const bool ORTAK_ANOT_LED = false;  // ortak anotlu RGB LED ise true yapin
const bool AKTIF_BUZZER = true;     // pasif (ton isteyen) buzzer ise false yapin
const int PASIF_BUZZER_FREKANS = 2000;

const int GAZ_ESIK = 400;      // 0-1023 arasi; atolyede deneyerek ayarlayin
const int DUMAN_ESIK = 400;
const int HISTEREZIS = 50;     // alarm, esigin bu kadar altina inince biter
const int ONAY_SAYISI = 5;     // art arda bu kadar okuma esik ustundeyse alarm

const unsigned long ISINMA_MS = 60000UL;       // MQ sensorleri isinma suresi
const unsigned long OKUMA_ARALIGI_MS = 200UL;
const unsigned long OLCUM_GONDERME_MS = 5000UL;
const unsigned long SUSTURMA_MS = 300000UL;    // SUSTUR: buzzer 5 dk susar
const unsigned long PI_ZAMAN_ASIMI_MS = 30000UL;

// ---------------- DURUM ----------------
const int ORTALAMA_BOYU = 8;
int gazOkumalar[ORTALAMA_BOYU];
int dumanOkumalar[ORTALAMA_BOYU];
int okumaSirasi = 0;
bool okumaDoldu = false;

int gazDeger = 0;
int dumanDeger = 0;
int gazSayac = 0;
int dumanSayac = 0;
bool gazAlarm = false;
bool dumanAlarm = false;
bool dusmeAlarm = false;
bool acilAlarm = false;
bool butonOnceki = false;
unsigned long butonBasma = 0;
bool butonIptalGonderildi = false;

unsigned long sonOkuma = 0;
unsigned long sonOlcumGonderme = 0;
unsigned long sonPiMesaji = 0;
unsigned long susturmaBitis = 0;
bool susturuldu = false;
bool piHicKonusmadi = true;

char gelen[32];
int gelenUzunluk = 0;

// ---------------- YARDIMCILAR ----------------
void renkYaz(int r, int g, int b) {
  if (ORTAK_ANOT_LED) {
    r = 255 - r;
    g = 255 - g;
    b = 255 - b;
  }
  analogWrite(KIRMIZI_PIN, r);
  analogWrite(YESIL_PIN, g);
  analogWrite(MAVI_PIN, b);
}

void buzzerYaz(bool acik) {
  if (AKTIF_BUZZER) {
    digitalWrite(BUZZER_PIN, acik ? HIGH : LOW);
  } else if (acik) {
    tone(BUZZER_PIN, PASIF_BUZZER_FREKANS);
  } else {
    noTone(BUZZER_PIN);
  }
}

bool yanipSon(unsigned long simdi, unsigned long periyot) {
  return (simdi / periyot) % 2 == 0;
}

int ortalama(const int *dizi) {
  int adet = okumaDoldu ? ORTALAMA_BOYU : okumaSirasi;
  if (adet == 0) return 0;
  long toplam = 0;
  for (int i = 0; i < adet; i++) toplam += dizi[i];
  return (int)(toplam / adet);
}

void susturmayiKaldir() {
  susturuldu = false;
}

// Esik ve histerezisli alarm kontrolu. Durum degisirse Pi'ye bildirir.
void alarmKontrol(const char *ad, int deger, int esik, int &sayac, bool &alarm) {
  if (!alarm) {
    sayac = (deger >= esik) ? sayac + 1 : 0;
    if (sayac >= ONAY_SAYISI) {
      alarm = true;
      sayac = 0;
      susturmayiKaldir();  // yeni tehlike: susturma iptal
      Serial.print("ALARM ");
      Serial.print(ad);
      Serial.print("=");
      Serial.println(deger);
    }
  } else if (deger < esik - HISTEREZIS) {
    alarm = false;
    Serial.print("TEMIZ ");
    Serial.println(ad);
  }
}

void komutIsle(const char *komut, unsigned long simdi) {
  sonPiMesaji = simdi;
  piHicKonusmadi = false;
  if (strcmp(komut, "DUSME") == 0) {
    if (!dusmeAlarm) susturmayiKaldir();
    dusmeAlarm = true;
  } else if (strcmp(komut, "TAMAM") == 0) {
    dusmeAlarm = false;
  } else if (strcmp(komut, "SIFIRLA") == 0) {
    dusmeAlarm = false;
    acilAlarm = false;
  } else if (strcmp(komut, "SUSTUR") == 0) {
    susturuldu = true;
    susturmaBitis = simdi + SUSTURMA_MS;
    Serial.println("SUSTURULDU");
  }
  // PING: sadece "Pi hayatta" demek, sonPiMesaji guncellendi.
}

void seriOku(unsigned long simdi) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (gelenUzunluk > 0) {
        gelen[gelenUzunluk] = '\0';
        komutIsle(gelen, simdi);
        gelenUzunluk = 0;
      }
    } else if (gelenUzunluk < (int)sizeof(gelen) - 1) {
      gelen[gelenUzunluk++] = c;
    }
  }
}

void butonOku(unsigned long simdi) {
  bool basili = digitalRead(BUTON_PIN) == LOW;  // INPUT_PULLUP: basinca LOW
  if (basili && !butonOnceki) {
    butonBasma = simdi;
    butonIptalGonderildi = false;
  }
  if (basili && !butonIptalGonderildi && simdi - butonBasma >= BUTON_IPTAL_MS) {
    butonIptalGonderildi = true;
    if (acilAlarm) {
      acilAlarm = false;
      Serial.println("ACIL_IPTAL");
    }
  }
  if (!basili && butonOnceki && !butonIptalGonderildi && simdi - butonBasma >= 50) {
    if (!acilAlarm) susturmayiKaldir();
    acilAlarm = true;
    Serial.println("ACIL");
  }
  butonOnceki = basili;
}

void cikislariGuncelle(unsigned long simdi) {
  bool isiniyor = simdi < ISINMA_MS;
  bool piYok = !piHicKonusmadi && (simdi - sonPiMesaji > PI_ZAMAN_ASIMI_MS);
  if (susturuldu && (long)(simdi - susturmaBitis) >= 0) susturuldu = false;

  bool sesli = false;
  if (acilAlarm) {
    bool a = yanipSon(simdi, 150);
    a ? renkYaz(255, 0, 0) : renkYaz(0, 0, 0);
    sesli = a;
  } else if (dumanAlarm) {
    bool a = yanipSon(simdi, 150);
    a ? renkYaz(160, 0, 255) : renkYaz(0, 0, 0);
    sesli = a;
  } else if (gazAlarm) {
    bool a = yanipSon(simdi, 150);
    a ? renkYaz(255, 80, 0) : renkYaz(0, 0, 0);
    sesli = a;
  } else if (dusmeAlarm) {
    bool a = yanipSon(simdi, 500);
    a ? renkYaz(255, 0, 0) : renkYaz(0, 0, 0);
    sesli = a;
  } else if (piYok) {
    yanipSon(simdi, 700) ? renkYaz(0, 0, 255) : renkYaz(0, 0, 0);
  } else if (isiniyor) {
    renkYaz(0, 0, 255);
  } else {
    renkYaz(0, 255, 0);
  }
  buzzerYaz(sesli && !susturuldu);
}

// ---------------- ANA PROGRAM ----------------
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(KIRMIZI_PIN, OUTPUT);
  pinMode(YESIL_PIN, OUTPUT);
  pinMode(MAVI_PIN, OUTPUT);
  pinMode(BUTON_PIN, INPUT_PULLUP);
  buzzerYaz(false);
  renkYaz(0, 0, 255);
  Serial.begin(9600);
  Serial.println("HAZIR");
}

void loop() {
  unsigned long simdi = millis();
  seriOku(simdi);
  butonOku(simdi);

  if (simdi - sonOkuma >= OKUMA_ARALIGI_MS) {
    sonOkuma = simdi;
    gazOkumalar[okumaSirasi] = analogRead(GAZ_PIN);
    dumanOkumalar[okumaSirasi] = analogRead(DUMAN_PIN);
    okumaSirasi = (okumaSirasi + 1) % ORTALAMA_BOYU;
    if (okumaSirasi == 0) okumaDoldu = true;
    gazDeger = ortalama(gazOkumalar);
    dumanDeger = ortalama(dumanOkumalar);

    if (simdi >= ISINMA_MS) {
      alarmKontrol("gaz", gazDeger, GAZ_ESIK, gazSayac, gazAlarm);
      alarmKontrol("duman", dumanDeger, DUMAN_ESIK, dumanSayac, dumanAlarm);
    }
  }

  if (simdi - sonOlcumGonderme >= OLCUM_GONDERME_MS) {
    sonOlcumGonderme = simdi;
    if (simdi < ISINMA_MS) {
      Serial.print("ISINIYOR kalan=");
      Serial.println((ISINMA_MS - simdi) / 1000);
    } else {
      Serial.print("OLCUM gaz=");
      Serial.print(gazDeger);
      Serial.print(" duman=");
      Serial.println(dumanDeger);
    }
  }

  cikislariGuncelle(simdi);
}
