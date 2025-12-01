# SplitScore - Padel Scoreboard sustav

Funkcionalni prototip sustava za praćenje i prikaz rezultata padel mečeva koji povezuje fizičke ESP32 uređaje s web aplikacijom.

## 🏗️ Arhitektura

Sustav se sastoji od tri glavna dijela:

1. **ESP32 Kontroler** - 4 tipkala za upravljanje rezultatom (ESP-NOW komunikacija)
2. **ESP32 Scoreboard** - Prima komande, izračunava rezultat, šalje podatke na web (ESP-NOW + WiFi/HTTP)
3. **Next.js Web Aplikacija** - Real-time prikaz rezultata, povijest mečeva, osnovne stranice

## 🚀 Pokretanje Web Aplikacije

### Preduvjeti
- Node.js 18+ i npm/yarn
- Git (opcionalno)

### Instalacija i pokretanje

```bash
# Instaliraj dependencies
npm install

# Pokreni development server
npm run dev
```

Aplikacija će biti dostupna na `http://localhost:3000`

### Build za produkciju

```bash
# Kreiraj production build
npm run build

# Pokreni production server
npm start
```

## 📱 Stranice aplikacije

- `/` - Početna stranica s loginom i "Običan meč" gumbom
- `/match` - Real-time prikaz rezultata (osvježava svakih 500ms)
- `/history` - Povijest mečeva (mock podaci)
- `/challenge` - Izazov (skeleton)
- `/tournament` - Turnir (skeleton)
- `/points` - Bodovi (skeleton)

## 🔌 API Endpoints

### `POST /api/score`
Prima rezultat od ESP32 scoreboarda.

**Request body:**
```json
{
  "set": 1,
  "gem": 3,
  "plavi": 40,
  "crveni": 15
}
```

### `GET /api/score`
Vraća zadnji spremljeni rezultat.

**Response:**
```json
{
  "score": {
    "set": 1,
    "gem": 3,
    "plavi": 40,
    "crveni": 15,
    "timestamp": 1234567890
  }
}
```

## 🔧 ESP32 Postavljanje

### Kontroler (controller.ino)

1. Otvori `esp32/controller/controller.ino` u Arduino IDE
2. Instaliraj ESP32 board support (ESP32 by Espressif Systems)
3. **VAŽNO:** Promijeni MAC adresu scoreboarda u kodu:
   ```cpp
   uint8_t scoreboardAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
   ```
   Zamijeni s stvarnom MAC adresom scoreboarda
4. Uploadaj kod na ESP32 kontroler
5. Poveži tipkala:
   - GPIO 0 (Plavo) - poen za plavi tim
   - GPIO 2 (Crveno) - poen za crveni tim
   - GPIO 4 (Žuto) - undo
   - GPIO 5 (Zeleno) - finish

### Scoreboard (scoreboard.ino)

1. Otvori `esp32/scoreboard/scoreboard.ino` u Arduino IDE
2. **VAŽNO:** Promijeni konfiguraciju:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   const char* serverUrl = "http://192.168.1.100:3000/api/score";
   ```
   - Postavi WiFi SSID i lozinku
   - Postavi IP adresu računala gdje radi Next.js server
3. Uploadaj kod na ESP32 scoreboard
4. Zabilježi MAC adresu scoreboarda (prikazuje se u Serial Monitoru)
5. Unesi tu MAC adresu u controller kod

### ESP32 Libraries

Potrebne biblioteke (uglavnom uključene u ESP32 core):
- `WiFi.h` - WiFi komunikacija
- `HTTPClient.h` - HTTP zahtjevi
- `esp_now.h` - ESP-NOW komunikacija

## 📊 Komunikacija

### ESP-NOW (Kontroler ↔ Scoreboard)
- Kontroler šalje komande: `point_blue`, `point_red`, `undo`, `finish`
- Scoreboard prima komande i obrađuje rezultat

### HTTP (Scoreboard ↔ Web)
- Scoreboard šalje POST zahtjev na `/api/score` svaki put kada se rezultat promijeni
- Web aplikacija dohvaća rezultat svakih 500ms putem GET zahtjeva

## 🎮 Korištenje

1. **Pokreni web aplikaciju** (`npm run dev`)
2. **Prijavi se** na početnoj stranici (odaberi mock igrača)
3. **Klikni "Običan meč"** za prikaz rezultata
4. **Postavi ESP32 uređaje:**
   - Scoreboard se povezuje na WiFi i šalje podatke
   - Kontroler šalje komande preko ESP-NOW
5. **Pritisni tipke na kontroleru** - rezultat se automatski prikazuje na web stranici

## 🧪 Testiranje bez ESP32

Možeš testirati web aplikaciju bez ESP32 uređaja koristeći curl ili Postman:

```bash
curl -X POST http://localhost:3000/api/score \
  -H "Content-Type: application/json" \
  -d '{"set":1,"gem":3,"plavi":40,"crveni":15}'
```

Zatim otvori `/match` stranicu i vidi rezultat.

## 📝 Napomene

- Rezultati se trenutno čuvaju u memoriji (in-memory storage)
- Za produkciju bi trebalo dodati bazu podataka
- Mock login koristi localStorage
- ESP32 kod koristi jednostavnu logiku za padel scoring
- Za kompletnu padel logiku (tie-break, advantage, itd.) bi trebalo proširiti kod

## 🛠️ Tehnologije

- **Frontend/Backend:** Next.js 14, React, TypeScript
- **Styling:** Tailwind CSS
- **ESP32:** Arduino Core, C++
- **Komunikacija:** ESP-NOW, WiFi, HTTP

## 📄 Licenca

Projekt za diplomski rad.

