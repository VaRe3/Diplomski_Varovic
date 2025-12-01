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


