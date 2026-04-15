# ULog Sample Analysis

Ovaj direktorijum sadrži primere ULog fajlova i skripte za njihovu analizu.

## Dostupni fajlovi
- `motor.ulg` - primer ULog fajla sa podacima o motoru
- `motor2.ulg` - drugi primer ULog fajla
- `plot.py` - skripta za plotovanje podataka iz ULog fajlova

## Uputstvo za korišćenje

### 1. Osnovni plot podataka
```bash
cd src/samples  
python plot.py motor.ulg
```

### 2. Filtriranje voltage podataka (median filter)
```bash
cd BlackBoxESP-main  
python copy_filtered_ulog.py src/samples/motor.ulg
```

Ova komanda će kreirati `motor_filtered.ulg` fajl sa filtriranim voltage podacima.

### 3. Zahtevi
- Python 3.x
- pyulog library: `pip install pyulog`
- scipy library: `pip install scipy`
- matplotlib library: `pip install matplotlib`

## Napomene
- Filtrirani ULog fajlovi su kompatibilni sa PlotJuggler-om
- Median filter koristi kernel_size=5 za redukovanje šuma
- Originalni fajlovi ostaju nepromenjeni