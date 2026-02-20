# Test du bootloader CAN-FD depuis une Raspberry Pi

## Setup hardware

```
Raspberry Pi (MCP2518FD / MCP251xFD)  <--CAN-->  STM32G474RE (FDCAN)
```

- Interface CAN sur la Pi : `can0` (via SPI, ex: MCP2518FD hat)
- Le bus doit avoir une terminaison 120 ohm aux deux extrémités

## Configuration de l'interface CAN

```bash
# CAN classique 500kbit/s
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# CAN-FD : 500kbit/s arbitration, 2Mbit/s data
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can0 up
```

## Identifiants CAN

Chaque carte a un `NODE_ID` unique (0–127), configuré au build dans le Makefile de l'application :

```makefile
# Dans le Makefile de l'application
RIOTBOOT_CAN_NODE_ID ?= 3
CFLAGS += -DRIOTBOOT_CAN_NODE_ID=$(RIOTBOOT_CAN_NODE_ID)

# Utiliser le bootloader CAN (au lieu du riotboot par défaut)
RIOTBOOT_DIR ?= $(RIOTBASE)/bootloaders/riotboot_can
```

Le `RIOTBOOT_CAN_NODE_ID` est automatiquement propagé au bootloader via le build system
(`riotboot.mk` le passe dans l'environnement du sous-make).

Les CAN IDs sont dérivés automatiquement :

```
CMD  = 0x100 + 2 * NODE_ID    (Pi -> carte)
RESP = 0x100 + 2 * NODE_ID + 1  (carte -> Pi)
```

| NODE_ID | CMD     | RESP    | Description          |
|---------|---------|---------|----------------------|
| 0       | `0x100` | `0x101` | Carte par défaut     |
| 1       | `0x102` | `0x103` | Carte 1              |
| 2       | `0x104` | `0x105` | Carte 2              |
| 3       | `0x106` | `0x107` | Carte 3              |
| ...     | ...     | ...     | ...                  |

## Protocole

### Format des frames

**Sync / Boot** (pas de CRC) :
```
[type(1)]              ex: PROBE, ENTER_LOADER
[type(1) | slot(1)]    ex: BOOT + '0' ou '\n'
```

**Commandes avec données** :
```
[type(1) | len(1) | data(len) | crc8(1)]
```

CRC-8 calculé sur `[type, len, data...]` avec polynome `0x31`, init `0xFF`.

### Codes

| Hex    | Char | Nom             | Direction |
|--------|------|-----------------|-----------|
| `0x3F` | `?`  | PROBE           | CMD       |
| `0x42` | `B`  | ENTER_LOADER    | CMD       |
| `0x62` | `b`  | CMD_BOOT        | CMD       |
| `0x65` | `e`  | CMD_ERASE       | CMD       |
| `0x77` | `w`  | CMD_WRITE       | CMD       |
| `0x50` | `P`  | CMD_GET_PAGE    | CMD       |
| `0x2E` | `.`  | STAT_OK         | RESP      |
| `0x3F` | `?`  | STAT_BAD_CRC    | RESP      |
| `0x21` | `!`  | STAT_ILLEGAL    | RESP      |
| `0x62` | `b`  | STAT_WAITING    | RESP      |
| `0x3E` | `>`  | STAT_READY      | RESP      |

## Outils

### Écouter les réponses

```bash
# Terminal dédié - laisser tourner pendant les tests
# Pour NODE_ID=0 (RESP=0x101) :
candump can0,101:7FF

# Pour NODE_ID=3 (RESP=0x107) :
candump can0,107:7FF

# Écouter TOUTES les réponses bootloader (0x101, 0x103, 0x105, ...) :
candump can0,101:001
```

### Calcul du CRC-8

Script Python pour calculer le CRC :

```python
#!/usr/bin/env python3
"""Calcul CRC-8 pour le protocole riotboot CAN."""

def crc8(data: bytes, poly: int = 0x31, init: int = 0xFF) -> int:
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

# Exemples :
# GET_PAGE pour addr 0x08004000
data = bytes([0x50, 0x04, 0x00, 0x40, 0x00, 0x08])  # type='P', len=4, addr LE
print(f"CRC GET_PAGE: 0x{crc8(data):02X}")

# ERASE page 16
data = bytes([0x65, 0x02, 0x10, 0x00])  # type='e', len=2, page=16 LE
print(f"CRC ERASE p16: 0x{crc8(data):02X}")
```

## Scénarios de test

Les exemples ci-dessous utilisent NODE_ID=0 (CMD=0x100, RESP=0x101).
Pour une autre carte, remplacer les IDs selon la table ci-dessus.
Ex: NODE_ID=3 -> CMD=0x106, RESP=0x107.

### 1. Vérifier la présence du bootloader

Au boot (dans les 2 premières secondes), le bootloader envoie `STAT_WAITING` (`0x62`) sur son ID RESP.

```bash
# Écouter les messages du bootloader au reset (NODE_ID=0)
candump can0,101:7FF
# => Attendre le reset de la carte
# => Doit afficher : can0  101   [1]  62    (= 'b' = WAITING)
```

### 2. Entrer dans le bootloader

Envoyer `ENTER_LOADER` (`0x42`) sur l'ID CMD de la carte pendant la phase WAITING (dans les 2s après reset) :

```bash
# Envoyer ENTER_LOADER (NODE_ID=0, CMD=0x100)
cansend can0 100#42
# => Réponse attendue sur 0x101 : 3E (= '>' = READY)

# Pour NODE_ID=3 (CMD=0x106) :
cansend can0 106#42
```

### 3. Probe

```bash
cansend can0 100#3F
# => Réponse : 62 (WAITING) si en bootdelay, ou 62 si déjà en mode loader
```

### 4. GET_PAGE - Obtenir le numéro de page d'une adresse

Format : `[0x50, len, addr_le(4B), crc8]`

Exemple pour l'adresse `0x08004000` (début du slot 0, RIOTBOOT_LEN = 0x4000) :

```bash
# Calculer le CRC d'abord avec le script Python
# data = [0x50, 0x04, 0x00, 0x40, 0x00, 0x08]
# => addr en little-endian : 0x08004000 -> 00 40 00 08

python3 -c "
def crc8(d, p=0x31, i=0xFF):
    c = i
    for b in d:
        c ^= b
        for _ in range(8): c = ((c<<1)^p)&0xFF if c&0x80 else (c<<1)&0xFF
    return c
data = bytes([0x50, 0x04, 0x00, 0x40, 0x00, 0x08])
print(f'{crc8(data):02X}')
"
# => Donne le CRC, ex: XX

# Envoyer la commande (remplacer XX par le CRC calculé)
cansend can0 100#50.04.00.40.00.08.XX
# => Réponse sur 0x101 : 2E PP PP PP PP (OK + page number en LE 32bit)
```

### 5. ERASE - Effacer une page

Format : `[0x65, 0x02, page_le(2B), crc8]`

```bash
# Effacer la page 16 (0x0010 en LE = 10 00)
python3 -c "
def crc8(d, p=0x31, i=0xFF):
    c = i
    for b in d:
        c ^= b
        for _ in range(8): c = ((c<<1)^p)&0xFF if c&0x80 else (c<<1)&0xFF
    return c
data = bytes([0x65, 0x02, 0x10, 0x00])
print(f'{crc8(data):02X}')
"

cansend can0 100#65.02.10.00.XX
# => Réponse : 2E (OK) ou 21 (ILLEGAL si page protégée)
```

### 6. WRITE - Écrire des données

Format : `[0x77, len, addr_le(4B), data(n), crc8]`

Le `len` couvre `addr(4) + data(n)`. En CAN-FD, on peut envoyer jusqu'à 64 bytes par frame, donc `n` max = 64 - 4 (addr) - 2 (type+len) - 1 (crc) = **57 bytes** de payload par frame.

```bash
# Écrire 4 bytes (AA BB CC DD) à l'adresse 0x08004000
# len = 4 (addr) + 4 (data) = 8
python3 -c "
def crc8(d, p=0x31, i=0xFF):
    c = i
    for b in d:
        c ^= b
        for _ in range(8): c = ((c<<1)^p)&0xFF if c&0x80 else (c<<1)&0xFF
    return c
data = bytes([0x77, 0x08, 0x00, 0x40, 0x00, 0x08, 0xAA, 0xBB, 0xCC, 0xDD])
print(f'{crc8(data):02X}')
"

cansend can0 100#77.08.00.40.00.08.AA.BB.CC.DD.XX
# => Réponse : 2E (OK)
```

### 7. BOOT - Démarrer un slot

```bash
# Boot le slot par défaut (meilleure version)
cansend can0 100#62.0A
# 0x62 = 'b', 0x0A = '\n'
# => Réponse : 2E (OK), puis le bootloader saute vers l'application

# Boot le slot 0
cansend can0 100#62.30
# 0x62 = 'b', 0x30 = '0'

# Boot le slot 1
cansend can0 100#62.31
# 0x62 = 'b', 0x31 = '1'
```

## Séquence complète de flash

```bash
# Variables selon la carte cible
NODE_ID=3                           # NODE_ID de la carte
CMD_ID=$(printf '%03X' $((0x100 + 2 * NODE_ID)))    # => 106
RESP_ID=$(printf '%03X' $((0x100 + 2 * NODE_ID + 1))) # => 107

# 1. Écouter les réponses dans un terminal séparé
candump can0,${RESP_ID}:7FF &

# 2. Reset la carte et entrer dans le bootloader
cansend can0 ${CMD_ID}#42          # ENTER_LOADER
# Attendre réponse '>' (0x3E)

# 3. Pour chaque page du firmware :
#    a. GET_PAGE de l'adresse de début
#    b. ERASE la page
#    c. WRITE par blocs (max 57 bytes par frame CAN-FD)

# 4. Booter
cansend can0 ${CMD_ID}#62.0A       # BOOT default
```

## Script Python complet

```python
#!/usr/bin/env python3
"""Flash firmware via riotboot CAN-FD bootloader."""

import argparse
import can
import struct
import sys

CAN_BASE_ID = 0x100
CRC8_POLY   = 0x31
CRC8_INIT   = 0xFF

def crc8(data: bytes) -> int:
    crc = CRC8_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

class RiotbootCAN:
    def __init__(self, node_id: int = 0, interface='can0', is_fd=True):
        self.id_cmd  = CAN_BASE_ID + 2 * node_id
        self.id_resp = CAN_BASE_ID + 2 * node_id + 1
        self.bus = can.Bus(channel=interface, interface='socketcan',
                           fd=is_fd, bitrate=500000, data_bitrate=2000000)
        print(f"Node {node_id}: CMD=0x{self.id_cmd:03X} RESP=0x{self.id_resp:03X}")

    def send(self, data: bytes):
        msg = can.Message(arbitration_id=self.id_cmd, data=data, is_fd=True)
        self.bus.send(msg)

    def recv(self, timeout=2.0) -> can.Message:
        while True:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                raise TimeoutError("No response from bootloader")
            if msg.arbitration_id == self.id_resp:
                return msg

    def enter_bootloader(self, retries=20):
        """Wait for WAITING and send ENTER_LOADER."""
        for _ in range(retries):
            self.send(bytes([0x42]))  # ENTER_LOADER
            try:
                resp = self.recv(timeout=0.2)
                if resp.data[0] == 0x3E:  # READY
                    print("Bootloader ready")
                    return
                elif resp.data[0] == 0x62:  # WAITING
                    continue
            except TimeoutError:
                continue
        raise RuntimeError("Failed to enter bootloader")

    def get_page(self, addr: int) -> int:
        payload = bytes([0x50, 0x04]) + struct.pack('<I', addr)
        payload += bytes([crc8(payload)])
        self.send(payload)
        resp = self.recv()
        assert resp.data[0] == 0x2E, f"GET_PAGE failed: {resp.data[0]:#x}"
        return struct.unpack('<I', resp.data[1:5])[0]

    def erase(self, page: int):
        payload = bytes([0x65, 0x02]) + struct.pack('<H', page)
        payload += bytes([crc8(payload)])
        self.send(payload)
        resp = self.recv()
        assert resp.data[0] == 0x2E, f"ERASE failed: {resp.data[0]:#x}"

    def write(self, addr: int, data: bytes):
        """Write data in chunks of max 57 bytes (CAN-FD limit)."""
        MAX_CHUNK = 57  # 64 - 1(type) - 1(len) - 4(addr) - 1(crc)
        offset = 0
        while offset < len(data):
            chunk = data[offset:offset + MAX_CHUNK]
            length = 4 + len(chunk)  # addr + data
            payload = bytes([0x77, length])
            payload += struct.pack('<I', addr + offset)
            payload += chunk
            payload += bytes([crc8(payload)])
            self.send(payload)
            resp = self.recv()
            assert resp.data[0] == 0x2E, f"WRITE failed at {addr+offset:#x}: {resp.data[0]:#x}"
            offset += len(chunk)

    def boot(self, slot=None):
        if slot is None:
            self.send(bytes([0x62, 0x0A]))  # '\n' = default
        else:
            self.send(bytes([0x62, 0x30 + slot]))  # '0' or '1'
        resp = self.recv()
        assert resp.data[0] == 0x2E, f"BOOT failed: {resp.data[0]:#x}"

    def flash(self, firmware_path: str, base_addr: int):
        with open(firmware_path, 'rb') as f:
            firmware = f.read()

        print(f"Firmware size: {len(firmware)} bytes")

        start_page = self.get_page(base_addr)
        if len(firmware) == 0:
            return

        end_page = self.get_page(base_addr + len(firmware) - 1)
        print(f"Pages {start_page} to {end_page}")

        for page in range(start_page, end_page + 1):
            print(f"  Erasing page {page}...")
            self.erase(page)

        print(f"  Writing {len(firmware)} bytes...")
        self.write(base_addr, firmware)
        print("Done!")

def main():
    parser = argparse.ArgumentParser(description="Flash firmware via riotboot CAN-FD")
    parser.add_argument("firmware", help="Path to firmware .bin file")
    parser.add_argument("-n", "--node-id", type=int, default=0,
                        help="Target node ID (default: 0)")
    parser.add_argument("-a", "--base-addr", default="0x08004000",
                        help="Base address in hex (default: 0x08004000)")
    parser.add_argument("-i", "--interface", default="can0",
                        help="CAN interface (default: can0)")
    parser.add_argument("-s", "--slot", type=int, default=None,
                        help="Boot slot (0 or 1, default: best version)")
    args = parser.parse_args()

    base_addr = int(args.base_addr, 16)

    loader = RiotbootCAN(node_id=args.node_id, interface=args.interface)
    print("Waiting for bootloader...")
    loader.enter_bootloader()
    loader.flash(args.firmware, base_addr)
    print("Booting...")
    loader.boot(slot=args.slot)

if __name__ == '__main__':
    main()
```

Usage :
```bash
# Flasher la carte avec NODE_ID=0 (défaut)
python3 riotboot_can_flash.py firmware.bin

# Flasher la carte avec NODE_ID=3
python3 riotboot_can_flash.py firmware.bin --node-id 3

# Flasher avec options complètes
python3 riotboot_can_flash.py firmware.bin -n 3 -a 0x08004000 -i can0 -s 0
```

## Dépannage

| Symptome | Cause probable | Solution |
|----------|----------------|----------|
| Pas de message `0x101` au reset | CAN non initialisé ou bitrate incorrect | Vérifier bitrate, terminaison, câblage |
| `candump` vide après `cansend` | Filtre CAN mal configuré ou carte pas en bootloader | Reset la carte, envoyer rapidement `0x42` |
| Réponse `0x21` (ILLEGAL) | Adresse protégée ou paramètre invalide | Vérifier que l'adresse est >= SLOT0_OFFSET |
| Réponse `0x3F` (BAD_CRC) | CRC incorrect | Recalculer le CRC avec le script Python |
| Timeout 2s puis boot auto | `ENTER_LOADER` envoyé trop tard | Envoyer `0x42` en boucle avant le reset |
| `cansend: write: No buffer space` | Queue CAN pleine | Ajouter un délai entre les envois |
