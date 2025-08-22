# Volz RS-485 Tool (Tkinter GUI)

A small helper GUI to scan/change IDs, send position, and switch ACE1/ACE2 master on VOLZ servos.

## Features
- Open/close serial port, COM list & baud select
- Scan IDs (0x01..0x1E or broadcast 0x1F when only one device)
- Change ID (0xAA → 0x55, auto verify)
- Send Position (0x76 → 0x56) single-shot
- Force ACE2→Master / ACE1→Master, reset default role
- Hex TX/RX log, optional manual RTS for half-duplex

## Install
```bash
pip install -r requirements.txt
python volz_gui.py
