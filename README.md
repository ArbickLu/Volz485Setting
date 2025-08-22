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
pip install pyserial>=3.5
python volz_gui.py

# Optional: build tool for EXE packaging
pip install pyinstaller>=6
pyinstaller -w volz_gui.py --name VolzRS485Tool ^
  --hidden-import=serial.tools.list_ports --hidden-import=serial.tools.list_ports_windows

