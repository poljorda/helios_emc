# HELIOS_EMC
Python GUI tool that will be used conduct the EMC tests on the HELIOS BMS system

## Virtual CAN channels
First install KVaser drivers: https://www.kvaser.com/download/

## Creating the virtual environment
Please use Python 3.12.0 or higher

### Windows
```bash
python -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -e .
python -m pip install -r requirements.txt
```

### Linux
```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -e .
python3 -m pip install -r requirements.txt