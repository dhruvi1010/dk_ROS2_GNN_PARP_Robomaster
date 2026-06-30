#!/usr/bin/env python3
import subprocess
import sys
import time
import re
from datetime import datetime

# ============================================================
# AT-BEFEHL SENDEN
# ============================================================

def send_at_command(cmd):
    """Sendet AT-Befehl via mmcli mit sudo"""
    try:
        result = subprocess.run(
            ['sudo', 'mmcli', '-m', '0', f'--command={cmd}'],
            capture_output=True,
            text=True,
            timeout=5
        )
        output = result.stdout + result.stderr
        if 'response:' in output:
            return output.split('response:', 1)[1].strip()
        return output
    except subprocess.TimeoutExpired:
        return ""
    except Exception as e:
        return ""

# ============================================================
# PARSER FUNKTIONEN
# ============================================================

def _parse_quoted_csv(content: str) -> list:
    """Parst CSV mit Anführungszeichen."""
    parts = []
    current = ""
    in_quotes = False
    
    for char in content:
        if char == '"':
            in_quotes = not in_quotes
        elif char == ',' and not in_quotes:
            parts.append(current.strip().strip('"'))
            current = ""
            continue
        current += char
    
    if current:
        parts.append(current.strip().strip('"'))
    
    return parts


def _parse_antenna_values(content: str, antenna_data: dict, metric: str) -> None:
    """Parst Antennenwerte (PRX, DRX, RX2, RX3)."""
    parts = content.strip().split(',')
    antennas = ['PRX', 'DRX', 'RX2', 'RX3']
    
    for i, ant in enumerate(antennas):
        if i < len(parts):
            antenna_data[ant][metric] = parts[i].strip()


def parse_modem_responses(responses: dict) -> dict:
    """
    Parst alle AT-Befehlsausgaben und gibt strukturierte Daten zurück.
    """
    
    # Datencontainer initialisieren
    signal_quality = {
        'RSRP': None, 'RSRQ': None, 'SINR': None,
        'MCS': None, 'RI': None, 'CQI': None, 'PMI': None
    }
    
    antenna_data = {
        'PRX': {'RSRP': None, 'RSRQ': None, 'SINR': None},
        'DRX': {'RSRP': None, 'RSRQ': None, 'SINR': None},
        'RX2': {'RSRP': None, 'RSRQ': None, 'SINR': None},
        'RX3': {'RSRP': None, 'RSRQ': None, 'SINR': None}
    }
    
    temperatures = {}
    
    # Alle Responses zusammenfügen und zeilenweise parsen
    all_data = "\n".join(responses.values())
    
    for line in all_data.split('\n'):
        line = line.strip().strip("'")
        if not line:
            continue
        
        # +QENG parsen (Serving Cell Info)
        if line.startswith('+QENG:'):
            parts = _parse_quoted_csv(line[6:].strip())
            if len(parts) > 14 and parts[0] == 'servingcell':
                signal_quality['RSRP'] = parts[12]
                signal_quality['RSRQ'] = parts[13]
                signal_quality['SINR'] = parts[14]
        
        # +QNWCFG parsen (CSI Info: MCS, RI, CQI, PMI)
        elif line.startswith('+QNWCFG:'):
            parts = [p.strip().strip('"') for p in line[9:].split(',')]
            if parts[0] in ['nr5g_csi', 'lte_csi'] and len(parts) >= 5:
                signal_quality['MCS'] = parts[1]
                signal_quality['RI'] = parts[2]
                signal_quality['CQI'] = parts[3]
                signal_quality['PMI'] = parts[4]
        
        # +QRSRP parsen
        elif line.startswith('+QRSRP:'):
            _parse_antenna_values(line[8:], antenna_data, 'RSRP')
        
        # +QRSRQ parsen
        elif line.startswith('+QRSRQ:'):
            _parse_antenna_values(line[8:], antenna_data, 'RSRQ')
        
        # +QSINR parsen
        elif line.startswith('+QSINR:'):
            _parse_antenna_values(line[8:], antenna_data, 'SINR')
        
        # +QTEMP parsen
        elif line.startswith('+QTEMP:'):
            match = re.match(r'\+QTEMP:"([^"]+)","(\d+)"', line)
            if match:
                temperatures[match.group(1)] = match.group(2)
    
    return {
        'signal_quality': signal_quality,
        'antenna_data': antenna_data,
        'temperatures': temperatures
    }

# ============================================================
# AUSGABE FUNKTIONEN
# ============================================================

def print_tables(data: dict) -> None:
    """Gibt alle drei Tabellen formatiert aus."""
    
    signal_quality = data['signal_quality']
    antenna_data = data['antenna_data']
    temperatures = data['temperatures']
    
    # Tabelle 1: Signalqualität (horizontal)
    print("=" * 70)
    print("  Tabelle 1: Signalqualität")
    print("=" * 70)
    
    params = ['RSRP', 'RSRQ', 'SINR', 'MCS', 'RI', 'CQI', 'PMI']
    col_width = 8
    
    # Header-Zeile
    header = "|"
    separator = "|"
    for param in params:
        header += f" {param:^{col_width}} |"
        separator += "-" * (col_width + 2) + "|"
    
    print(header)
    print(separator)
    
    # Datenzeile
    data_row = "|"
    for param in params:
        val = signal_quality.get(param) or 'N/A'
        data_row += f" {val:^{col_width}} |"
    
    print(data_row)
    
    # Tabelle 2: Antennenwerte
    antenna_labels = {
        'PRX': 'PRX (Primary RX)',
        'DRX': 'DRX (Diversity RX)',
        'RX2': 'RX2 (MIMO-RX2)',
        'RX3': 'RX3 (MIMO-RX3)'
    }
    
    print("\n" + "=" * 60)
    print("  Tabelle 2: Antennenwerte")
    print("=" * 60)
    print(f"| {'Antenne':<22} | {'RSRP':<8} | {'RSRQ':<8} | {'SINR':<8} |")
    print(f"|{'-' * 24}|{'-' * 10}|{'-' * 10}|{'-' * 10}|")
    
    for ant in ['PRX', 'DRX', 'RX2', 'RX3']:
        rsrp = antenna_data[ant]['RSRP'] or 'N/A'
        rsrq = antenna_data[ant]['RSRQ'] or 'N/A'
        sinr = antenna_data[ant]['SINR'] or 'N/A'
        print(f"| {antenna_labels[ant]:<22} | {rsrp:<8} | {rsrq:<8} | {sinr:<8} |")
    
    # Tabelle 3: Temperaturen
    print("\n" + "=" * 45)
    print("  Tabelle 3: Temperaturen")
    print("=" * 45)
    print(f"| {'Sensor':<25} | {'Temp (°C)':<10} |")
    print(f"|{'-' * 27}|{'-' * 12}|")
    
    if temperatures:
        for sensor, temp in temperatures.items():
            print(f"| {sensor:<25} | {temp:<10} |")
    else:
        print(f"| {'Keine Daten':<25} | {'-':<10} |")


def clear_screen():
    """Bildschirm löschen (ANSI-Escape)"""
    print("\033[2J\033[H", end="", flush=True)

# ============================================================
# HAUPTSCHLEIFE
# ============================================================

def send_all_commands_loop(interval=5):
    """Sendet AT-Befehle in Dauerschleife"""
    
    commands = {
        'servingcell': 'AT+QENG="servingcell"',
        'csi': 'AT+QNWCFG="nr5g_csi"',
        'rsrq': 'AT+QRSRQ',
        'rsrp': 'AT+QRSRP',
        'sinr': 'AT+QSINR',
        'temp': 'AT+QTEMP',
    }
    
    iteration = 0
    
    try:
        while True:
            # Alle Befehle senden
            responses = {}
            for key, cmd in commands.items():
                responses[key] = send_at_command(cmd)
            
            # Daten parsen
            data = parse_modem_responses(responses)
            
            # Bildschirm leeren
            clear_screen()
            
            iteration += 1
            loop_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # Header
            print("=" * 70)
            print(f"  5G Modem Monitor | Iteration: {iteration} | {loop_time}")
            print("=" * 70)
            print(f"  Intervall: {interval}s | Beenden: Ctrl+C\n")
            
            # Tabellen ausgeben
            print_tables(data)
            
            # Warten
            #time.sleep(interval)
            
    except KeyboardInterrupt:
        print(f"\n\n{'='*70}")
        print("✓ Beendet durch Benutzer (Ctrl+C)")
        print(f"{'='*70}\n")


if __name__ == "__main__":
    try:
        send_all_commands_loop(interval=0.2)
    except Exception as e:
        print(f"✗ Fehler: {e}")
        sys.exit(1)
