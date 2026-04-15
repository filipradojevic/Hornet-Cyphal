from pyulog import ULog
import numpy as np
from scipy.signal import medfilt
import plotly.graph_objects as go
import csv
import sys
import os
import argparse

def main():
    # Argumenti komandne linije
    parser = argparse.ArgumentParser(description="Prikaži napon iz ULog fajla sa median filtriranjem i opcijom crtanja linija i/ili tačaka.")
    parser.add_argument("ulog_file", help="Putanja do .ulg fajla")
    parser.add_argument("--style", choices=["lines", "markers", "lines+markers"], default="lines+markers", help="Način crtanja: samo linije, samo tačke ili linije+tačke")
    parser.add_argument("--marker-size", type=int, default=3, help="Veličina markera kada su tačke uključene")
    parser.add_argument("--line-width", type=float, default=1.0, help="Debljina linije za originalni signal")
    parser.add_argument("--filtered-line-width", type=float, default=2.0, help="Debljina linije za filtrirani signal")
    parser.add_argument("--kernel-size", type=int, default=5, help="Kernel veličina za median filter (neparan broj)")
    parser.add_argument("--dt", type=float, default=0.001, help="Vremenski korak između uzoraka u sekundama (podrazumevano 0.001 s)")
    parser.add_argument("--no-plot", action="store_true", help="Ne prikazuj plotove, samo kreiraj CSV za PlotJuggler")
    parser.add_argument("--plotjuggler-only", action="store_true", help="Kreiraj samo CSV za PlotJuggler bez plotova")

    args = parser.parse_args()

    ulog_file = args.ulog_file

    if not os.path.exists(ulog_file):
        print(f"Greška: fajl '{ulog_file}' ne postoji.")
        sys.exit(1)

    print(f"[INFO] Učitavam ULog fajl: {ulog_file}")
    ulog = ULog(ulog_file)

    voltage = None
    topic_name = None

    # Tražimo poruku koja sadrži 'voltage'
    for msg in ulog.data_list:
        if 'voltage' in msg.data:
            topic_name = msg.name
            voltage = np.array(msg.data['voltage'])
            break

    if voltage is None:
        print("Nije pronađeno polje 'voltage' u ULog fajlu.")
        sys.exit(1)

    print(f"[INFO] Pronađena poruka: {topic_name}")
    print(f"[INFO] Broj uzoraka: {len(voltage)}")

    # Median filter (kernel_size mora biti neparan broj)
    kernel_size = args.kernel_size if args.kernel_size % 2 == 1 else max(3, args.kernel_size + 1)
    filtered_voltage = medfilt(voltage, kernel_size=kernel_size)
    print(f"[INFO] Primenjen median filter (kernel_size={kernel_size})")

    # Generiši timestamp sa podesivim korakom dt
    N = len(voltage)
    timestamp_s = np.arange(N) * args.dt

    # Figura 1: samo tačke (markers)
    fig_markers = go.Figure()
    fig_markers.add_trace(go.Scatter(
        x=timestamp_s,
        y=voltage,
        mode='markers',
        name='Original (tačke)',
        marker=dict(color='blue', size=args.marker_size)
    ))
    fig_markers.add_trace(go.Scatter(
        x=timestamp_s,
        y=filtered_voltage,
        mode='markers',
        name='Filtrirani (tačke)',
        marker=dict(color='red', size=args.marker_size)
    ))
    fig_markers.update_layout(
        title=f"Napon (tačke) – kernel={kernel_size}",
        xaxis_title="Vreme [s]",
        yaxis_title="Napon [V]",
        template="plotly_white"
    )

    # Figura 2: samo linije
    fig_lines = go.Figure()
    fig_lines.add_trace(go.Scatter(
        x=timestamp_s,
        y=voltage,
        mode='lines',
        name='Original (linije)',
        line=dict(color='blue', width=args.line_width)
    ))
    fig_lines.add_trace(go.Scatter(
        x=timestamp_s,
        y=filtered_voltage,
        mode='lines',
        name='Filtrirani (linije)',
        line=dict(color='red', width=args.filtered_line_width)
    ))
    fig_lines.update_layout(
        title=f"Napon (linije) – kernel={kernel_size}",
        xaxis_title="Vreme [s]",
        yaxis_title="Napon [V]",
        template="plotly_white"
    )

    # Prikaži plotove samo ako nije --no-plot ili --plotjuggler-only
    if not args.no_plot and not args.plotjuggler_only:
        fig_markers.show()
        fig_lines.show()
    elif args.plotjuggler_only:
        print("[INFO] PlotJuggler mode - plotovi nisu prikazani")

    # Ekspresuj u CSV fajl za PlotJuggler
    csv_name = ulog_file.replace('.ulg', '_for_plotjuggler.csv')
    with open(csv_name, "w", newline="") as f:
        writer = csv.writer(f)
        # PlotJuggler header format
        writer.writerow(["timestamp", "voltage_original", "voltage_filtered", "voltage_diff"])
        # Izračunaj razliku između originalnog i filtriranog
        voltage_diff = voltage - filtered_voltage
        writer.writerows(zip(timestamp_s, voltage, filtered_voltage, voltage_diff))

    print(f"[INFO] CSV fajl za PlotJuggler kreiran: '{csv_name}'")
    print(f"[INFO] Otvorite fajl u PlotJuggler-u za lepše plotovanje!")
    print(f"[INFO] Dostupni signali: timestamp, voltage_original, voltage_filtered, voltage_diff")

if __name__ == "__main__":
    main()
