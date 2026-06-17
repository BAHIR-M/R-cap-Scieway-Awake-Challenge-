import time
import serial
import struct
import math

# --- Configuration ---
LIDAR_PORT = "COM11"
LIDAR_BAUD = 230400

ESP32_PORT = "COM7"
ESP32_BAUD = 115200

# --- Constantes du protocole LD19 ---
HEADER    = 0x54
VERLEN    = 0x2C
PKT_SIZE  = 47   # octets par paquet
NB_POINTS = 12   # points de mesure par paquet

CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
    0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
    0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
    0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
    0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
    0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
    0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
    0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
    0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
    0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
    0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
]

def calc_crc(data):
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc

def parse_packet(raw):
    """
    Analyse un paquet LD19 de 47 octets et retourne une liste de points (angle_rad, distance_mm).
    Retourne None si le paquet est invalide (mauvaise taille, en-tête ou CRC incorrect).

    Structure du paquet :
      [0]     Header     0x54
      [1]     VerLen     0x2C
      [2:4]   Speed      (u16 LE, deg/s)
      [4:6]   StartAngle (u16 LE, 0.01 deg)
      [6:42]  12 × (Distance u16 LE mm, Confidence u8)
      [42:44] EndAngle   (u16 LE, 0.01 deg)
      [44:46] Timestamp  (u16 LE, ms)
      [46]    CRC
    """
    if len(raw) != PKT_SIZE:
        return None
    if raw[0] != HEADER or raw[1] != VERLEN:
        return None
    if calc_crc(raw[:PKT_SIZE - 1]) != raw[PKT_SIZE - 1]:
        return None

    start_angle = struct.unpack_from("<H", raw, 4)[0] * 0.01   # degrés
    end_angle   = struct.unpack_from("<H", raw, 42)[0] * 0.01  # degrés

    if end_angle < start_angle:
        end_angle += 360.0
    step = (end_angle - start_angle) / (NB_POINTS - 1) if NB_POINTS > 1 else 0.0

    points = []
    for i in range(NB_POINTS):
        offset   = 6 + i * 3
        distance = struct.unpack_from("<H", raw, offset)[0]  # mm
        angle_deg = (start_angle + i * step) % 360.0
        angle_rad = math.radians(angle_deg)
        if 0 < distance < 2000:  # ignorer les mesures hors plage (> 2 m)
            points.append((angle_rad, distance))
    return points

def read_packet(ser):
    """
    Bloque jusqu'à la réception d'un paquet LD19 valide de 47 octets.
    Se synchronise sur la séquence Header 0x54 / VerLen 0x2C.
    """
    while True:
        byte = ser.read(1)
        if not byte:
            continue
        if byte[0] != HEADER:
            continue
        next_byte = ser.read(1)
        if not next_byte or next_byte[0] != VERLEN:
            continue
        rest = ser.read(PKT_SIZE - 2)
        if len(rest) < PKT_SIZE - 2:
            continue
        raw = bytes([HEADER, VERLEN]) + rest
        return raw

angles    = []
distances = []

def update(ser, esp):
    global angles, distances
    angles.clear()
    distances.clear()

    # Accumuler des paquets jusqu'à ce que chaque tranche de 10° dans [-90°, +90°] ait au moins 10 points,
    # garantissant une couverture complète de l'hémisphère avant avant l'analyse du gap.
    while True:
        raw = read_packet(ser)
        pts = parse_packet(raw)
        if pts:
            for a, d in pts:
                angles.append(a)
                distances.append(d)

        bin_coverage = {i: 0 for i in range(-90, 91, 10)}
        for a in angles:
            a_deg = round(math.degrees(a))
            a_deg = (a_deg - 360) if a_deg > 180 else a_deg
            for bin_start in bin_coverage:
                if bin_start <= a_deg < bin_start + 10:
                    bin_coverage[bin_start] += 1
                    break

        if all(count >= 10 for count in bin_coverage.values()):
            break

    # Recadrer les angles de [0, 2π] vers [-π, π]
    angles[:] = [a - 2*math.pi if a > math.pi else a for a in angles]

    # Garder uniquement la mesure la plus proche par degré entier dans [-90°, +90°]
    closest_dist_per_angle = {}
    for a, d in zip(angles, distances):
        a_deg = round(math.degrees(a))
        if -90 <= a_deg <= 90:
            if a_deg not in closest_dist_per_angle or d < closest_dist_per_angle[a_deg]:
                closest_dist_per_angle[a_deg] = d

    # Diviser [-90°, +90°] en 10 secteurs de 18° chacun
    sector_edges = [-90 + i * 18 for i in range(11)]

    sector_stats = []
    for i in range(len(sector_edges) - 1):
        a_min = sector_edges[i]
        a_max = sector_edges[i + 1]
        points_in_sector = [d for a_deg, d in closest_dist_per_angle.items()
                            if a_min <= a_deg < a_max]
        if points_in_sector:
            avg_dist = sum(points_in_sector) / len(points_in_sector)
            std_dist = math.sqrt(sum((d - avg_dist) ** 2 for d in points_in_sector) / len(points_in_sector))
            sector_stats.append((a_min, a_max, avg_dist, std_dist, len(points_in_sector)))

    if sector_stats:
        # Choisir le secteur avec la plus grande distance moyenne ; départager par le plus faible écart-type
        sector_stats.sort(key=lambda x: (x[2], -x[3]), reverse=True)
        best_sector = sector_stats[0]
        print(f"Gap: {best_sector[0]}° to {best_sector[1]}°  "
              f"avg={best_sector[2]:.1f} mm  std={best_sector[3]:.1f} mm  n={best_sector[4]}")
        gap_center_deg = (best_sector[0] + best_sector[1]) / 2
        # Mettre à l'échelle par 0.75 pour limiter la déviation servo à ±33.75° (évite la butée mécanique)
        esp.write(f"{0.75 * gap_center_deg},100\n".encode())
        print(f"-> sending: {0.75 * gap_center_deg:.1f}°, 100 rpm")
    else:
        print("No gap detected")


def main():
    ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=1)
    esp = serial.Serial(ESP32_PORT, ESP32_BAUD, dsrdtr=False, rtscts=False)
    print(f"LIDAR on {LIDAR_PORT} at {LIDAR_BAUD} baud")
    try:
        while True:
            update(ser, esp)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        ser.close()
        esp.close()
        print("Serial ports closed")

if __name__ == "__main__":
    main()
