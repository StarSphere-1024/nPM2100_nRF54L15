import argparse
import struct
import wave

import serial


def main(port, output):
    raw_sound = []

    with serial.Serial(port, 115200, timeout=1) as ser:
        ser.write(b'mic capture 5\r')
        ser.readline()
        
        start = False
        finished = False
        while not finished:
            line = ser.readline().decode().strip()
            if line == "S":
                start = True
                continue
            elif "E" in line:
                finished = True
            else:
                if not start:
                    continue
                try:
                    raw_sound.append(int(line))
                except ValueError:
                    print(f"Invalid sample {line}")
    with wave.open(output, "w") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(16000)

        for value in raw_sound:
            if value > 32767 or value < -32768:
                print(f"Value out of range: {value}")
                continue
            data = struct.pack('<h', value)
            wav.writeframesraw(data)
 


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", required=True, help="Serial port")
    parser.add_argument("-o", "--output", required=True, help="Output file")
    args = parser.parse_args()

    main(args.port, args.output)