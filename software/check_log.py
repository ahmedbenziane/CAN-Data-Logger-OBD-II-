import re

log_file = "capture.log"          # <-- change if your file has a different name

# Read all raw bytes
with open(log_file, "rb") as f:
    raw = f.read()

# Try to decode the binary mess into a single string
text = None
for enc in ["utf-8", "utf-16-le", "utf-16-be", "latin-1"]:
    try:
        text = raw.decode(enc)
        break
    except UnicodeDecodeError:
        continue

if text is None:
    print("❌ Could not decode the file. Try a different encoding or file.")
    exit()

# Find every candump frame: (12.345678) something 1A0#00FF...
pattern = re.compile(r"\((\d+\.\d+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)")

lines_out = 0
with open("pure_can.txt", "w") as out:
    for match in pattern.finditer(text):
        ts, can_id, data = match.groups()
        out.write(f"({ts}) vcan0 {can_id}#{data}\n")
        lines_out += 1

print(f"✅ Extracted {lines_out} candump lines → pure_can.txt")