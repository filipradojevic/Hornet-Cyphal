import sys
import os
import re

bin_path = sys.argv[1]
header_path = sys.argv[2]

size = os.path.getsize(bin_path)
esp_size = size - 64

with open(header_path, 'r') as f:
    content = f.read()

content = re.sub(
    r'#define FW_IMAGE_SIZE_WITH_HEADER \(\d+U\)',
    f'#define FW_IMAGE_SIZE_WITH_HEADER ({size}U)',
    content
)
content = re.sub(
    r'#define FW_IMAGE_SIZE \(\d+U\)',
    f'#define FW_IMAGE_SIZE ({esp_size}U)',
    content
)

with open(header_path, 'w') as f:
    f.write(content)

print(f"[fw_size] FW_IMAGE_SIZE={esp_size}, FW_IMAGE_SIZE_WITH_HEADER={size}")