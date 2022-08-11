import wget
from datetime import datetime

now = datetime.now()

filename = f"POFLUX_{now.strftime('%y%m%d_%H%M%S')}"

print(filename)
wget.download("http://192.168.1.101/pof-lux/tracking", filename)