from ftplib import FTP
from time import sleep
import requests
import wget
from datetime import datetime

now = datetime.now()

filepath = "./tracking files/"
filename = f"POFLUX_{now.strftime('%y%m%d_%H%M%S')}.csv"

print(filename)
wget.download("http://192.168.1.101/pof-lux/tracking", filepath + filename)

sleep(15)

x = requests.get("http://192.168.1.101/pof-lux/clear_tracking")

ftp = FTP("146.164.132.215")
ftp.login("poflux", "hDCsGW&BNBz%K3K")
with open(filepath + filename, "rb") as datafile:
    responseMessage = ftp.storbinary('STOR '+ filename, datafile) 
    print(responseMessage)
