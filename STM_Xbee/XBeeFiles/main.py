import micropython
from helpers.simple import MQTTClient
import sys

# ---------------- Azure Event Grid ----------------
HOST = "lynx-mqtt.swedencentral-1.ts.eventgrid.azure.net"
PORT = 8883
CLIENT_ID = "xbee"  # MQTT Client ID (session name)
USERNAME = "xbee"  # Client Authentication Name registered in Azure
PASSWORD = ""  # unused with certificate auth, but umqtt needs a
# string (not None) whenever USERNAME is set

# Topic comes from the STM32 at the start of every line:
#   HPF26/Telemetry/Can_1 {...}  or  HPF26/Telemetry/Can_2 {...}
# The Azure topic space must allow publishing to both.


# ---------------- Certificates ----------------
CERT_DIR = "/flash/cert"
CA_FILE = CERT_DIR + "/DigiCertGlobalRootG3.crt.pem"
CERT_FILE = CERT_DIR + "/client.crt"
KEY_FILE = CERT_DIR + "/client.key"

SSL_PARAMS = {
    "ca_certs": CA_FILE,  # XBee checks the Azure server with this
    "certfile": CERT_FILE,  # Azure checks the XBee with these two
    "keyfile": KEY_FILE,
}

CONNACK_CODES = {
    1: "unacceptable protocol version",
    2: "client ID rejected",
    3: "server unavailable",
    4: "bad username or password",
    5: "not authorized (cert thumbprint or username does not match the Azure client)",
}


rx = b""

def poll_stm32(client):
    global rx
    chunk = sys.stdin.buffer.read()   # всё, что пришло по UART; None, если пусто
    if not chunk:
        return
    rx += chunk
    while b"\n" in rx:
        line, rx = rx.split(b"\n", 1)
        line = line.strip()           # убирает \r, если STM32 шлёт \r\n
        if b" {" in line:             # "<topic> <json>"
            topic, payload = line.split(b" ", 1)
            client.publish(topic, payload)
    if len(rx) > 2048:                # мусор без \n долго копится — сбрасываем
        rx = b""

def main():


    client = MQTTClient(CLIENT_ID, HOST, port=PORT,
                        user=USERNAME, password=PASSWORD,
                        ssl=True, ssl_params=SSL_PARAMS)

    print("Connecting")
    client.connect()
    print("Connected")

    micropython.kbd_intr(-1)

    while True:
        poll_stm32(client)



if __name__ == "__main__":
    main()