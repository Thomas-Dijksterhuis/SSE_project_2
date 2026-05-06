import paho.mqtt.client as mqtt
import json


def on_message(client, userdata, message):
    try:
        pass
    except Exception as e:
        print("Error:", e)  

broker_address = "192.168.1.226"

client = mqtt.Client(
    client_id="python1",
    callback_api_version=mqtt.CallbackAPIVersion.VERSION1
)

client.on_message = on_message

client.connect(broker_address)
client.subscribe("Readings")
client.loop_forever()