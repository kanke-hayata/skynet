import pybitflyer
import json
import websocket
import hmac
from hashlib import sha256
from secrets import token_hex
import time
from threading import Thread
import signal
import os
import pandas as pd
import datetime
import requests
import numpy as np
import rospy
from std_msgs.msg import String

# -------------------------------------
key = '9p18g8Ju3SDDDgdY1JkjBD'
secret = '4y+iSjbJo7hRnYpzXvIaiheCrL+HU24E1iCuWx3tY2Y='

end_point = 'wss://ws.lightstream.bitflyer.com/json-rpc'

public_channels = ['lightning_executions_FX_BTC_JPY','lightning_board_snapshot_FX_BTC_JPY']
private_channels = ['child_order_events', 'parent_order_events']
#-------------------------------------

def quit_loop(signal, frame):
    os._exit(0)

class bFwebsocket(object):
    def __init__(self, end_point, public_channels, private_channels, key, secret):
        self._end_point = end_point
        self._public_channels = public_channels
        self._private_channels = private_channels
        self._key = key
        self._secret = secret
        self._JSONRPC_ID_AUTH = 1
        self.position = 'no-position'
        self.asks = pd.DataFrame(columns=['price','size'])
        self.bids = pd.DataFrame(columns=['price','size'])
        self.httpclient = pybitflyer.API(api_key=key,api_secret=secret)
        self.span_before = time.time()
        self.request_count = 0
        self.parent_orders = pd.DataFrame(columns=[])
        self.child_orders = pd.DataFrame(columns=[])
        
    def startWebsocket(self):
        def on_open(ws):
            print("Websocket connected")

            if len(self._private_channels) > 0:
                auth(ws)

            if len(self._public_channels) > 0:
                params = [{'method': 'subscribe', 'params': {'channel': c}}
                         for c in self._public_channels]
                ws.send(json.dumps(params))

        def on_error(ws, error):
            print(error)

        def on_close(ws):
            print("Websocket closed")

        def run(ws):
            while True:
                ws.run_forever()
                print("******")
                time.sleep(3)

        def on_message(ws, message):
            messages = json.loads(message)

            # auth response
            if 'id' in messages and messages['id'] == self._JSONRPC_ID_AUTH:
                if 'error' in messages:
                    print('auth error: {}'.format(messages["error"]))
                elif 'result' in messages and messages['result'] == True:
                    params = [{'method': 'subscribe', 'params': {'channel': c}}
                             for c in self._private_channels]
                    ws.send(json.dumps(params))

            if 'method' not in messages or messages['method'] != 'channelMessage':
                return

            params = messages["params"]
            channel = params["channel"]
            data = params["message"]

            if channel == 'child_order_events':
                print('child_order_events',data)
            else:
                print(channel, len(data))

        def auth(ws):
            now = int(time.time())
            nonce = token_hex(16)
            sign = hmac.new(self._secret.encode(
               'utf-8'), ''.join([str(now), nonce]).encode('utf-8'), sha256).hexdigest()
            params = {'method': 'auth', 'params': {'api_key': self._key, 'timestamp': now,
                                                  'nonce': nonce, 'signature': sign}, 'id': self._JSONRPC_ID_AUTH}
            ws.send(json.dumps(params))

        ws = websocket.WebSocketApp(self._end_point, on_open=on_open,
                                   on_message=on_message, on_error=on_error, on_close=on_close)
        websocketThread = Thread(target=run, args=(ws, ))
        websocketThread.start()


if __name__ == '__main__':
   # Ctrl+C
    signal.signal(signal.SIGINT, quit_loop)

    pub = rospy.Publisher('bf-ws-msg',String, queue_size=100)
    rospy.init_node('bf-ws',anonymous=True)

    ws = bFwebsocket(end_point, public_channels, private_channels, key, secret)
    ws.startWebsocket()