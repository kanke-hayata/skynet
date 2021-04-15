#!/usr/bin/env python3.6

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
from bf.msg import (
    BfBoard,BfBoardSnapshot,
    BfChildOrderEvents,BfExecutions,
    BfParentOrderEvents,BfTicker
)

# -------------------------------------
key = os.environ['API_KEY']
secret = os.environ['API_SECRET']

end_point = 'wss://ws.lightstream.bitflyer.com/json-rpc'

public_channels = ['lightning_executions_FX_BTC_JPY','lightning_board_snapshot_FX_BTC_JPY','lightning_board_FX_BTC_JPY']
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

            if channel == 'lightning_board_FX_BTC_JPY':
                pub_board.publish(data)
            elif channel == 'lightning_board_snapshot_FX_BTC_JPY':
                msg = BfBoardSnapshot()
                msg.mid_price = int(data['mid_price'])

                bids = pd.DataFrame(data['bids']).sort_values('price',ascending=False)
                bids['price'] = bids['price'].round().astype(int)
                bids['size'] = (bids['size']*10**8).round().astype(int)
                msg.bids_price = bids['price'].values
                msg.bids_size = bids['size'].values

                asks = pd.DataFrame(data['asks']).sort_values('price')
                asks['price'] = asks['price'].round().astype(int)
                asks['size'] = (asks['size']*10**8).round().astype(int)
                msg.asks_price = asks['price'].values
                msg.asks_size = asks['size'].values

                pub_board_snapshot.publish(msg)

            elif channel == 'lightning_executions_FX_BTC_JPY':
                for e in data:
                    msg = BfExecutions()
                    msg.id = e['id']
                    msg.side = e['side']
                    msg.price = e['price']
                    msg.size = e['size']
                    msg.exec_date = e['exec_date']
                    msg.buy_child_order_acceptance_id = e['buy_child_order_acceptance_id']
                    msg.sell_child_order_acceptance_id = e['sell_child_order_acceptance_id']
                    pub_executions.publish(msg)

            elif channel == 'child_order_events':
                rospy.loginfo(data)
                for e in data:
                    msg = BfChildOrderEvents()
                    msg.product_code = e['product_code']
                    msg.child_order_id = e['child_order_id']
                    msg.child_order_acceptance_id = e['child_order_acceptance_id']
                    msg.event_date = e['event_date']
                    msg.event_type = e['event_type']
                    if e['event_type'] == 'ORDER':
                        msg.child_order_type = e['child_order_type']
                        msg.expire_date = e['expire_date']
                        msg.side = e['side']
                        msg.price = e['price']
                        msg.size = e['size']
                    elif e['event_type'] == 'ORDER_FAILED':
                        msg.reason = e['reason']
                    elif e['event_type'] == 'EXECUTION':
                        msg.exec_id = e['exec_id']
                        msg.side = e['side']
                        msg.price = e['price']
                        msg.size = e['size']
                        msg.commission = e['commission']
                        msg.sfd = e['sfd']
                    elif e['event_type'] == 'CANCEL':
                        msg.price = e['price']
                        msg.size = e['size']
                    else:
                        rospy.logwarn(data)

                    #rospy.loginfo('child_order_events: '+e['event_type'])
                    pub_child_order_events.publish(msg)
                    #rospy.Rate(30).sleep()

            elif channel == 'parent_order_events':
                rospy.loginfo(data)
                for e in data:
                    msg = BfParentOrderEvents()
                    msg.product_code = e['product_code']
                    msg.parent_order_id = e['parent_order_id']
                    msg.parent_order_acceptance_id = e['parent_order_acceptance_id']
                    msg.event_date = e['event_date']
                    msg.event_type = e['event_type']
                    if e['event_type'] == 'ORDER':
                        msg.parent_order_type = e['parent_order_type']
                        msg.expire_date = e['expire_date']
                    elif e['event_type'] == 'ORDER_FAILED':
                        msg.reason = e['reason']
                    elif e['event_type'] == 'TRIGGER':
                        msg.child_order_type = e['child_order_type']
                        msg.parameter_index = e['parameter_index']
                        msg.child_order_acceptance_id = e['child_order_acceptance_id']
                        msg.side = e['side']
                        msg.price = e['price']
                        msg.size = e['size']
                        msg.expire_date = e['expire_date']
                    elif e['event_type'] == 'COMPLETE':
                        msg.parameter_index = e['parameter_index']
                        msg.child_order_acceptance_id = e['child_order_acceptance_id']
                    elif e['event_type'] == 'CANCEL':
                        pass
                    else:
                        rospy.logwarn(data)

                    #rospy.loginfo('parent_order_events: '+e['event_type'])
                    pub_parent_order_events.publish(msg)
            else:
                pass
                #print(channel, len(data))

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

    pub_board = rospy.Publisher('board',String,queue_size=10)
    pub_board_snapshot = rospy.Publisher('board_snapshot',BfBoardSnapshot, queue_size=150)
    pub_executions = rospy.Publisher('executions', BfExecutions, queue_size=150)
    pub_child_order_events = rospy.Publisher('child_order_events', BfChildOrderEvents, queue_size=150)
    pub_parent_order_events = rospy.Publisher('parent_order_events', BfParentOrderEvents, queue_size=150)

    rospy.init_node('ws')

    ws = bFwebsocket(end_point, public_channels, private_channels, key, secret)
    ws.startWebsocket()
