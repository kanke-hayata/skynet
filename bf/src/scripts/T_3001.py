#!/usr/bin/env python3.6

import os
import time
import json
import copy
import datetime
from threading import Thread
import threading

import numpy as np
import pandas as pd
import talib
import xgboost 
import pickle
import requests
import pybitflyer
import rospy

import utils
import xgboost_ohlc
from bot_base import Bot
from bf.msg import (
    BfBoard,BfBoardSnapshot,
    BfChildOrderEvents,BfExecutions,
    BfParentOrderEvents,BfTicker
)

# -----------bot description-----------
# mm series
# rate幅におさまるかどうかを機械学習で判断
# -------------------------------------

if __name__ == '__main__':
    bot = Bot(balance = 0.01,name='T_3001')

    rospy.init_node('bot_T_3001',anonymous=False)
    rospy.Subscriber('child_order_events',BfChildOrderEvents,bot.callback_child_order_events) 
    rospy.Subscriber('parent_order_events',BfParentOrderEvents,bot.callback_parent_order_events) 
    rospy.Subscriber('board_snapshot',BfBoardSnapshot,bot.callback_board_snapshot) 

    xgbc = pickle.load(open("/root/model/T_3001/xgbc.pickle","rb"))

    rate = 0.00035
    bid = 0
    bid2 = 0
    ask = 0
    ask2 = 0

    side_sum = 0
    trials = 1

    data_columns = ['MACDS/O','RSI','ATR/O','Time','C/O','O-1/C','H/O','L/O']


    while True:
        if 1 < time.time() % 300 and time.time() % 300 < 11:
            
            state = bot.pbf.getboardstate(product_code='FX_BTC_JPY')
            print(state)
            if type(state) == dict and 'state' in state.keys() and state['state'] == 'RUNNING' and state['health'] in ['NORMAL','BUSY','VERY BUSY']:

                df= utils.get_ohlc(periods=300,datasize=500)
                df= utils.shaping_ohlc(df,['timestamp','Close']+data_columns)

                latest = df[-1:][data_columns]

                side = xgbc.predict(latest)[0] == 1
                if side:
                    bid = int(df['Close'].values[-1]*(1-rate))
                    ask = int(df['Close'].values[-1]*(1+rate))
                    bot.limit_buy(price=bid,size=0.01)
                    bot.limit_sell(price=ask,size=0.01)
                    side_sum += 1

                time.sleep(2)
                bot.info()
                print(time.time())
                print(df[-1:].values)
                print(side)
                print(side_sum/trials)
                print("##################")

                trials += 1

                time.sleep(10)

            else:
                time.sleep(1)

        elif 290 < time.time() % 300 and time.time() % 300 < 295:

            bot.cancel_all_child_orders()
            bot.settlement_positions()

            time.sleep(5)
        else:
            time.sleep(0.5)
