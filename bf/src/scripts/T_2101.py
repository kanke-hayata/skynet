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
#機械学習利用T_2000 一時間足ver. classifier
#単純にデータを増やしまくると予測性能下がるっぽいのでコンパクトなデータサイズで学習したものを利用 <- でもそれは過学習なのでは
#時刻データを特徴量として利用
# -------------------------------------

if __name__ == '__main__':
    bot = Bot(balance = 0.01,name='T_2101')

    rospy.init_node('bot_T_2101',anonymous=False)
    rospy.Subscriber('child_order_events',BfChildOrderEvents,bot.callback_child_order_events) 
    rospy.Subscriber('board_snapshot',BfBoardSnapshot,bot.callback_board_snapshot) 

    xgbcs = []
    for i in range(5):
        xgbcs.append(pickle.load(open("/root/model/T_2101/xgbr"+str(i+1)+".pickle","rb")))


    data_columns_list = [
        ['Time','MACDM/O'],
        ['RSI','Time','MACDL/O','O-1/C'],
        ['MACDS/O', 'RSI', 'L/O', 'Time', 'MACDL/O'],
        ['MACDS/O', 'RSI', 'C/O', 'H/O', 'Time', 'ATR/O', 'MACDL/O', 'O-1/C', 'L-1/C'],
        ['RSI','Time','MACDS/O','O-1/C'],
        ['RSI','Time','MACDM/O']
    ]
    data_columns = []
    for l in data_columns_list:
        for c in l:
            if c not in data_columns:
                data_columns.append(c)

    while True:
        if 1 < time.time() % 3600 and time.time() % 3600 < 11:
            
            state = bot.pbf.getboardstate(product_code='FX_BTC_JPY')
            print(state)
            if type(state) == dict and 'state' in state.keys() and state['state'] == 'RUNNING' and state['health'] in ['NORMAL','BUSY','VERY BUSY']:

                df = utils.get_ohlc(periods=3600,datasize=500)
                df = utils.shaping_ohlc(df,['timestamp','Close']+data_columns)

                side = 0
                for i,xgbc in enumerate(xgbcs):
                    latest = df[-1:][data_columns_list[i]]
                    side += xgbc.predict(latest)[0]
                side_log = side

                if side == 5:
                    side = 3
                elif side == 3:
                    side = 2
                elif side == 1:
                    side = 1
                elif side == -1:
                    side = -1
                elif side == -3:
                    side = -2
                elif side == -5:
                    side = -3


                if side > 0:
                    if bot.side == 'no-position':
                        bot.market_buy(size=round(side*0.01,3))
                    elif bot.side == 'SELL':
                        bot.market_buy(size=round(bot.pos_vol()+side*0.01,3))
                    elif bot.side == 'BUY' and bot.pos_vol() < round(side*0.01,3):
                        bot.market_buy(size=round(side*0.01-bot.pos_vol(),3))
                    elif bot.side == 'BUY' and bot.pos_vol() > round(side*0.01,3):
                        bot.market_sell(size=round(bot.pos_vol()-side*0.01,3))
                elif side < 0:
                    side = -side
                    if bot.side == 'no-position':
                        bot.market_sell(size=round(side*0.01,3))
                    elif bot.side == 'BUY':
                        bot.market_sell(size=round(bot.pos_vol()+side*0.01,3))
                    elif bot.side == 'SELL' and bot.pos_vol() < round(side*0.01,3):
                        bot.market_sell(size=round(side*0.01-bot.pos_vol(),3))
                    elif bot.side == 'SELL' and bot.pos_vol() > round(side*0.01,3):
                        bot.market_buy(size=round(bot.pos_vol()-side*0.01,3))

                time.sleep(2)
                bot.info()
                print(bot.pos_vol())
                print(df[-1:].values)
                print(side_log)
                print("##################")

                time.sleep(10)

            else:
                time.sleep(1)
        else:
            time.sleep(1)
