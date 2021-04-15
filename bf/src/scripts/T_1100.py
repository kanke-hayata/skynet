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
#機械学習利用T_1000シリーズの三作目 一時間足ver.
#単純にデータを増やしまくると予測性能下がるっぽいのでコンパクトなデータサイズで学習したものを利用
#1000に比べてMACDS->MACDS/Oに変更 賞味期限はより良いと思われる
# -------------------------------------

if __name__ == '__main__':
    bot = Bot(balance = 0.01,name='T_1100')

    rospy.init_node('bot_T_1100',anonymous=False)
    rospy.Subscriber('child_order_events',BfChildOrderEvents,bot.callback_child_order_events) 
    rospy.Subscriber('board_snapshot',BfBoardSnapshot,bot.callback_board_snapshot) 

    xgbr = pickle.load(open("/root/model/T_1100/xgbr.pickle","rb"))


    while True:
        if 1 < time.time() % 3600 and time.time() % 3600 < 11:
            
            state = bot.pbf.getboardstate(product_code='FX_BTC_JPY')
            print(state)
            if type(state) == dict and 'state' in state.keys() and state['state'] == 'RUNNING' and state['health'] in ['NORMAL','BUSY','VERY BUSY']:

                before = utils.get_ohlc(periods=300,datasize=500)
                before['UP'] = (before['Open'] <= before['Close'])*2 -1
                bins = list(range(int(before['timestamp'].values[0]-(before['timestamp'].values[0]-300)%3600),int(before['timestamp'].values[-1]-(before['timestamp'].values[-1]-300)%3600+3600*2),3600))
                before['bins'] = pd.cut(before['timestamp'],bins,right=False)
                grouped = before.groupby('bins')
                df = pd.DataFrame()
                df['timestamp'] = grouped['timestamp'].last()
                df['Open'] = grouped['Open'].first()
                df['High'] = grouped['High'].max()
                df['Low'] = grouped['Low'].min()
                df['Close'] = grouped['Close'].last()
                df['FirstUP'] = grouped['UP'].first()
                df = df.reset_index(drop=True)
                df['UP'] = (df['Open'] <= df['Close'])*2 -1


                df['H/O'] = df['High']/df['Open']
                df['L/O'] = df['Low']/df['Open']
                df['C/O'] = df['Close']/df['Open']
                df['C-1/C'] = df['Close'].shift(1)/df['Close']

                df['MACDS/O'] = talib.MACD(df['C/O'],fastperiod=6,slowperiod=19,signalperiod=9)[0]

                latest = df[-1:][['MACDS/O','UP','FirstUP','C/O','C-1/C','H/O','L/O']]


                side = xgbr.predict(latest)[0] >= 1.000
                if side:
                    if bot.side == 'no-position':
                        bot.market_buy(size=bot.balance)
                    elif bot.side == 'SELL':
                        bot.market_buy(size=bot.balance*2)
                elif not side:
                    if bot.side == 'no-position':
                        bot.market_sell(size=bot.balance)
                    elif bot.side == 'BUY':
                        bot.market_sell(size=bot.balance*2)

                print(side)
                print(df[-1:]['timestamp'].values[0],df[-1:]['Open'].values[0],df[-1:]['Close'].values[0])
                ohlc = utils.get_ohlc(periods=3600,datasize=100)
                print(ohlc[-1:]['timestamp'].values[0],ohlc[-1:]['Open'].values[0],ohlc[-1:]['Close'].values[0])
                time.sleep(2)
                bot.info()

                time.sleep(10)

            else:
                time.sleep(1)
        else:
            time.sleep(1)
