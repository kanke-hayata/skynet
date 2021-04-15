#!/usr/bin/env python3.6

import os
import time
import json
import copy
import datetime
from pytz import timezone
from threading import Thread
import threading

import numpy as np
import pandas as pd
import dateutil.parser
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
#秒足1mokuを利用したmmbotの作成
# -------------------------------------

class HFTBot(Bot):
    def __init__(self,balance=0.0,name='',disparity_rate=0.0):
        super().__init__(balance,name)
        self.executions = pd.DataFrame(columns=['price','exec_date'])
        self.seconds = pd.DataFrame()

    def callback_executions(self,data):
        exec_date = np.ceil(dateutil.parser.parse(data.exec_date).timestamp())
        execution = pd.Series([data.price,exec_date],index=['price','exec_date'])
        self.executions = self.executions.append(execution,ignore_index=True)

        if self.executions['exec_date'].nunique() >= 2:
            minimum = self.executions['exec_date'].min()
            grouped = self.executions[self.executions['exec_date'] == minimum].groupby('exec_date')
            # 新品だけ持ち越し
            self.executions = self.executions[self.executions['exec_date'] != minimum]

            seconds = pd.DataFrame()
            seconds['timestamp'] = grouped['exec_date'].first()
            seconds['Open'] = grouped['price'].first()
            seconds['High'] = grouped['price'].max()
            seconds['Low'] = grouped['price'].min()
            seconds['Close'] = grouped['price'].last()
            seconds = seconds.reset_index(drop=True)
            #merge
            self.seconds = pd.concat([self.seconds,seconds],ignore_index=True)
            #60tickまでに抑える
            self.seconds = self.seconds[self.seconds['timestamp'].values[-1] - self.seconds['timestamp'] <= 90]

            seconds_processed = self.seconds.copy()
            seconds_processed = utils.shaping_ohlc(seconds_processed,['Close','1moku-precedent-span1','1moku-precedent-span2','1moku-signal3','1moku-signal5'])

            if seconds_processed['1moku-signal3'].values[-1] == 1 or seconds_processed['1moku-signal5'].values[-1] == -1:
                self.limit_buy(price=data.price,size=0.01)
            elif seconds_processed['1moku-signal3'].values[-1] == -1 or seconds_processed['1moku-signal5'].values[-1] == 1:
                self.limit_sell(price=data.price,size=0.01)

            print(seconds_processed[-1:][['Close','1moku-precedent-span1','1moku-precedent-span2']].values)

if __name__ == '__main__':
    bot = HFTBot(balance = 0.01,name='T_3000')

    rospy.init_node('bot_T_3000',anonymous=False)
    rospy.Subscriber('child_order_events',BfChildOrderEvents,bot.callback_child_order_events) 
    rospy.Subscriber('parent_order_events',BfParentOrderEvents,bot.callback_parent_order_events) 
    rospy.Subscriber('board_snapshot',BfBoardSnapshot,bot.callback_board_snapshot) 
    rospy.Subscriber('executions',BfExecutions,bot.callback_executions)

    while True:
        time.sleep(3)
        bot.info()
