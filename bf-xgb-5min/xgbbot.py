import pybitflyer
import json
import time
from threading import Thread
import threading
import os
import pandas as pd
import talib
import requests
import numpy as np
import xgboost 
from sklearn.model_selection import train_test_split, GridSearchCV
import copy
import datetime
import pickle
import utils
import xgboost_ohlc

# -------------------------------------
key = '9p18g8Ju3SDDDgdY1JkjBD'
secret = '4y+iSjbJo7hRnYpzXvIaiheCrL+HU24E1iCuWx3tY2Y='
# -------------------------------------

def trade_loop(pyb):

    xgbc = pickle.load(open("xgbc_5min.pickle","rb"))

    base_volume = [0.01,0.03,0.05,0.05,0.07,0.08]
    bv_ind = 2
    while True:
        if 1 < time.time() % 300 and time.time() % 300 < 11:
            
            state = pyb.getboardstate(product_code='FX_BTC_JPY')
            if type(state) == dict and 'state' in state.keys() and state['state'] == 'RUNNING' and state['health'] in ['NORMAL','BUSY','VERY BUSY']:

                ohlc = utils.get_ohlc(periods=300,datasize=500)
                ohlc, data_columns, target_columns = utils.shaping_ohlc(ohlc,drop=False)

                X_test = ohlc[-1:][data_columns]


                flag = xgbc.predict(X_test)[0] == 1

                side = 'NO'
                position = pyb.getpositions(product_code="FX_BTC_JPY")
                pos_volume = 0
                if type(position) == list and len(position) >= 1:
                    side = position[0]['side']
                    for p in position:
                        pos_volume += p['size']


                board = pyb.board(product_code="FX_BTC_JPY")
                if (type(board) != dict) or (type(board) == dict and 'bids' not in board.keys()):
                    continue

                result = 'continue'
                if flag and side == 'SELL' and round(pos_volume+base_volume[bv_ind],2)>=0.01:
                    result = pyb.sendchildorder(
                        product_code="FX_BTC_JPY",
                        child_order_type="LIMIT",
                        side="BUY",
                        size=round(pos_volume+base_volume[bv_ind],2),
                        price= board['bids'][0]['price']+1,
                        minute_to_expire=4,
                        time_in_force="GTC"
                    )
                elif not flag and side == 'BUY' and round(pos_volume+base_volume[bv_ind],2)>=0.01:
                    result = pyb.sendchildorder(
                        product_code="FX_BTC_JPY",
                        child_order_type="LIMIT",
                        side="SELL",
                        size=round(pos_volume+base_volume[bv_ind],2),
                        price=board['asks'][0]['price']-1,
                        minute_to_expire=4,
                        time_in_force="GTC"
                    )
                elif flag and side == 'NO' and base_volume[bv_ind] >= 0.01:
                    result = pyb.sendchildorder(
                        product_code="FX_BTC_JPY",
                        child_order_type="LIMIT",
                        side="BUY",
                        size=base_volume[bv_ind],
                        price=board['bids'][0]['price']+1,
                        minute_to_expire=4,
                        time_in_force="GTC"
                    )
                elif not flag and side == 'NO' and base_volume[bv_ind] >= 0.01:
                    result = pyb.sendchildorder(
                        product_code="FX_BTC_JPY",
                        child_order_type="LIMIT",
                        side="SELL",
                        size=base_volume[bv_ind],
                        price=board['asks'][0]['price']-1,
                        minute_to_expire=4,
                        time_in_force="GTC"
                    )
                #買いまし 売りまし
                elif flag and side == 'BUY' and round(base_volume[bv_ind]-pos_volume,2)>=0.01:
                    result = pyb.sendchildorder(
                        product_code="FX_BTC_JPY",
                        child_order_type="LIMIT",
                        side="BUY",
                        size=round(base_volume[bv_ind]-pos_volume,2),
                        price=board['bids'][0]['price']+1,
                        minute_to_expire=4,
                        time_in_force="GTC"
                    )
                elif flag and side == 'BUY' and round(base_volume[bv_ind]-pos_volume,2)<=-0.01:
                    result = pyb.sendchildorder(
                        product_code="FX_BTC_JPY",
                        child_order_type="LIMIT",
                        side="SELL",
                        size=round(pos_volume-base_volume[bv_ind],2),
                        price=board['bids'][0]['price']+1,
                        minute_to_expire=4,
                        time_in_force="GTC"
                    )
                elif not flag and side == 'SELL' and round(base_volume[bv_ind]-pos_volume,2)>=0.01:
                    result = pyb.sendchildorder(
                        product_code="FX_BTC_JPY",
                        child_order_type="LIMIT",
                        side="SELL",
                        size=round(base_volume[bv_ind]-pos_volume,2),
                        price=board['asks'][0]['price']-1,
                        minute_to_expire=4,
                        time_in_force="GTC"
                    )
                elif not flag and side == 'SELL' and round(base_volume[bv_ind]-pos_volume,2)<=-0.01:
                    result = pyb.sendchildorder(
                        product_code="FX_BTC_JPY",
                        child_order_type="LIMIT",
                        side="BUY",
                        size=round(pos_volume-base_volume[bv_ind],2),
                        price=board['asks'][0]['price']-1,
                        minute_to_expire=4,
                        time_in_force="GTC"
                    )
                
                #base_volume = [0.01,0.03,0.05,0.07,0.1,0.13]
                history = pyb.getcollateralhistory()
                if type(history) == list and len(history) >= 1:
                    history_count = 0
                    history_sum = 0
                    for h in history:
                        history_count += (h['change']>0)*1
                        history_sum += h['change']
                    if history_sum > 0 and history_count > 90:
                        bv_ind = 5
                    elif history_sum > 0 and history_count > 75:
                        bv_ind = 5
                    elif history_sum > 0 and history_count > 50:
                        bv_ind = 4
                    else:
                        bv_ind = 3 
                        
                print(flag,result)
                print(datetime.datetime.now())
                print('ohlc gap',time.time()-ohlc['timestamp'].values[-1])
                print(base_volume,bv_ind,pos_volume,history_count,history_sum)
                print(int(time.time())%(3600*24))

                time.sleep(10)
            else:
                time.sleep(1)
        else:
            time.sleep(1)


if __name__ == '__main__':
    
    pyb = pybitflyer.API(api_key=key,api_secret=secret)
    
    tradeThread = Thread(target=trade_loop,args=(pyb,))
    tradeThread.start()