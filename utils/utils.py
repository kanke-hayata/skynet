import pandas as pd
import time
import talib
import requests
import numpy as np

#period=60,300,3600,14400,24h,7days
def get_ohlc(periods=3600,datasize=3000):
    cnt = 0
    now = int(time.time())
    ohlc = pd.DataFrame([],columns=['timestamp','Open','High','Low','Close'])
    while True:
        response = requests.get("https://api.cryptowat.ch/markets/bitflyer/btcfxjpy/ohlc",params = {"periods": periods,"before":now-periods*cnt*1000,"after":now-periods*(cnt+1)*1000})
        if response.json():
            ohlc_tmp = np.array(response.json()['result'][str(periods)])[:,:5] # 長すぎると計算時間が増えるので最新100件に整形
            ohlc = pd.concat([pd.DataFrame(ohlc_tmp,columns=['timestamp','Open','High','Low','Close']),ohlc],ignore_index=True)
            cnt += 1
    
        if cnt >= datasize/1000:
            return ohlc[-datasize:]
        
def shaping_ohlc(ohlc,drop=True):
    
    data_columns = []
    ohlc['O/O'] = ohlc['Open']/ohlc['Open']
    ohlc['H/O'] = ohlc['High']/ohlc['Open']
    ohlc['L/O'] = ohlc['Low']/ohlc['Open']
    ohlc['C/O'] = ohlc['Close']/ohlc['Open']
    data_columns += ['O/O','H/O','L/O','C/O']
    ohlc['O/C'] = ohlc['Open']/ohlc['Close']
    ohlc['H/C'] = ohlc['High']/ohlc['Close']
    ohlc['L/C'] = ohlc['Low']/ohlc['Close']
    ohlc['C/C'] = ohlc['Close']/ohlc['Close']
    data_columns += ['O/C','H/C','L/C','C/C']

    ohlc['H-L/O'] = ohlc['H/O']-ohlc['L/O']
    data_columns += ['H-L/O']

    ohlc['PositiveLine'] = (ohlc['Open'] <= ohlc['Close'])*1
    ohlc['NegativeLine'] = (ohlc['Open'] > ohlc['Close'])*(-1)
    ohlc['SignLine'] = ohlc['PositiveLine'] + ohlc['NegativeLine']
    ohlc['3SignLine'] = ohlc['SignLine'].rolling(3).sum()
    ohlc['6SignLine'] = ohlc['SignLine'].rolling(6).sum()
    ohlc['12SignLine'] = ohlc['SignLine'].rolling(12).sum()
    data_columns += ['PositiveLine','NegativeLine','SignLine','3SignLine','6SignLine','12SignLine']

    ohlc['MACDS'] = talib.MACD(ohlc['Close'],fastperiod=6,slowperiod=19,signalperiod=9)[0]
    ohlc['MACDS/O'] = talib.MACD(ohlc['C/O'],fastperiod=6,slowperiod=19,signalperiod=9)[0]
    ohlc['MACDM'] = talib.MACD(ohlc['Close'],fastperiod=12,slowperiod=26,signalperiod=9)[0]
    ohlc['MACDM/O'] = talib.MACD(ohlc['C/O'],fastperiod=12,slowperiod=26,signalperiod=9)[0]
    ohlc['MACDL'] = talib.MACD(ohlc['Close'],fastperiod=19,slowperiod=39,signalperiod=9)[0]
    ohlc['MACDL/O'] = talib.MACD(ohlc['C/O'],fastperiod=19,slowperiod=39,signalperiod=9)[0]
    data_columns += ['MACDS','MACDS/O','MACDM','MACDM/O','MACDL','MACDL/O']

    ohlc['ATR'] = talib.ATR(ohlc['High'],ohlc['Low'],ohlc['Close'],timeperiod=14)
    data_columns += ['ATR']
    ohlc['ATR/O'] = talib.ATR(ohlc['H/O'],ohlc['L/O'],ohlc['O/O'],timeperiod=14)
    data_columns += ['ATR/O']

    ohlc['RSI'] = talib.RSI(ohlc['Close'],timeperiod=14)
    data_columns += ['RSI']

    ohlc['1moku-base'] = (ohlc['High'].rolling(26).max()+ohlc['Low'].rolling(26).min())/2
    ohlc['1moku-conversion'] = (ohlc['High'].rolling(9).max()+ohlc['Low'].rolling(9).min())/2
    ohlc['1moku-precedent-span1'] = ((ohlc['1moku-base']+ohlc['1moku-conversion'])/2).shift(26)
    ohlc['1moku-precedent-span2'] = ((ohlc['High'].rolling(52).max()+ohlc['Low'].rolling(52).min())/2).shift(26)
    #ohlc['1moku-late-span'] = ohlc['Close'].shift(-26)
    ohlc['1moku-signal1'] = ((ohlc['1moku-conversion']-ohlc['1moku-base'] > 0) *1).diff(1)
    ohlc['1moku-signal2'] = (ohlc['1moku-conversion']-ohlc['1moku-base'] > 0) *1
    ohlc['1moku-signal3'] = (((ohlc['Close'] > ohlc['1moku-precedent-span1'])*1) * ((ohlc['Close'] > ohlc['1moku-precedent-span2'])*1)).diff(1)
    ohlc['1moku-signal4'] = ((ohlc['Close'] > ohlc['1moku-precedent-span1'])*1) * ((ohlc['Close'] > ohlc['1moku-precedent-span2'])*1)
    ohlc['1moku-signal5'] = (((ohlc['Close'] < ohlc['1moku-precedent-span1'])*1) * ((ohlc['Close'] < ohlc['1moku-precedent-span2'])*1)).diff(1)
    ohlc['1moku-signal6'] = ((ohlc['Close'] < ohlc['1moku-precedent-span1'])*1) * ((ohlc['Close'] < ohlc['1moku-precedent-span2'])*1)
    data_columns += ['1moku-base','1moku-conversion','1moku-precedent-span1','1moku-precedent-span2']
    data_columns += ['1moku-signal1','1moku-signal2','1moku-signal3','1moku-signal4','1moku-signal5','1moku-signal6']

    target_columns = []

    ohlc['Target_Open'] = ohlc['Open'].shift(-1)
    ohlc['Target_High'] = ohlc['High'].shift(-1)
    ohlc['Target_Low'] = ohlc['Low'].shift(-1)
    ohlc['Target_Close'] = ohlc['Close'].shift(-1)
    ohlc['TO/O'] = ohlc['Target_Open']/ohlc['Open']
    ohlc['TH/O'] = ohlc['Target_High']/ohlc['Open']
    ohlc['TL/O'] = ohlc['Target_Low']/ohlc['Open']
    ohlc['TC/O'] = ohlc['Target_Close']/ohlc['Open']
    ohlc['TH/C'] = ohlc['Target_High']/ohlc['Close']
    ohlc['TL/C'] = ohlc['Target_Low']/ohlc['Close']
    ohlc['TC/C'] = ohlc['Target_Close']/ohlc['Close']
    target_columns += ['Target_Open','Target_high','Target_Low','Target_Close','TO/O','TH/O','TL/O','TC/O','TH/C','TL/C','TC/C']

    ohlc['Class'] = (ohlc['TC/C'] > 1)*1
    target_columns += ['Class']

    if drop:
        ohlc = ohlc.dropna()
    
    return ohlc,data_columns,target_columns