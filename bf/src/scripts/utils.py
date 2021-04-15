import pandas as pd
import time
import talib
import requests
import numpy as np
import os

#period=60,300,3600,14400,24h,7days
def get_ohlc(periods=3600,datasize=3000,api_key=True):
    cnt = 0
    now = int(time.time())
    ohlc = pd.DataFrame([],columns=['timestamp','Open','High','Low','Close'])
    while True:
        if api_key:
            response = requests.get("https://api.cryptowat.ch/markets/bitflyer/btcfxjpy/ohlc",params = {"periods": periods,"before":now-periods*cnt*1000,"after":now-periods*(cnt+1)*1000,"apikey": os.environ['CW_API_KEY']})
        else:
            response = requests.get("https://api.cryptowat.ch/markets/bitflyer/btcfxjpy/ohlc",params = {"periods": periods,"before":now-periods*cnt*1000,"after":now-periods*(cnt+1)*1000})
        if type(response.json()) == dict and 'result' in response.json().keys():
            ohlc_tmp = np.array(response.json()['result'][str(periods)])[:,:5] # 長すぎると計算時間が増えるので最新100件に整形
            ohlc = pd.concat([pd.DataFrame(ohlc_tmp,columns=['timestamp','Open','High','Low','Close']),ohlc],ignore_index=True)
            cnt += 1
    
        if cnt >= datasize/1000:
            return ohlc[-datasize:]

def check_cdrr(column,cdrr,columns):
    for c in columns:
        if c in cdrr[column]:
            return True
    return False

def shaping_ohlc(ohlc,columns,drop=False,bbtp=9,over_rate=0.00035):

    #columns dependency reverse resolution
    cdrr = {
            'O/O':['O/O','ATR/O'],
            'H/O':['H/O','H-L/O','ATR/O'],
            'L/O':['L/O','H-L/O','ATR/O'],
            'C/O':['C/O','MACDS/O','MACDM/O','MACDL/O'],
            'O/C':['O/C'],
            'H/C':['H/C','H-L/C'],
            'L/C':['L/C','H-L/C'],
            'C/C':['C/C'],
            'H-L/O':['H-L/O'],
            'H-L/C':['H-L/C'],
            'O-1/C':['O-1/C'],
            'H-1/C':['H-1/C'],
            'L-1/C':['L-1/C'],
            'C-1/C':['C-1/C'],
            'PositiveLine':['PositiveLine','SignLine','3SignLine','6SignLine','12SignLine'],
            'NegativeLine':['NegativeLine','SignLine','3SignLine','6SignLine','12SignLine'],
            'SignLine':['SignLine'],
            '3SignLine':['3SignLine'],
            '6SignLine':['6SignLine'],
            '12SignLine':['12SignLine'],
            'BB':['BBUpper','BBMiddle','BBLower','BBOver','BBUnder'],
            'BBOver':['BBOver'],
            'BBUnder':['BBUnder'],
            'MACDS':['MACDS'],
            'MACDM':['MACDM'],
            'MACDL':['MACDL'],
            'MACDS/O':['MACDS/O'],
            'MACDM/O':['MACDM/O'],
            'MACDL/O':['MACDL/O'],
            'ATR':['ATR'],
            'ATR/O':['ATR/O'],
            'RSI':['RSI'],
            'Time':['Time'],
            'Over':['Over','Target_Over'],
            '1moku-base':['1moku-base','1moku-precedent-span1','1moku-signal1','1moku-signal2','1moku-signal3','1moku-signal4','1moku-signal5','1moku-signal6'],
            '1moku-conversion':['1moku-conversion','1moku-precedent-span1','1moku-signal1','1moku-signal2','1moku-signal3','1moku-signal4','1moku-signal5','1moku-signal6'],
            '1moku-precedent-span1':['1moku-precedent-span1','1moku-signal3','1moku-signal4','1moku-signal5','1moku-signal6'],
            '1moku-precedent-span2':['1moku-precedent-span2','1moku-signal3','1moku-signal4','1moku-signal5','1moku-signal6'],
            '1moku-late-span':['1moku-late-span'],
            '1moku-signal1':['1moku-signal1'],
            '1moku-signal2':['1moku-signal2'],
            '1moku-signal3':['1moku-signal3'],
            '1moku-signal4':['1moku-signal4'],
            '1moku-signal5':['1moku-signal5'],
            '1moku-signal6':['1moku-signal6'],
            'Target_Open':['Target_Open','Target_O/O','Target_H/O','Target_L/O','Target_C/O','Target_O/C'],
            'Target_High':['Target_High','Target_H/O','Target_H/C'],
            'Target_Low':['Target_Low','Target_L/O','Target_L/C'],
            'Target_Close':['Target_Close','Target_C/O','Target_O/C','Target_H/C','Target_L/C','Target_C/C'],
            'Target_O/O':['Target_O/O'],
            'Target_H/O':['Target_H/O'],
            'Target_L/O':['Target_L/O'],
            'Target_C/O':['Target_C/O'],
            'Target_O/C':['Target_O/C'],
            'Target_H/C':['Target_H/C'],
            'Target_L/C':['Target_L/C'],
            'Target_C/C':['Target_C/C'],
            'Target_Over':['Target_Over'],
            }
    if check_cdrr('O/O',cdrr,columns):
        ohlc['O/O'] = ohlc['Open']/ohlc['Open']

    if check_cdrr('H/O',cdrr,columns):
        ohlc['H/O'] = ohlc['High']/ohlc['Open']

    if check_cdrr('L/O',cdrr,columns):
        ohlc['L/O'] = ohlc['Low']/ohlc['Open']

    if check_cdrr('C/O',cdrr,columns):
        ohlc['C/O'] = ohlc['Close']/ohlc['Open']

    if check_cdrr('O/C',cdrr,columns):
        ohlc['O/C'] = ohlc['Open']/ohlc['Close']

    if check_cdrr('H/C',cdrr,columns):
        ohlc['H/C'] = ohlc['High']/ohlc['Close']

    if check_cdrr('L/C',cdrr,columns):
        ohlc['L/C'] = ohlc['Low']/ohlc['Close']

    if check_cdrr('C/C',cdrr,columns):
        ohlc['C/C'] = ohlc['Close']/ohlc['Close']

    if check_cdrr('H-L/O',cdrr,columns):
        ohlc['H-L/O'] = ohlc['H/O']-ohlc['L/O']

    if check_cdrr('H-L/C',cdrr,columns):
        ohlc['H-L/C'] = ohlc['H/C']-ohlc['L/C']

    if check_cdrr('O-1/C',cdrr,columns):
        ohlc['O-1/C'] = ohlc['Open'].shift(1)/ohlc['Close']

    if check_cdrr('H-1/C',cdrr,columns):
        ohlc['H-1/C'] = ohlc['High'].shift(1)/ohlc['Close']

    if check_cdrr('L-1/C',cdrr,columns):
        ohlc['L-1/C'] = ohlc['Low'].shift(1)/ohlc['Close']

    if check_cdrr('C-1/C',cdrr,columns):
        ohlc['C-1/C'] = ohlc['Close'].shift(1)/ohlc['Close']

    if check_cdrr('PositiveLine',cdrr,columns):
        ohlc['PositiveLine'] = (ohlc['Open'] <= ohlc['Close'])*1

    if check_cdrr('NegativeLine',cdrr,columns):
        ohlc['NegativeLine'] = (ohlc['Open'] > ohlc['Close'])*(-1)

    if check_cdrr('SignLine',cdrr,columns):
        ohlc['SignLine'] = ohlc['PositiveLine'] + ohlc['NegativeLine']

    if check_cdrr('3SignLine',cdrr,columns):
        ohlc['3SignLine'] = ohlc['SignLine'].rolling(3).sum()

    if check_cdrr('6SignLine',cdrr,columns):
        ohlc['6SignLine'] = ohlc['SignLine'].rolling(6).sum()

    if check_cdrr('12SignLine',cdrr,columns):
        ohlc['12SignLine'] = ohlc['SignLine'].rolling(12).sum()

    if check_cdrr('BB',cdrr,columns):
        upper, middle, lower = talib.BBANDS(ohlc['Close'],timeperiod=bbtp,nbdevup=2,nbdevdn=2,matype=0)
        ohlc['BBUpper'] = upper
        ohlc['BBMiddle'] = middle
        ohlc['BBLower'] = lower

    if check_cdrr('BBOver',cdrr,columns):
        ohlc['BBOver'] = ohlc['BBUpper'] < ohlc['Close']

    if check_cdrr('BBUnder',cdrr,columns):
        ohlc['BBUnder'] = ohlc['Close'] < ohlc['BBLower']

    if check_cdrr('MACDS',cdrr,columns):
        ohlc['MACDS'] = talib.MACD(ohlc['Close'],fastperiod=6,slowperiod=19,signalperiod=9)[0]
        
    if check_cdrr('MACDM',cdrr,columns):
        ohlc['MACDM'] = talib.MACD(ohlc['Close'],fastperiod=12,slowperiod=26,signalperiod=9)[0]

    if check_cdrr('MACDL',cdrr,columns):
        ohlc['MACDL'] = talib.MACD(ohlc['Close'],fastperiod=19,slowperiod=39,signalperiod=9)[0]

    if check_cdrr('MACDS/O',cdrr,columns):
        ohlc['MACDS/O'] = talib.MACD(ohlc['C/O'],fastperiod=6,slowperiod=19,signalperiod=9)[0]

    if check_cdrr('MACDM/O',cdrr,columns):
        ohlc['MACDM/O'] = talib.MACD(ohlc['C/O'],fastperiod=12,slowperiod=26,signalperiod=9)[0]
        
    if check_cdrr('MACDL/O',cdrr,columns):
        ohlc['MACDL/O'] = talib.MACD(ohlc['C/O'],fastperiod=19,slowperiod=39,signalperiod=9)[0]

    if check_cdrr('ATR',cdrr,columns):
        ohlc['ATR'] = talib.ATR(ohlc['High'],ohlc['Low'],ohlc['Close'],timeperiod=14)

    if check_cdrr('ATR/O',cdrr,columns):
        ohlc['ATR/O'] = talib.ATR(ohlc['H/O'],ohlc['L/O'],ohlc['O/O'],timeperiod=14)

    if check_cdrr('RSI',cdrr,columns):
        ohlc['RSI'] = talib.RSI(ohlc['Close'],timeperiod=14)

    if check_cdrr('Time',cdrr,columns):
        #time relation
        tr = {
                '60':[60,60],
                '300':[300,12],
                '3600':[3600,24],
                }
        period = str(int(ohlc['timestamp'].diff().mode()))
        ohlc['Time'] = ohlc['timestamp']//tr[period][0]%tr[period][1]

    if check_cdrr('Over',cdrr,columns):
        ohlc['Over'] = ((ohlc['Open']*(1+over_rate) <= ohlc['High']) * (ohlc['Open']*(1-over_rate) >= ohlc['Low'])).astype(bool)

    if check_cdrr('1moku-base',cdrr,columns):
        ohlc['1moku-base'] = (ohlc['High'].rolling(26).max()+ohlc['Low'].rolling(26).min())/2

    if check_cdrr('1moku-conversion',cdrr,columns):
        ohlc['1moku-conversion'] = (ohlc['High'].rolling(9).max()+ohlc['Low'].rolling(9).min())/2

    if check_cdrr('1moku-precedent-span1',cdrr,columns):
        ohlc['1moku-precedent-span1'] = ((ohlc['1moku-base']+ohlc['1moku-conversion'])/2).shift(26)

    if check_cdrr('1moku-precedent-span2',cdrr,columns):
        ohlc['1moku-precedent-span2'] = ((ohlc['High'].rolling(52).max()+ohlc['Low'].rolling(52).min())/2).shift(26)

    if check_cdrr('1moku-late-span',cdrr,columns):
        ohlc['1moku-late-span'] = ohlc['Close'].shift(-26)

    if check_cdrr('1moku-signal1',cdrr,columns):
        ohlc['1moku-signal1'] = ((ohlc['1moku-conversion']-ohlc['1moku-base'] > 0) *1).diff(1)

    if check_cdrr('1moku-signal2',cdrr,columns):
        ohlc['1moku-signal2'] = (ohlc['1moku-conversion']-ohlc['1moku-base'] > 0) *1

    if check_cdrr('1moku-signal3',cdrr,columns):
        ohlc['1moku-signal3'] = (((ohlc['Close'] > ohlc['1moku-precedent-span1'])*1) * ((ohlc['Close'] > ohlc['1moku-precedent-span2'])*1)).diff(1)

    if check_cdrr('1moku-signal4',cdrr,columns):
        ohlc['1moku-signal4'] = ((ohlc['Close'] > ohlc['1moku-precedent-span1'])*1) * ((ohlc['Close'] > ohlc['1moku-precedent-span2'])*1)

    if check_cdrr('1moku-signal5',cdrr,columns):
        ohlc['1moku-signal5'] = (((ohlc['Close'] < ohlc['1moku-precedent-span1'])*1) * ((ohlc['Close'] < ohlc['1moku-precedent-span2'])*1)).diff(1)

    if check_cdrr('1moku-signal6',cdrr,columns):
        ohlc['1moku-signal6'] = ((ohlc['Close'] < ohlc['1moku-precedent-span1'])*1) * ((ohlc['Close'] < ohlc['1moku-precedent-span2'])*1)


    if check_cdrr('Target_Open',cdrr,columns):
        ohlc['Target_Open'] = ohlc['Open'].shift(-1)

    if check_cdrr('Target_High',cdrr,columns):
        ohlc['Target_High'] = ohlc['High'].shift(-1)

    if check_cdrr('Target_Low',cdrr,columns):
        ohlc['Target_Low'] = ohlc['Low'].shift(-1)

    if check_cdrr('Target_Close',cdrr,columns):
        ohlc['Target_Close'] = ohlc['Close'].shift(-1)
        
    if check_cdrr('Target_O/O',cdrr,columns):
        ohlc['Target_O/O'] = ohlc['Target_Open']/ohlc['Open']

    if check_cdrr('Target_H/O',cdrr,columns):
        ohlc['Target_H/O'] = ohlc['Target_High']/ohlc['Open']

    if check_cdrr('Target_L/O',cdrr,columns):
        ohlc['Target_L/O'] = ohlc['Target_Low']/ohlc['Open']

    if check_cdrr('Target_C/O',cdrr,columns):
        ohlc['Target_C/O'] = ohlc['Target_Close']/ohlc['Open']

    if check_cdrr('Target_O/C',cdrr,columns):
        ohlc['Target_O/C'] = ohlc['Target_Open']/ohlc['Close']

    if check_cdrr('Target_H/C',cdrr,columns):
        ohlc['Target_H/C'] = ohlc['Target_High']/ohlc['Close']

    if check_cdrr('Target_L/C',cdrr,columns):
        ohlc['Target_L/C'] = ohlc['Target_Low']/ohlc['Close']

    if check_cdrr('Target_C/C',cdrr,columns):
        ohlc['Target_C/C'] = ohlc['Target_Close']/ohlc['Close']

    if check_cdrr('Target_Over',cdrr,columns):
        ohlc['Target_Over'] = ohlc['Over'].shift(-1).astype(bool)

    ohlc = ohlc[columns]

    if drop:
        ohlc = ohlc.dropna()
    
    return ohlc
