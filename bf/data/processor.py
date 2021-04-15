import pandas as pd
import os
import talib

timeaxis = {
        'min':[60,0,60],
        'min_five':[300,60,12],
        'hour':[3600,300,24],
        'day':[3600*24,3600,7]
        }

for ta in timeaxis.keys():
    if ta == 'min':
        files = []
        for f in os.listdir('raw/'):
            if '-min' and 'csv' in f:
                files.append('raw/'+f)
        files.sort()

        print(files)

        df = pd.DataFrame()
        for f in files:
            tmp = pd.read_csv(f)
            df = pd.concat([df,tmp])

        ##### なぜかずれているのでここで誤差解消 #####
        df['timestamp'] = df['timestamp'] + 60
        ##########
        df['UP'] = (df['Open'] <= df['Close'])*2 -1
           
    else:
        bins = list(range(int(before['timestamp'].values[0]-(before['timestamp'].values[0]-timeaxis[ta][1])%timeaxis[ta][0]),int(before['timestamp'].values[-1]-(before['timestamp'].values[-1]-timeaxis[ta][1])%timeaxis[ta][0]+timeaxis[ta][0]*2),timeaxis[ta][0]))
        before['bins'] = pd.cut(before['timestamp'],bins,right=False)
        grouped = before.groupby('bins')
        df = pd.DataFrame()
        df['timestamp'] = grouped['timestamp'].last()
        df['Open'] = grouped['Open'].first()
        df['High'] = grouped['High'].max()
        df['Low'] = grouped['Low'].min()
        df['Close'] = grouped['Close'].last()
        df['Volume'] = grouped['Volume'].sum()
        df['FirstUP'] = grouped['UP'].first()
        df = df.reset_index(drop=True)
        df['UP'] = (df['Open'] <= df['Close'])*2 -1
        df['first_last_samedir'] = (df['UP'] == df['FirstUP'])*2 -1
    # if end
    df['Time'] = df['timestamp']//timeaxis[ta][0]%timeaxis[ta][2]
    df['Open'] = df['Open'].interpolate()
    df['Open'] = df['Open'].fillna(df['Open'].dropna().values[0])
    df['High'] = df['High'].interpolate()
    df['High'] = df['High'].fillna(df['High'].dropna().values[0])
    df['Low'] = df['Low'].interpolate()
    df['Low'] = df['Low'].fillna(df['Low'].dropna().values[0])
    df['Close'] = df['Close'].interpolate()
    df['Close'] = df['Close'].fillna(df['Close'].dropna().values[0])

    df['O/O'] = df['Open']/df['Open']
    df['H/O'] = df['High']/df['Open']
    df['L/O'] = df['Low']/df['Open']
    df['C/O'] = df['Close']/df['Open']
    df['O/C'] = df['Open']/df['Close']
    df['H/C'] = df['High']/df['Close']
    df['L/C'] = df['Low']/df['Close']
    df['C/C'] = df['Close']/df['Close']

    df['H-L/O'] = df['H/O']-df['L/O']
    df['H-L/C'] = df['H/C']-df['L/C']

    tick_range = 12
    for i in range(1,tick_range+1):
        df['O-'+str(i)+'/C'] = df['Open'].shift(i)/df['Close']
        df['H-'+str(i)+'/C'] = df['High'].shift(i)/df['Close']
        df['L-'+str(i)+'/C'] = df['Low'].shift(i)/df['Close']
        df['C-'+str(i)+'/C'] = df['Close'].shift(i)/df['Close']

    df['3SignLine'] = df['UP'].rolling(3).sum()
    df['6SignLine'] = df['UP'].rolling(6).sum()
    df['12SignLine'] = df['UP'].rolling(12).sum()

    df['MACDS'] = talib.MACD(df['Close'],fastperiod=6,slowperiod=19,signalperiod=9)[0]
    df['MACDM'] = talib.MACD(df['Close'],fastperiod=12,slowperiod=26,signalperiod=9)[0]
    df['MACDL'] = talib.MACD(df['Close'],fastperiod=19,slowperiod=39,signalperiod=9)[0]
    df['MACDS/O'] = talib.MACD(df['C/O'],fastperiod=6,slowperiod=19,signalperiod=9)[0]
    df['MACDM/O'] = talib.MACD(df['C/O'],fastperiod=12,slowperiod=26,signalperiod=9)[0]
    df['MACDL/O'] = talib.MACD(df['C/O'],fastperiod=19,slowperiod=39,signalperiod=9)[0]


    df['ATR'] = talib.ATR(df['High'],df['Low'],df['Close'],timeperiod=14)
    df['ATR/O'] = talib.ATR(df['H/O'],df['L/O'],df['O/O'],timeperiod=14)

    df['RSI'] = talib.RSI(df['Close'],timeperiod=14)

    df['1moku-base'] = (df['High'].rolling(26).max()+df['Low'].rolling(26).min())/2
    df['1moku-conversion'] = (df['High'].rolling(9).max()+df['Low'].rolling(9).min())/2
    df['1moku-precedent-span1'] = ((df['1moku-base']+df['1moku-conversion'])/2).shift(26)
    df['1moku-precedent-span2'] = ((df['High'].rolling(52).max()+df['Low'].rolling(52).min())/2).shift(26)
    #df['1moku-late-span'] = df['Close'].shift(-26)
    df['1moku-signal1'] = ((df['1moku-conversion']-df['1moku-base'] > 0) *1).diff(1)
    df['1moku-signal2'] = (df['1moku-conversion']-df['1moku-base'] > 0) *1
    df['1moku-signal3'] = (((df['Close'] > df['1moku-precedent-span1'])*1) * ((df['Close'] > df['1moku-precedent-span2'])*1)).diff(1)
    df['1moku-signal4'] = ((df['Close'] > df['1moku-precedent-span1'])*1) * ((df['Close'] > df['1moku-precedent-span2'])*1)
    df['1moku-signal5'] = (((df['Close'] < df['1moku-precedent-span1'])*1) * ((df['Close'] < df['1moku-precedent-span2'])*1)).diff(1)
    df['1moku-signal6'] = ((df['Close'] < df['1moku-precedent-span1'])*1) * ((df['Close'] < df['1moku-precedent-span2'])*1)

    df['Target_Open'] = df['Open'].shift(-1)
    df['Target_High'] = df['High'].shift(-1)
    df['Target_Low'] = df['Low'].shift(-1)
    df['Target_Close'] = df['Close'].shift(-1)
    df['Target_O/O'] = df['Target_Open']/df['Open']
    df['Target_H/O'] = df['Target_High']/df['Open']
    df['Target_L/O'] = df['Target_Low']/df['Open']
    df['Target_C/O'] = df['Target_Close']/df['Open']
    df['Target_H/C'] = df['Target_High']/df['Close']
    df['Target_L/C'] = df['Target_Low']/df['Close']
    df['Target_C/C'] = df['Target_Close']/df['Close']
    df['Target_UP'] = (df['Target_Open'] <= df['Target_Close'])*2-1

    df.to_csv('processed/'+ta+'.csv',index=False)
    print(ta+' done.')
    before = df
