import pybitflyer
from datetime import timedelta, timezone
import dateutil.parser
import datetime
import time
import pandas as pd
import requests

timestamp = int(time.time())
while True:
    df = pd.DataFrame()
    r = requests.get('https://lightchart.bitflyer.com/api/ohlc?symbol=FX_BTC_JPY&period=m&before='+str(int(timestamp)*10**3-1))
    tmp_df = pd.DataFrame(r.json()[::-1])

    df['timestamp'] = tmp_df[0]/10**3
    df['month'] = df['timestamp'].apply(lambda x: (datetime.datetime.fromtimestamp(x)+timedelta(hours=9)).month)
    df['year'] = df['timestamp'].apply(lambda x: (datetime.datetime.fromtimestamp(x)+timedelta(hours=9)).year)
    df['Open'] = tmp_df[1]
    df['High'] = tmp_df[2]
    df['Low'] = tmp_df[3]
    df['Close'] = tmp_df[4]
    df['Volume'] = tmp_df[5]

    while True:
        r = requests.get('https://lightchart.bitflyer.com/api/ohlc?symbol=FX_BTC_JPY&period=m&before='+str(int(df['timestamp'].values[0])*10**3-1))
        time.sleep(10)
        tmp_df = pd.DataFrame(r.json()[::-1])

        df_mini = pd.DataFrame()
        df_mini['timestamp'] = tmp_df[0]/10**3
        df_mini['month'] = df_mini['timestamp'].apply(lambda x: (datetime.datetime.fromtimestamp(x)+timedelta(hours=9)).month)
        df_mini['year'] = df_mini['timestamp'].apply(lambda x: (datetime.datetime.fromtimestamp(x)+timedelta(hours=9)).year)
        df_mini['Open'] = tmp_df[1]
        df_mini['High'] = tmp_df[2]
        df_mini['Low'] = tmp_df[3]
        df_mini['Close'] = tmp_df[4]
        df_mini['Volume'] = tmp_df[5]
        df = pd.concat([df_mini,df],ignore_index=True)

        if df['month'].nunique() >= 2:
            df = df[df['month'] == df['month'].values[-1]]
            timestamp = df['timestamp'].values[0]
            month =str(df['month'].values[0]).zfill(2)
            year = str(df['year'].values[0])
            df = df[['timestamp','Open','High','Low','Close','Volume']]
            df.to_csv('data_raw/'+year+'-'+month+'-'+'min.csv',index=False)
            print(year+'-'+month+'-'+'min.csv')
            break