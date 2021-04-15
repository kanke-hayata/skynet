#!/usr/bin/env python3.6

import pybitflyer
from datetime import timedelta, timezone
import dateutil.parser
import datetime
import time
import pandas as pd 
import requests
import os

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
        time.sleep(15)
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
            #以下の行有効で常に最新dataを取ることにする
            timestamp = int(time.time())
            month =str(df['month'].values[0]).zfill(2)
            year = str(df['year'].values[0])
            df = df[['timestamp','Open','High','Low','Close','Volume']]
            df.to_csv('/root/data/raw/'+year+'-'+month+'-'+'min.csv',index=False)
            print(year+'-'+month+'-'+'min.csv')
            break

"""
import pybitflyer
from datetime import timedelta, timezone
import dateutil.parser
import datetime
import time
import pandas as pd

# -------------------------------------
key = os.environ['API_KEY']
secret = os.environ['API_SECRET']

end_point = 'wss://ws.lightstream.bitflyer.com/json-rpc'

public_channels = ['lightning_executions_FX_BTC_JPY','lightning_board_snapshot_FX_BTC_JPY']
private_channels = ['child_order_events', 'parent_order_events']
# -------------------------------------

pb = pybitflyer.API(api_key=key,api_secret=secret)
executions = pb.executions(product_code='FX_BTC_JPY')
date = (dateutil.parser.parse(executions[-1]['exec_date']) + timedelta(hours=9))
last_id = 10**10
before = time.time()
while True:
    df_day = pd.DataFrame(columns=['start-id','end-id','timestamp','Open','High','Low','Close','Volume','day','month','year'])
    while True:
        df_minute = pd.DataFrame(columns=['id','side','price','size','timestamp','minute','day','month','year'])
        while True:
            time.sleep(0.6)
            r = pb.executions(product_code='FX_BTC_JPY',count=500,before=last_id)
            if type(r) != list:
                print(r)
                time.sleep(60)
            else:
                executions = r[::-1]
                df_executions = pd.DataFrame(executions)
                df_executions['timestamp'] = df_executions['exec_date'].apply(lambda x: int(dateutil.parser.parse(x).timestamp()))
                df_executions['minute'] = df_executions['exec_date'].apply(lambda x: (dateutil.parser.parse(x)+timedelta(hours=9)).minute)
                df_executions['day'] = df_executions['exec_date'].apply(lambda x: (dateutil.parser.parse(x)+timedelta(hours=9)).day)
                df_executions['month'] = df_executions['exec_date'].apply(lambda x: (dateutil.parser.parse(x)+timedelta(hours=9)).month)
                df_executions['year'] = df_executions['exec_date'].apply(lambda x: (dateutil.parser.parse(x)+timedelta(hours=9)).year)
                df_executions = df_executions[['id','side','price','size','timestamp','minute','day','month','year']]
                df_minute = pd.concat([df_executions,df_minute],ignore_index=True)

                last_id = df_minute['id'].values[0]

                if df_minute['minute'].nunique() >= 2:
                    df_minute = df_minute[df_minute['minute'] == df_minute['minute'].values[-1]]
                    #無駄に捨てすぎないように再利用
                    last_id = df_minute['id'].values[0]
                    minute_record = pd.DataFrame({'start-id':[df_minute['id'].values[0]],
                                                                             'end-id':[df_minute['id'].values[-1]],
                                                                             'timestamp':[int(df_minute['timestamp'].mean())-(int(df_minute['timestamp'].mean())%60)],
                                                                             'Open':[df_minute['price'].values[0]],
                                                                             'High':[df_minute['price'].max()],
                                                                             'Low':[df_minute['price'].min()],
                                                                             'Close':[df_minute['price'].values[-1]],
                                                                            'Volume':[df_minute['size'].sum()],
                                                                            'day':[df_minute['day'].values[0]],
                                                                            'month':[df_minute['month'].values[0]],
                                                                            'year':[df_minute['year'].values[0]]})
                    df_day = pd.concat([minute_record,df_day],ignore_index=True)
                    break
        
        print(df_day.shape)
        
        if df_day['day'].nunique() >= 2:
            df_day = df_day[df_day['day'] == df_day['day'].values[-1]]
            last_id = df_day['start-id'].values[0]
            day = str(df_day['day'].values[0]).zfill(2)
            month =str(df_day['month'].values[0]).zfill(2)
            year = str(df_day['year'].values[0])
            df_day = df_day[['start-id','end-id','timestamp','Open','High','Low','Close','Volume']]
            df_day.to_csv(year+'-'+month+'-'+day+'-min.csv',index=False)
            print(year+'-'+month+'-'+day)

            # 常に最新のtick取得するための一行 過去データ取るときにはコメントアウト
            last_id = 10**10

            break
"""
