#!/usr/bin/env python3.6

import os

import pybitflyer
from datetime import timedelta, timezone
import dateutil.parser
import datetime
import time
import pandas as pd

# -------------------------------------
key = os.environ['API_KEY']
secret = os.environ['API_SECRET']
# -------------------------------------
### executions を UTC で保存
pb = pybitflyer.API(api_key=key,api_secret=secret)
last_id = 10**10
df = pd.DataFrame()
cnt = 0
while True:
    time.sleep(0.6)
    r = pb.executions(product_code='FX_BTC_JPY',count=500,before=last_id)
    if type(r) != list:
        print(r)
        time.sleep(60)
    else:
        executions = pd.DataFrame(r[::-1])
        df = pd.concat([executions,df],ignore_index=True)
        last_id = df['id'].values[0]

        if cnt % 100 == 0:
            df.to_csv("/root/data/raw/executions.csv",index=False)

    cnt += 1

