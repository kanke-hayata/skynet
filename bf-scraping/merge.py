import pandas as pd
import os

files = []
for file in os.listdir('data_raw/'):
    if '-min' and 'csv' in file:
        files.append('data_raw/'+file)
files.sort()

min_tickers = pd.DataFrame()
for f in files:
    df = pd.read_csv(f)
    min_tickers = pd.concat([min_tickers,df])
    
min_tickers.to_csv('data_processed/min.csv',index=False)