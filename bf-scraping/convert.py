import pandas as pd

min_tickers = pd.read_csv('data_processed/min.csv')

bins = list(range(int(min_tickers['timestamp'].values[0]),int(min_tickers['timestamp'].values[-1]+300),300))
min_tickers['bins'] = pd.cut(min_tickers['timestamp'],bins,right=False)
grouped = min_tickers.groupby('bins')
tickers_5min = pd.DataFrame()
tickers_5min['timestamp'] = grouped['timestamp'].first()
tickers_5min['Open'] = grouped['Open'].first()
tickers_5min['High'] = grouped['High'].max()
tickers_5min['Low'] = grouped['Low'].min()
tickers_5min['Close'] = grouped['Close'].last()
tickers_5min['Volume'] = grouped['Volume'].sum()

tickers_5min = tickers_5min.reset_index(drop=True)

tickers_5min.to_csv('data_processed/5min.csv',index=False)