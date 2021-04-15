#!/usr/bin/env python3.6

import datetime
import time
#security的にもkeyとsecretは環境変数にして一括管理しないと問題が起きそう
import os

import numpy as np
import pandas as pd
import dateutil.parser
import pybitflyer
import rospy

class Bot(object):
  def __init__(self,balance=0.0,name=''):
    key = os.environ['API_KEY']
    secret = os.environ['API_SECRET']
    self.pbf = pybitflyer.API(api_key=key,api_secret=secret)
    self.side = 'no-position'
    self.positions = pd.DataFrame(columns=['price','size'])
    self.balance = balance
    self.income = 0
    self.child_orders = pd.DataFrame(columns=['child_order_id','expire_date','size','executed'])
    self.parent_orders = pd.DataFrame(columns=['parent_order_id'])
    self.child_acceptance_ids = []
    self.parent_acceptance_ids = []

    self.mid_price = 0
    self.bids = pd.DataFrame(columns = ['price','size'])
    self.asks = pd.DataFrame(columns = ['price','size'])

    #汎用 取引量抑えたりに使える
    self.cnt = 0

    self.name = name

    with open("/root/model/"+self.name+"/log.csv",mode="w") as logcsv:
        logcsv.write('timestamp,income\n')

  def market_buy(self,size,minutes_to_expire=43200,time_in_force="GTC"):
    result = self.pbf.sendchildorder(
      product_code='FX_BTC_JPY',
      child_order_type='MARKET',
      side='BUY',
      size=round(size,3),
      minutes_to_expire=minutes_to_expire,
      time_in_force=time_in_force
    )
    if type(result) == dict and 'child_order_acceptance_id' in result.keys():
      self.child_acceptance_ids.append(result['child_order_acceptance_id'])
    else:
      print("market_buy failed")
      print(result)

  def market_sell(self,size,minutes_to_expire=43200,time_in_force="GTC"):
    result = self.pbf.sendchildorder(
      product_code='FX_BTC_JPY',
      child_order_type='MARKET',
      side='SELL',
      size=round(size,3),
      minutes_to_expire=minutes_to_expire,
      time_in_force=time_in_force
    )
    if type(result) == dict and 'child_order_acceptance_id' in result.keys():
      self.child_acceptance_ids.append(result['child_order_acceptance_id'])
    else:
      print("market_sell failed")
      print(result)

  def limit_buy(self,price,size,minutes_to_expire=43200,time_in_force="GTC"):
    result = self.pbf.sendchildorder(
      product_code='FX_BTC_JPY',
      child_order_type='LIMIT',
      side='BUY',
      price=int(price),
      size=round(size,3),
      minutes_to_expire=minutes_to_expire,
      time_in_force=time_in_force
    )
    if type(result) == dict and 'child_order_acceptance_id' in result.keys():
      self.child_acceptance_ids.append(result['child_order_acceptance_id'])
    else:
      print("limit_buy failed")
      print(result)

  def limit_sell(self,price,size,minutes_to_expire=43200,time_in_force="GTC"):
    result = self.pbf.sendchildorder(
      product_code='FX_BTC_JPY',
      child_order_type='LIMIT',
      side='SELL',
      price=int(price),
      size=round(size,3),
      minutes_to_expire=minutes_to_expire,
      time_in_force=time_in_force
    )
    if type(result) == dict and 'child_order_acceptance_id' in result.keys():
      self.child_acceptance_ids.append(result['child_order_acceptance_id'])
    else:
      print("limit_sell failed")
      print(result)

  def oco(self,order_set='',minute_to_expire=43200,p1=0,s1=0,side1='',p2=0,s2=0,side2=''):
    if order_set == 'ls':
      result = self.pbf.sendparentorder(
            order_method =  "OCO",
            minute_to_expire = minute_to_expire,
            time_in_force = "GTC",
            parameters = [{
              "product_code": "FX_BTC_JPY",
              "condition_type": "LIMIT",
              "side": side1,
              "price": float(p1),
              "size": s1
            },
            {
              "product_code": "FX_BTC_JPY",
              "condition_type": "STOP",
              "side": side2,
              "trigger_price": float(p2),
              "size": s2
            }]
      )
    if type(result) == dict and 'parent_order_acceptance_id' in result.keys():
      self.parent_acceptance_ids.append(result['parent_order_acceptance_id'])
    else:
      print("oco failed")
      print(result)

  def ifdoco(self,order_set='',minute_to_expire=43200,p1=0,s1=0,side1='',p2=0,s2=0,side2='',p3=0,s3=0,side3=''):
    if order_set == 'lls':
      result = self.pbf.sendparentorder(
            order_method =  "IFDOCO",
            minute_to_expire = minute_to_expire,
            time_in_force = "GTC",
            parameters = [{
              "product_code": "FX_BTC_JPY",
              "condition_type": "LIMIT",
              "side": side1,
              "price": float(p1),
              "size": s1
            },
            {
              "product_code": "FX_BTC_JPY",
              "condition_type": "LIMIT",
              "side": side2,
              "price": float(p2),
              "size": s2
            },
            {
              "product_code": "FX_BTC_JPY",
              "condition_type": "STOP",
              "side": side3,
              "trigger_price": float(p3),
              "size": s3
            }]
      )
    if type(result) == dict and 'parent_order_acceptance_id' in result.keys():
      self.parent_acceptance_ids.append(result['parent_order_acceptance_id'])
    else:
      print("ifdoco failed")
      print(result)

  def cancel_all_child_orders(self):
      for child_order_id in self.child_orders['child_order_id'].values:
          result = self.pbf.cancelchildorder(product_code="FX_BTC_JPY",child_order_id=child_order_id)
          print(result)

  def settlement_positions(self):
      size_sum = 0
      if self.side == "BUY":
          size_sum = round(self.positions['size'].sum()*10**-8,3)
          self.market_sell(size=size_sum)
      elif self.side == "SELL":
          size_sum = round(self.positions['size'].sum()*10**-8,3)
          self.market_buy(size=size_sum)
      

  def check_expire_date(self):
    now = datetime.datetime.now()
    self.child_orders = self.child_orders[self.child_orders['expire_date'].dt.to_pydatetime() <= now]

  def callback_child_order_events(self,data):
    if data.event_type == 'ORDER':
      #発注時の処理が追いつかずにacceptance_idが空になっている可能性があるのでsleep(不要かも)
      time.sleep(0.01)
      if data.child_order_acceptance_id in self.child_acceptance_ids:
        self.child_acceptance_ids.remove(data.child_order_acceptance_id)
        expire_date = dateutil.parser.parse(data.expire_date)
        satoshi = round(round(data.size,8)*10**8)
        order = pd.Series([data.child_order_id,expire_date,satoshi,0],index=['child_order_id','expire_date','size','executed'])
        self.child_orders = self.child_orders.append(order,ignore_index=True)

    elif data.event_type == 'EXECUTION' and data.child_order_id in self.child_orders['child_order_id'].values:

      #実行された分を加算
      satoshi = round(round(data.size,8)*10**8)
      self.child_orders.loc[self.child_orders['child_order_id'] == data.child_order_id,'executed'] += satoshi
      #注文の全量が約定されたものは注文から排除
      self.child_orders = self.child_orders[np.logical_not(self.child_orders['size'] == self.child_orders['executed'])]

      #set position
      if self.side == 'no-position':
        self.side = data.side
        price_size = pd.Series([data.price,satoshi],index=['price','size'])
        self.positions = self.positions.append(price_size,ignore_index=True)

      # buy more or sell more
      elif data.side == self.side:
        price_size = pd.Series([data.price,satoshi],index=['price','size'])
        self.positions = self.positions.append(price_size,ignore_index=True)

      # switch side
      elif data.side != self.side:
        # completely settlement
        if satoshi == self.positions['size'].sum():
          self.income += (1 if self.side == 'BUY' else -1)*(((data.price - self.positions['price']) * self.positions['size']).sum())*(10**-8)
          self.side = 'no-position'
          self.positions = pd.DataFrame(columns=['price','size'])
        # partly settlement
        elif satoshi < self.positions['size'].sum():
          self.positions['cumsum'] = self.positions['size'].cumsum()
          self.positions['cumsum2'] = self.positions['cumsum']-self.positions['size']
          pos = self.positions[self.positions['cumsum2'] < satoshi].reset_index(drop=True)
          pos.at[pos.shape[0]-1,'size'] = satoshi - pos.at[pos.shape[0]-1,'cumsum2']
          self.income += (1 if self.side == 'BUY' else -1)*(((data.price-pos['price'])*pos['size']).sum())*(10**-8)

          self.positions = self.positions[self.positions['cumsum'] > satoshi].reset_index(drop=True)
          self.positions.at[0,'size'] = self.positions.at[0,'cumsum'] - satoshi
          self.positions = self.positions[['price','size']]
        # settlement and setting position
        else:
          self.income += (1 if self.side == 'BUY' else -1)*(((data.price - self.positions['price']) * self.positions['size']).sum())*(10**-8)
          self.side = data.side
          price_size = pd.Series([data.price,satoshi-self.positions['size'].sum()],index=['price','size'])
          self.positions = pd.DataFrame(columns=['price','size'])
          self.positions = self.positions.append(price_size,ignore_index=True)

    elif data.event_type in ['CANCEL','EXPIRE','ORDER_FAILED']:
      self.child_orders = self.child_orders[self.child_orders['child_order_id'] != data.child_order_id]
  
  def callback_parent_order_events(self,data):
    #rospy.loginfo(data)
    if data.event_type == 'ORDER':
      #発注時の処理が追いつかずにacceptance_idが空になっている可能性があるのでsleep(不要かも)
      time.sleep(0.05)
      if data.parent_order_acceptance_id in self.parent_acceptance_ids:
        self.parent_acceptance_ids.remove(data.parent_order_acceptance_id)
        expire_date = dateutil.parser.parse(data.expire_date)
        parent_order = pd.Series([data.parent_order_id,expire_date],index=['parent_order_id','expire_date'])
        self.parent_orders = self.parent_orders.append(parent_order,ignore_index=True)
    elif data.event_type == 'TRIGGER' and data.parent_order_id in self.parent_orders['parent_order_id'].values:
      self.child_acceptance_ids.append(data.child_order_acceptance_id)
    elif data.event_type in ['CANCEL','EXPIRE'] and data.parent_order_id in self.parent_orders['parent_order_id'].values:
      self.parent_orders = self.parent_orders[self.parent_orders['parent_order_id'] != data.parent_order_id]
    elif data.event_type in ['COMPLETE']:
        pass
    else:
      print(data)

  def callback_board_snapshot(self,data):
    self.mid_price = data.mid_price

    self.bids = pd.DataFrame([])
    self.bids['price'] = data.bids_price
    self.bids['size'] = data.bids_size

    self.asks = pd.DataFrame([])
    self.asks['price'] = data.asks_price
    self.asks['size'] = data.asks_size

  def reset(self,balance=0.0):
    self.side = 'no-position'
    self.positions = pd.DataFrame(columns=['price','size'])
    self.balance = balance
    #self.income = 0
    self.child_orders = pd.DataFrame(columns=['child_order_id','expire_date'])
    self.child_acceptance_ids = []

  def pos_vol(self):
    return round(self.positions['size'].sum()*10**-8,8)

  def pos_eval(self):
    if self.mid_price == 0:
      return 0
    else:
      return (1 if self.side == 'BUY' else -1)*(((self.mid_price - self.positions['price']) * self.positions['size']).sum())*(10**-8)

  def vol_in_use(self):
    return round((self.positions['size'].sum()+(self.child_orders['size']-self.child_orders['executed']).sum())*(10**-8),8) 

  def info(self):
    print("#####bot info#####")
    print(datetime.datetime.fromtimestamp(time.time()))
    print('side',self.side)
    print('child_acceptance_ids',self.child_acceptance_ids)
    print('parent_acceptance_ids',self.parent_acceptance_ids)
    print('child_orders',self.child_orders[['size','executed']])
    print('positions',self.positions)
    if self.mid_price == 0:
      tmp = self.income
    else:
      tmp = self.income+(1 if self.side == 'BUY' else -1)*(((self.mid_price - self.positions['price']) * self.positions['size']).sum())*(10**-8)
    print('income',tmp)
    #print("##################")

    with open("/root/model/"+self.name+"/log.csv",mode="a") as logcsv:
        logcsv.write(str(int(time.time()))+','+str(int(tmp))+'\n')


