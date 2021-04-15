import xgboost
from sklearn.model_selection import GridSearchCV
import pickle
import datetime

import utils


def r_fit(X_train, y_train):
  xgbr = xgboost.XGBRegressor()

  # ハイパーパラメータ探索
  xgbr_cv = GridSearchCV(xgbr, {'max_depth': [2,4,6,8,10,12], 'eta': [0.05,0.1,0.3]}, verbose=1)
  xgbr_cv.fit(X_train, y_train)

  # 改めて最適パラメータで学習
  xgbr = xgboost.XGBRegressor(**xgbr_cv.best_params_)
  xgbr.fit(X_train, y_train)
  return xgbr


def c_fit(X_train, y_train,over_fit=False):
  xgbc = xgboost.XGBClassifier()

  # ハイパーパラメータ探索
  if not over_fit:
      xgbc_cv = GridSearchCV(xgbc, {'max_depth': [2,4,6,8,10,12], 'eta': [0.05,0.1,0.3]}, verbose=1)
      xgbc_cv.fit(X_train, y_train)
  elif over_fit:
      xgbc_cv = GridSearchCV(xgbc, {'max_depth': [10], 'eta': [0.001]}, verbose=1)
      xgbc_cv.fit(X_train, y_train)


  # 改めて最適パラメータで学習
  xgbc = xgboost.XGBClassifier(**xgbc_cv.best_params_)
  xgbc.fit(X_train, y_train)
  return xgbc


def r_model_generate(version,ohlc,data_columns,target_column,data_size,test_size=1000,save_model=True,save_ticks=False,return_model=True,return_result=True,return_data=True):

    train = ohlc[-data_size:-test_size]
    test = ohlc[-test_size:]
    backtest = ohlc[-test_size:]
    all_data = ohlc[-data_size:]

    data_train = train[data_columns]
    data_test = test[data_columns]

    target_train = train[target_column]
    target_test = test[target_column]

    xgbr = r_fit(data_train, target_train)

    predict = xgbr.predict(data_test)

    xgbr_all_data = r_fit(all_data[data_columns],all_data[target_column])

    if save_model:
        pickle.dump(xgbr_all_data, open("skynet/bf/model/T_"+version+"/xgbr.pickle", "wb"))
        with open("skynet/bf/model/T_"+version+"/spec.txt", mode="w") as txt:
            txt.write(str(datetime.datetime.now())+'\n')
            txt.write('data_columns: '+str(data_columns)+"\n")
            txt.write('target_column: '+str(target_column)+"\n")

    if save_ticks:
        train.to_csv("skynet/bf/model/T_"+version+"/train.csv")
        test.to_csv("skynet/bf/model/T_"+version+"/test.csv")
        backtest.to_csv("skynet/bf/model/T_"+version+"/backtest.csv")

    if return_model and return_result and return_data:
        return xgbr, predict, target_test, all_data
    elif return_model:
        return xgbr
    elif return_result:
        return predict, target_test

def c_model_generate(version,ohlc,data_columns,target_column,data_size,test_size=1000,save_model=True,save_ticks=False,return_model=True,return_result=True,return_data=True,over_fit=False):

    train = ohlc[-data_size:-test_size]
    test = ohlc[-test_size:]
    backtest = ohlc[-test_size:]
    all_data = ohlc[-data_size:]

    data_train = train[data_columns]
    data_test = test[data_columns]

    target_train = train[target_column]
    target_test = test[target_column]

    xgbc = c_fit(data_train, target_train, over_fit=over_fit)

    predict = xgbc.predict(data_test)
    print((target_test == predict).mean())

    xgbc_all_data = c_fit(all_data[data_columns],all_data[target_column],over_fit=over_fit)

    if save_model:
        pickle.dump(xgbc_all_data, open("skynet/bf/model/T_"+version+"/xgbc.pickle", "wb"))
        with open("skynet/bf/model/T_"+version+"/spec.txt", mode="w") as txt:
            txt.write(str(datetime.datetime.now())+'\n')
            txt.write('data_columns: '+str(data_columns)+"\n")
            txt.write('target_column: '+str(target_column)+"\n")

    if save_ticks:
        train.to_csv("skynet/bf/model/T_"+version+"/train.csv")
        test.to_csv("skynet/bf/model/T_"+version+"/test.csv")
        backtest.to_csv("skynet/bf/model/T_"+version+"/backtest.csv")

    if return_model and return_result and return_data:
        return xgbc, predict, target_test, all_data
    elif return_model:
        return xgbc
    elif return_result:
        return predict, target_test





