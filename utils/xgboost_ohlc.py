import xgboost
from sklearn.model_selection import GridSearchCV


def r_fit(X_train, y_train):
  xgbr = xgboost.XGBRegressor()

  # ハイパーパラメータ探索
  xgbr_cv = GridSearchCV(xgbr, {'max_depth': [2,4,6,8,10], 'n_estimators': [50,100,200]}, verbose=1)
  xgbr_cv.fit(X_train, y_train)

  # 改めて最適パラメータで学習
  xgbr = xgboost.XGBRegressor(**xgbr_cv.best_params_)
  xgbr.fit(X_train, y_train)
  return xgbr


def c_fit(X_train, y_train):
  xgbc = xgboost.XGBClassifier()

  # ハイパーパラメータ探索
  xgbc_cv = GridSearchCV(xgbc, {'max_depth': [2,4,6,8,10], 'n_estimators': [50,100,200]}, verbose=1)
  xgbc_cv.fit(X_train, y_train)

  # 改めて最適パラメータで学習
  xgbc = xgboost.XGBClassifier(**xgbc_cv.best_params_)
  xgbc.fit(X_train, y_train)
  return xgbc