import pandas as pd
import numpy as np


name = "outer"
suffix = "_track_line.csv"

try:
    # Reading the x and y values from the respective files
    outer_track = pd.read_csv(name+suffix, header=None)
    print(outer_track)

    offset_x = 21921.515
    offset_y = 51752.761
    offset_z = 182.9
    interval = 10
    # xとyの値を抽出する
    x_values = outer_track.iloc[1, :]+offset_x
    y_values = outer_track.iloc[2, :]+offset_y

    # yaw角を計算する
    shifted_x = x_values.shift(-1)
    shifted_y = y_values.shift(-1)
    yaw_values = np.arctan2(shifted_y - y_values, shifted_x - x_values)
    yaw_values = np.degrees(yaw_values)  # ラジアンから度に変換する場合

    print(x_values,y_values)

    # 新しいデータフレームを作成する
    final_dataset = pd.DataFrame({
        'x': x_values,
        'y': y_values,
        'z': offset_z,  # zの値がないため、0を使用
        'yaw': yaw_values
    })

    # 最終行は無効なyaw値を含むため削除する
    final_dataset = final_dataset[::interval-1]

    # 結果を表示する
    print(final_dataset.head())

    # CSVファイルとしてエクスポートする
    export_path = 'converted'+name+suffix
    final_dataset.to_csv(export_path, index=False)

except Exception as e:
    display_outer = None
    error_outer = str(e)

