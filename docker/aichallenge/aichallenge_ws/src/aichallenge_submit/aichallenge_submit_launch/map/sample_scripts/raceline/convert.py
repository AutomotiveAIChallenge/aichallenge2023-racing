import pandas as pd
import math
# ファイルパス
file_path = 'updated_shader_index.csv'

# ファイルの読み込み
shader_index_df = pd.read_csv(file_path)

# 最初の数行を表示してデータを確認
shader_index_df.head()

df_swapped = shader_index_df.copy()
shader_index_df['x'], shader_index_df['y'] = df_swapped['y'], df_swapped['x']

# オフセット値の設定
offset_x = 21921.515
offset_y = 51752.761
offset_z = 182.9 - 75
yaw_offset = 90.0*math.pi/180
# 各列にオフセットを適用
shader_index_df['x'] += offset_x
shader_index_df['y'] += offset_y
shader_index_df['z'] += 182.9
shader_index_df['yaw'] = yaw_offset - shader_index_df['yaw']

# 変更後のデータの最初の数行を表示
print(shader_index_df.head())
# CSVファイルとしてエクスポートする
export_path = 'converted.csv'
shader_index_df.to_csv(export_path, index=False)
