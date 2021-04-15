#!/usr/bin/env python3.6

import os

import discord
import pandas as pd
import matplotlib.pyplot as plt
import rosnode

def node_is_up(model):
    flag = False
    for node in rosnode.get_node_names():
        if model in node:
            flag = True

    return flag

# 自分のBotのアクセストークンに置き換えてください
TOKEN = os.environ['DISCORD_BOT_TOKEN']

# 接続に必要なオブジェクトを生成
client = discord.Client()

# 起動時に動作する処理
@client.event
async def on_ready():
    # 起動したらターミナルにログイン通知が表示される
    print('Login')

# メッセージ受信時に動作する処理
@client.event
async def on_message(message):
    if message.author.bot:
        return
    if message.content == 'log':
        for mod in os.listdir('/root/model'):
            if os.path.exists('/root/model/'+mod+'/log.csv') and node_is_up(str(mod)):
                log = pd.read_csv('/root/model/'+mod+'/log.csv')
                if not log.empty:
                    plt.title(mod)
                    plt.fill_between(log['timestamp'],0,log['income'],where=0<log['income'],color='#87cefa',interpolate=True)
                    plt.fill_between(log['timestamp'],0,log['income'],where=0>log['income'],color='#ff6347',interpolate=True)
                    plt.savefig('/root/model/'+mod+'/PLgraph.png')
                    plt.close()
                    await message.channel.send(content=mod,file=discord.File("/root/model/"+mod+"/PLgraph.png","PLgraph.png"))

# Botの起動とDiscordサーバーへの接続
client.run(TOKEN)
