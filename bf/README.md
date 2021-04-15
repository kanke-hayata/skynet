# Interface for bit-flyer

1. Get API-KEY and API-SECRET and make your .env file.

ex)
```
$ cd skynet/bf
$ touch .env
```

```
[.env]
API_KEY=aaabbbcccddd
API_SECRET=eeefffggghhh
```

2. Make your bot.  
You can take 'skynet/bf/src/scripts/sample.py' for a model.
All you have to change is files under '/scripts' and docker-compose.yml

3. Bring up your bot.

ex)
```
$ cd skynet/bf
$ docker-compose up -d sample
```

