from urllib.request import urlopen
from urllib.parse import urlencode
from urllib.error import URLError, HTTPError


def get_weather(place):
    api_key = "e13192ffee04075184e8e6347ab1ff9f"
    base_url = 'https://api.openweathermap.org/data/2.5/weather?'
    params =  {'q': place, 'appid': api_key, 'units': 'metric'}
    add_on_url = urlencode(params)
    print(add_on_url)
    full_url = base_url + add_on_url
    try:
        with urlopen(full_url) as response:
            data = response.read().decode('utf-8')
            print(data)

    except (URLError, HTTPError) as e:
        print(f"network error: {e}")


    except Exception as  e:
        print(e)
