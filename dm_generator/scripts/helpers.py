import os
import time
import re
import json
import rospkg
from xml.etree.ElementTree import Element, parse


# Constants
PACKAGE_PATH = rospkg.RosPack().get_path('dm_generator')
AUTHORIZATION_PATH = os.path.join(PACKAGE_PATH, 'keys')
CONFIGURATION_PATH = os.path.join(PACKAGE_PATH, 'configuration.json')
SCENARIO_PATH = os.path.join(PACKAGE_PATH, 'scenarios')


# Load configuration
with open(CONFIGURATION_PATH) as json_file:
    configuration = json.load(json_file)


def get_module_configuration(module_name: str) -> dict:
    return configuration['modules'][module_name]


# Timestamp string
def timestamp() -> str:
    return str(time.time())


# Check message language
def is_hangul(text: str) -> bool:
    hangul_count = len(re.findall(u'[\u3130-\u318F\uAC00-\uD7A3]+', text))
    return hangul_count > 0


# Reverse AM:PM
def reverse_time(time: str) -> str:
    if not time:
        return None

    hour = int(time[0:2])
    min = int(time[3:5])

    # to PM
    if hour < 12:
        return f'{hour + 12}:{min}'

    # to AM (zero padding)
    elif hour - 12 < 10:
        return f'0{hour - 12}:{min}'

    # to AM
    return f'{hour - 12}:{min}'


class XMLParser:
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.tree = parse(file_path)

    @property
    def root(self) -> Element:
        return self.tree.getroot()
