import os
import time
import re
import json
import rospkg


# Constants
PACKAGE_PATH = rospkg.RosPack().get_path('dm_generator')
AUTHORIZATION_PATH = os.path.join(PACKAGE_PATH, 'keys')
CONFIGURATION_PATH = os.path.join(PACKAGE_PATH, 'configuration.json')
SCENARIO_PATH = os.path.join(PACKAGE_PATH, 'scenarios')


# Load configuration
with open(CONFIGURATION_PATH) as json_file:
    configuration = json.load(json_file)


def get_module_configuration(module_name: str):
    return configuration['modules'][module_name]


# Authorize Google API
def authorize_credentials(key_file: str):
    key_path = os.path.join(AUTHORIZATION_PATH, key_file)
    os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = key_path


# Timestamp string
def timestamp():
    return str(time.time())


# Check message language
def is_hangul(text: str):
    hangul_count = len(re.findall(u'[\u3130-\u318F\uAC00-\uD7A3]+', text))
    return hangul_count > 0
